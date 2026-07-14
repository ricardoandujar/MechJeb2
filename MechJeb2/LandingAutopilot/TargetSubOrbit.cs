using System;
using KSP.Localization;
using MechJebLib.Functions;
using MuMech.LandingAutopilot;
using UnityEngine;
using static MuMech.RCSSolver;

namespace MuMech
{
    namespace Landing
    {
        public class TargetSubOrbit : AutopilotStep
        {
            enum Step { STATE_INIT, STATE_LATERAL, STATE_PROGRADE, STATE_LATERAL_FINE, STATE_RETROGRADE, STATE_CORRECT };
            Step step = Step.STATE_INIT;
            private const float H_CORRECTION_ANGLE_CONSTANT = 0.4f; // Used to determine horizontal correction angle 
            private const double OVERSHOOT_ANGLE_FRACTION = 0.9; // Small margin to avoid losing vertical control
            private const double LIMITED_MAX_THRUST_G_RATIO = 3.125; // this is multiplied with Mainbody g
            private const double TARGET_FAR_DISTANCE_THRESHOLD = 2.5*50000; // Distance to target at which to switch from prograde/retrograde burn to fine lateral control to target before final landing burn. This is used to avoid overshooting the target when doing a prograde or retrograde burn and allow for more precise targeting of the final landing burn.
            private const double TARGET_DISTANCE_THRESHOLD = 300; // Final Distance to target at which to switch to final landing burn.   
            private const double DELTA_TARGET_ALT_THRESHOLD = 600; // Threshold for switching between prograde and retrograde burn when close to target - if the target altitude is above the current altitude by this amount then we will do a prograde burn, if it's below then we will do a retrograde burn, if it's close then we will do a lateral burn to fine tune the approach to target before the final burn.
            private double hFarCorrectionAngle;             // far horizontal distance overshoot angle
            private double velaccum = 0;                    // Integral velocity error accumulator for PI Controller
            private double limitedMaxThrustAccel = 0;       // Limited thrust acceleration used to calculate TWR and overshoot angle + scale max speed
            private double actualLimitedMaxThrustAccel = 0; // Calculate actual limited max thrust used at this time.

            private Vector3d dvSolution = Vector3d.zero;
            private Vector3d deltaVApplied = Vector3d.zero;
            private Vector3d thrustDir = Vector3d.zero;
            // The steepness controls the ratio curvature - When horizontal distance to target is small we
            // dont want to modify the desired vertical velocity.
            // When horizontally far from target we want to maintain altitude, but slower allow vertical drop
            // as approach target.
            float steepness;// Defines the steepness to final descent ratio curvature to target.
            float maxRatio; // Desired Vertical speed is zero at this or larger ratios 
            float minRatio; // Desired Vertical speed is unmodified at this or smaller ratios
            private double vCorrectionAngle = 0.0;  // range is -1.0 to +1.0
            private double vCorrectAngleAccum = 0.0; // Used to accumulate vertical correction angle for smoothing.
            private float lastHorizontalThrust = 0.0f;
            double baseGain; // Calibrated gain for horizontal angle and to correct movement that would diverge from target.
            private double throttleAngle = 45; // <=30 (degrees) Throttle is only applied if pointing in direction of requested attitude
            private double ratio = 0; // Set once to check to see if we need to defer to landing burn

            private MechJebModuleLandingPredictions _predictor; //Landing prediction data:
            private bool forceHoriz = true;
            private double distanceToTarget;
            private double deltaHeight;
            private double tUT;

            private bool throttleOn = false;
            private int throttleCount = 0;
            private int LateralCount = 0;
            private int ProgradeCount = 0;
            private int RetrogradeCount = 0;
            private int LateralFineCount = 0;
            private double minVerticalAngle;

            public TargetSubOrbit(MechJebCore core) : base(core)
            {
                Core.Warp.MinimumWarp();
                _predictor = Core.GetComputerModule<MechJebModuleLandingPredictions>();
                Core.Thrust.ThrustOff();
                //radial = Core.Landing.RadialPercent / 100.0;

                double tempFactor = 20.0; // 20<=10(baseline)<=100<=1000 Allow large angles - if angles are too small there will not be enough horizontal thrust
                double baseGainLo = 0.03136 / (Core.Landing.g); // 0.03136<=0.1568 <= 0.196
                double baseGainHi = 0.4704 / (Core.Landing.g); // 0.1568 <= 0.196
                baseGain = Math.Max(baseGainLo, Math.Min(baseGainHi, baseGainLo + (baseGainHi - baseGainLo) * (Core.Landing.g - 0.05) / (2 - 0.05)));

                // The steepness controls the ratio curvature - When horizontal distance to target is small we
                // dont want to modify the desired vertical velocity.
                // When horizontally far from target we want to maintain altitude, but slower allow vertical drop
                // as approach target.
                steepness = Mathf.Max(10.0f, Core.Landing.BASE_STEEPNESS * (float)Core.Landing.steepness);
                // Ratio ranges
                maxRatio = (float)Core.Landing.maxRatio; // Desired Vertical speed is zero at this or larger ratios 
                minRatio = (float)Core.Landing.minRatio; // Desired Vertical speed is unmodified at this or smaller ratios

                throttleAngle = 30.0; // Allow throttling if within 30 degrees - in final landing angle will be wider.
                limitedMaxThrustAccel = Math.Min(VesselState.limitedMaxThrustAccel, tempFactor * LIMITED_MAX_THRUST_G_RATIO * Core.Landing.g);
                hFarCorrectionAngle = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(limitedMaxThrustAccel, 2) - Math.Pow(Core.Landing.g, 2)) / Core.Landing.g;
                actualLimitedMaxThrustAccel = VesselState.limitedMaxThrustAccel;

                if (VesselState.orbitTimeToAp < 120 )
                {
                    minVerticalAngle = 0;
                }
                else
                {
                    minVerticalAngle = -0.25;
                }

                // If the current altitude is low, then adjust this to 1/3 of the current altitude to avoid overshooting
                if (VesselState.altitudeTrue < 3 * Core.Landing.TgtAlt)
                {
                    Core.Landing.TgtAlt = VesselState.altitudeTrue / 3.0;
                }
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // This is checked once to determine if we need to move on to landing burn
                if (ratio == 0)                    
                {
                    ratio = Core.Landing.getHDistanceToTarget() / VesselState.altitudeTrue;
                    if ( ((ratio < 0.4) && (VesselState.altitudeTrue <= 1.1*Core.Landing.TargetAltitude)) ||
                        (VesselState.altitudeTrue <= 5000) )
                    {
                        Debug.Log("exit TargetSubOrbit to MoveToTarget");
                        Core.Thrust.ThrustOff();
                        return new MoveToTarget(Core);
                    }
                }

                if ( step != Step.STATE_CORRECT )
                {
                    double desiredVerticalSpeed = 0; // for now just set to zero vertical velocity

                    // Calculate Thrust Amount and Direction
                    SetPathToTarget(ref desiredVerticalSpeed, ref thrustDir);

                    // Desired thrust direction in inertial frame
                    Quaternion targetRot = Quaternion.LookRotation(thrustDir.normalized, Vessel.up)
                                         * Quaternion.Euler(0, 0, (float)Core.Landing.vesselAngle);
                    Core.Attitude.attitudeTo(targetRot, AttitudeReference.INERTIAL, this);

                    // Disable thrust when not pointing to target attitude
                    if ((Core.Attitude.attitudeAngleFromTarget() > throttleAngle))
                    {
                        Core.Thrust.RequestActiveThrottle(0);
                        velaccum = 0;
                        if (throttleOn == true)
                        {
                            throttleOn = false;
                            throttleCount++;
                            Debug.Log("step:throttleCount-STOP:" + step+":" + throttleCount + "  angle:" + Core.Attitude.attitudeAngleFromTarget().ToString("F1"));
                        }
                    }
                    else if ( lastHorizontalThrust > 0)
                    {
                        if (throttleOn == false)
                        {
                            throttleOn = true;
                            Debug.Log("step:throttleCount-START:" + step + ":" + throttleCount + "  angle:" + Core.Attitude.attitudeAngleFromTarget().ToString("F1"));
                        }
                        throttleOn = true;
                    }

                    return this;
                }

                Core.Thrust.ThrustOff();
                if ((ProgradeCount >= 4) || (RetrogradeCount >= 4) || (LateralFineCount >= 4))
                {
                    return new MoveToTarget(Core);
                }
                return new LandingBurn(Core);
            }

            // SetPathToTarget 
            /// <summary>
            /// </summary>
            /// <param name="desiredVerticalSpeed"></param>
            /// <param name="desiredThrustVector"></param>
            public void SetPathToTarget(ref double desiredVerticalSpeed, ref Vector3d desiredThrustVector)
            {
                // Get Target position in BODY-FIXED frame
                Core.Landing.ClosestPointToSurfaceTarget(out Vector3d closestPos, out tUT, out distanceToTarget, out Vector3d surfaceVel);
                Vector3d targetPos = Core.Target.GetPositionTargetPosition();
                Core.Landing.GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, tUT, out Vector3d downrangeVec, out Vector3d crossrangeVec, out Vector3d horizontalVec);
                double hTargetError = horizontalVec.magnitude;

                _predictor.debug1MarkerRadius = 10; // GREEN
                _predictor.debug1Lat = MainBody.GetLatitude(closestPos);
                _predictor.debug1Lon = MainBody.GetLongitude(closestPos);
                closestPos = closestPos - MainBody.position; // Convert to body fixed frame for calculations

                // Using the current horizontal velocity and desired horizontal velocity calculate the desired
                // horizontal thrust vector.  An velocity herror vector will be calculated.
                // A speed up would be an herror vector in the direction of target
                // A speed down would be an herror vector opposite in the direction of target.
                float ratio = (float)(hTargetError / VesselState.altitudeTrue);
                float maxverror = 2.0f;   // hysterisis in the negative direction.
                float minverror = -0.1f;  // hysterisis in the positive direction.

                // Set the desired vertical speed - it can be overriden to achieve suborbital flight
                setVerticalSpeed(ref desiredVerticalSpeed, ref ratio, ref minverror, ref maxverror);

                // Set the desired Horizontal Velocity to reach target. Error to subtract the vessels current horizontal velocity.
                double hCorrectionAngle = setHorizontalSpeed(downrangeVec, crossrangeVec, closestPos, targetPos, ref surfaceVel, out Vector3d desiredHorizontalVel, out Vector3d herror);

                // Set the thrust based on primarily vertical herror, with horizontal as an alternate for thrust calculation when vertical is not needed.
                vCorrectionAngle = setThrustAndVerticalAngle(out double verror, ref desiredVerticalSpeed, ref herror, ref desiredHorizontalVel);

                // Set thrust vector based on vertical and horizontal inputs
                setThrustVector(ref desiredThrustVector, herror, hCorrectionAngle, desiredVerticalSpeed, verror);

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status17", // Moving to target: speed(V:<<1>> , H:<<2>>) m/s
                    desiredVerticalSpeed.ToString("F1"), //trace);
                    desiredHorizontalVel.magnitude.ToString("F1"));
            }

            public void setThrustVector(ref Vector3d desiredThrustVector, Vector3d hError, double hCorrectionAngle, double desiredVerticalSpeed, double verror)
            {
                // Calculate desired thrust vector.
                desiredThrustVector = (vCorrectionAngle * VesselState.up + hCorrectionAngle * hError.normalized);  // herror was not normalized before
            }

            public void setVerticalSpeed(ref double desiredVerticalSpeed, ref float ratio, ref float minverror, ref float maxverror)
            {
                // Smooth Function to calculate desired vertical speed.
                float fraction = (ratio >= maxRatio) ? 0.0f : Mathf.Max(0.0f, Mathf.Min(1.0f, (maxRatio - ratio) / (maxRatio - minRatio)));
                if (Double.IsNaN(desiredVerticalSpeed)) return;
                desiredVerticalSpeed = (fraction < 0.0001) ? 0 : -(float)Math.Abs(desiredVerticalSpeed) * ((Mathf.Pow(steepness, fraction) - 1.0f) / (steepness - 1.0f));
                if (Double.IsNaN(desiredVerticalSpeed)) return;
            }

            public double setHorizontalSpeed(Vector3d downrangeVec, Vector3d crossrangeVec, Vector3d closestPos, Vector3d targetPos,
                 ref Vector3d surfaceVel, out Vector3d desiredHorizontalVel, out Vector3d herror)
            {
                double rc = 0;
                double _hCorrectionUpAngle = hFarCorrectionAngle; // Controls Max angle to move horizontally.
                double gainCancelHVelocity = baseGain; // Gain for horizontal velocity correction
                double height = closestPos.magnitude - targetPos.magnitude;
                Vector3d pos = (VesselState.CoM - Vessel.mainBody.position); // Current position vector from body center
                Vector3d targetDistance = targetPos - pos; // Vector from current position to target
                Vector3d closestDistance = closestPos - pos; // Vector from current position to target
                double   deltaDistance = closestDistance.magnitude - targetDistance.magnitude; // Difference in distance to target and closest approach
                deltaHeight = Core.Landing.TgtAlt - height;

                switch (step)
                {
                    default:
                    case Step.STATE_INIT: // Initial run - setup for lateral burn
                        forceHoriz = true; // Force horizontal thrust
                        step = Step.STATE_LATERAL;
                        desiredHorizontalVel = Vector3d.zero;
                        herror =  gainCancelHVelocity * crossrangeVec; // Add Cross range to horizontal error
                        rc = 100; // we only want horizontal thrust here
                        break;
                    // deltaHeight   = -10190 - means vessel travels 10.2km above target altitude at closest approach
                    // deltaDistance =  -1633 - means vessel's closest approach is in front of the target by 1.6km (negative means in front of target, positive means behind target)
                    // crossrangeVec.magnitude = < 500 - means vessel is within 500m of target in crossrange direction
                    case Step.STATE_LATERAL: // Burn lateral to set up for prograde burn
                        forceHoriz = true; // Force horizontal thrust
                        desiredHorizontalVel = Vector3d.zero;
                        herror =  gainCancelHVelocity * crossrangeVec; // Add Cross range to horizontal error
                        if ((throttleCount >= 4)||(crossrangeVec.magnitude < 500))
                        {
                            LateralCount = throttleCount;
                            throttleCount = 0;
                            if ((deltaHeight < -DELTA_TARGET_ALT_THRESHOLD) && (deltaDistance > -2000))
                                step = Step.STATE_RETROGRADE;
                            else if ((deltaHeight > DELTA_TARGET_ALT_THRESHOLD) && (deltaDistance < 2000))
                                step = Step.STATE_PROGRADE;
                            else if (deltaDistance < -1000 )
                                step = Step.STATE_PROGRADE;
                            else if (deltaHeight < -DELTA_TARGET_ALT_THRESHOLD*6)
                                step = Step.STATE_RETROGRADE;
                            else 
                                step = Step.STATE_PROGRADE;
                        }
                        Debug.Log(" lateral deltaHeight:" + deltaHeight.ToString("F1") + "  deltaDistance" + deltaDistance.ToString("F1"));
                        rc = 100; // we only want horizontal thrust here
                        break;
                    case Step.STATE_PROGRADE: // Burn prograde in the direction of the target until close
                        {
                            forceHoriz = false; // use prograde burn

                            // Calculate horizontal error using the orbital velocity relative to the desired orbital velocity to target.
                            double hOrbitalVelocity = Vector3d.Exclude(VesselState.up, VesselState.orbitalVelocity).magnitude; // Get current horizontal orbital velocity
                            double desiredHorizontal = Core.Landing.GetCircularOrbitSpeed(Core.vessel.altitude, Core.vessel.mainBody);
                            desiredHorizontalVel = desiredHorizontal * downrangeVec.normalized;
                            herror = 1.1 * desiredHorizontalVel - hOrbitalVelocity * Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity).normalized;

                            // Reduce the errror as we approach periapsis to avoid over correction
                            herror = 20 * herror * Mathf.Clamp01((float)((0.5 * VesselState.orbitApA - VesselState.orbitPeA) / (0.01 * VesselState.orbitApA))); // stop down range correction at periapsis = 0 and higher

                            Debug.Log(" prograde deltaHeight:" + deltaHeight.ToString("F0") + "  distance:" + distanceToTarget.ToString("F0") + "  deltaDistance" + deltaDistance.ToString("F1"));

                            // Switch to fine lateral control when close to target or if periapsis is above half target altitude
                            if ((throttleCount >= 4) || (Orbit.PeA > 0.5 * Core.Landing.TargetAltitude) || ((distanceToTarget < TARGET_FAR_DISTANCE_THRESHOLD) && (deltaHeight < -DELTA_TARGET_ALT_THRESHOLD)))
                            {
                                ProgradeCount = throttleCount;
                                throttleCount = 0;
                                step = Step.STATE_LATERAL_FINE;
                            }
                            // Calculate horizontal correction angle using horizontal herror.
                            rc = Math.Max(0, Math.Min(_hCorrectionUpAngle, baseGain * H_CORRECTION_ANGLE_CONSTANT * herror.magnitude));
                        }
                        break;
                    case Step.STATE_RETROGRADE:
                        // Closest position is close target - do final retrograde burn to get it to pass close to target altitude
                        {
                            forceHoriz = false; // use retrograde burn

                            double hOrbitalVelocity = Vector3d.Exclude(VesselState.up, VesselState.orbitalVelocity).magnitude; // Get current horizontal orbital velocity
                            double desiredHorizontal = 0.9*hOrbitalVelocity;
                            desiredHorizontalVel = desiredHorizontal * downrangeVec.normalized;

                            // Burn retrograde until height at closest position is close to target altitude, but only if it's crosses at a higher altitude.
                            Debug.Log(" retrograde deltaHeight:" + deltaHeight.ToString("F0") + "  distance:" + distanceToTarget.ToString("F0") + "  deltaDistance" + deltaDistance.ToString("F1"));

                            herror = deltaHeight * downrangeVec.normalized;
                            rc = 100; // we only want horizontal thrust here
                            if ((throttleCount >= 4) || (deltaHeight >= -DELTA_TARGET_ALT_THRESHOLD) )//|| (deltaDistance >= 2000))
                            {
                                RetrogradeCount = throttleCount;
                                throttleCount = 0;
                                // Now will perform final correction step.
                                step = Step.STATE_LATERAL_FINE;
                            }
                        }
                        break;
                    case Step.STATE_LATERAL_FINE: // Fine tune horizontal velocity to target using closest approach vector instead of crossRange vector
                        {
                            forceHoriz = true; // Force horizontal thrust
                            desiredHorizontalVel = Vector3d.zero;

                            herror = calculateHerror(ref closestPos, ref targetPos, ref distanceToTarget, ref surfaceVel);
                            herror = (Vector3d.Angle(VesselState.normalPlus, herror) > 90) ? -VesselState.normalPlus : VesselState.normalPlus;
                            MechJebModuleDebugArrows.debugVector2 = herror;
                            Debug.Log("Lateral Fine HError Mag:" + distanceToTarget.ToString("F0"));

                            if ((throttleCount >= 4) || (distanceToTarget < TARGET_DISTANCE_THRESHOLD) || (herror == Vector3d.zero) )
                            {
                                LateralFineCount = throttleCount;
                                throttleCount = 0;
                                step = Step.STATE_CORRECT;
                            }
                            rc = 100; // we only want horizontal thrust here
                        }
                        break;
                }

                return rc;
            }

            Vector3d calculateHerror(ref Vector3d closestPos, ref Vector3d targetPos, ref double distanceToTarget, ref Vector3d surfaceVel)
            {
                Vector3d positionF = closestPos.normalized;
                Vector3d targetF = (Core.Landing.RotateRelativePosition(targetPos, tUT - Planetarium.GetUniversalTime())).normalized;
                Vector3d targetPointerF = (targetF - positionF).normalized;
                Vector3d projectedNormal = (Vector3d.Project(targetPointerF, VesselState.normalPlusSurface)).normalized;

                MechJebModuleDebugArrows.debugVector = targetPointerF;
                MechJebModuleDebugArrows.debugVector2 = projectedNormal;

                return projectedNormal;
            }

            public double setThrustAndVerticalAngle(out double verror, ref double desiredVerticalSpeed, ref Vector3d herror, ref Vector3d desiredHorizontalVel)
            {
                verror = desiredVerticalSpeed - VesselState.speedVertical;

                // Horizontal Velocity Control is only activated after a integration delay.
                if (forceHoriz == true)
                {
                    double thrust; // set by pidVelocity.
                    double angle = Vector3d.Angle(VesselState.forward, herror);
                    thrust = pidVelocity(0.0, ((angle <= 5) || (angle >= 175)) ? -0.5 * herror.magnitude : 0); // Correct horizontal velocity.
                    if (step == Step.STATE_LATERAL_FINE)
                    {
                        thrust *= 0.05 + 0.95 * Mathf.Clamp01((float)distanceToTarget / 15000.0f);
                    }
                    else if (step == Step.STATE_LATERAL)
                    {
                        thrust *= 0.05 + 0.95 * Mathf.Clamp01((float)herror.magnitude / ((float)baseGain*15000.0f));
                    }
                    else
                    {
                    }
                    vCorrectionAngle = 0;
                    lastHorizontalThrust = (float)thrust;
                    double throttleDiv = Core.Landing.debug11/Core.Landing.g;
                    Core.Thrust.RequestActiveThrottle(lastHorizontalThrust/(float)throttleDiv);
                }

                // Vertical Control - we want to blend it based on the horizontal error and vertical error. When herror is large we use max thrust and primarily control
                // vertical thrust via the vCorrectionAngle quickly. As the herror reduces the vCorrectionAngle rate change slows down as the thrust goes down
                // but once the vCorrectionAngle is large relative to the hCorrectionAngle then the thrust can be increased to correct vertical velocity more quickly. This allows for more aggressive horizontal control when far from target,
                // but as we get closer and the herror reduces then we can use more thrust to correct vertical velocity and reduce the time to target.
                else
                {
                    double horizontalThrust = Mathf.Clamp01((float)herror.magnitude/10000.0f); 
                    double verticalThrust;   // set by pidVelocity.
                    double vanglediv = (6 + 6*(1.0-horizontalThrust)); // 3500/1.5 <= 3500/0.5 <= 4000/0.5 (slow on earth)

                    // Calculate vertical correction angle - this is used to reduce the track the desired vertical speed when horizontal error is large to avoid overshooting the target horizontally.
                    // When horizontal error is small then the vertical correction angle can be more responsive to correct vertical velocity and reduce time to target.
                    vCorrectAngleAccum = Math.Max(minVerticalAngle, Math.Min(1.5, vCorrectAngleAccum + ((verror>0)?0.002:0.0015) * verror / vanglediv));
                    vCorrectionAngle = Math.Max(minVerticalAngle, Math.Min(1.5, 1 * verror / vanglediv + vCorrectAngleAccum));
                    Debug.Log("vCorrectionAngle:" + vCorrectionAngle.ToString("F4") + "," + vCorrectAngleAccum.ToString("F4"));

                    // Use the thrust calculated from the vertical speed when the horizontal thrust is less than 50%
                    if (horizontalThrust < 0.5)
                    {
                        // Use local PI control - more damped and works over wide range of parameters
                        if (vCorrectionAngle >= 0)
                        {
                            // Only applies thrust if need to increase vertical speed
                            verticalThrust = pidVelocity(desiredVerticalSpeed, VesselState.speedVertical, true);
                        }
                        else
                        {
                            // Only applies thrust if need to decrease vertical speed
                            verticalThrust = pidVelocity(-desiredVerticalSpeed, -VesselState.speedVertical);
                        }
                    }
                    else
                    {
                        verticalThrust = horizontalThrust;
                    }
                    lastHorizontalThrust = (float)verticalThrust;
                    double throttleDiv = Core.Landing.debug11/Core.Landing.g;
                    Core.Thrust.RequestActiveThrottle(lastHorizontalThrust/(float)throttleDiv); // set vertical thrust
                }

                return vCorrectionAngle;
            }

            public double pidVelocity(double desiredSpeed, double currentSpeed, bool down = false)
            {
                const double PID_ACCUM_MAX = 60;// 50; // 50<=30 <= 20
                // Increase PI Gains in higher gravity worlds - to track desired vertical speed
                double PID_KP = 0.5 * Math.Max(20, Math.Min(50, 20 + (50 - 20) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 50(highg);// 40; // 40<=30 <= 20
                double PID_KI = Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 0.05(lowg) 0.175(highg);// 0.35;// 0.25;// 0.15;// 0.2;// 0.1;
                double verror = (desiredSpeed - currentSpeed);
                double _limitedMaxThrustAccel = actualLimitedMaxThrustAccel;

                if (_limitedMaxThrustAccel < VesselState.limitedMaxThrustAccel)
                {
                    _limitedMaxThrustAccel = actualLimitedMaxThrustAccel + (VesselState.limitedMaxThrustAccel - actualLimitedMaxThrustAccel)
                        * (Core.Landing.g - 0.05) / (23 - 0.05);
                }
                
                verror /= _limitedMaxThrustAccel;

                velaccum += verror;
                double gComp = (down == true) ? VesselState.localg: 0;
                velaccum = Math.Max(0, Math.Min(PID_ACCUM_MAX, velaccum));
                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;
                return (Math.Max(0, Math.Min(_limitedMaxThrustAccel, (gComp + PID_KP * (PID_KI * velaccum + verror))) / VesselState.maxThrustAccel));
            }
        }
    }
}
