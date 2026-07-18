using System;
using KSP.Localization;
using TMPro;
using UnityEngine;
using static alglib;
using static iT;

namespace MuMech
{
    namespace Landing
    {
        // Vector PID with gravity/TWR scaling
        public class VectorPID
        {
            private Vector3d Kp;
            private Vector3d Ki;
            private Vector3d MinI;
            private Vector3d MaxI;
            private Vector3d Kd;
            private double FilterAlpha;
            private Vector3d integral = Vector3d.zero;
            private Vector3d prevError = Vector3d.zero;
            private Vector3d filteredDeriv = Vector3d.zero;
            private Vessel vessel;
            private bool use_g;

            public VectorPID(ref Vessel _vessel, ref Vector3d kp, ref Vector3d ki, ref Vector3d kd, ref double filterAlpha, double maxI, bool use_g = false)
            {
                vessel = _vessel;
                Kp = kp;
                Ki = ki;
                Kd = kd;
                FilterAlpha = filterAlpha;
                MinI = new Vector3d(-maxI, -maxI, -maxI);
                MaxI = new Vector3d( maxI,  maxI,  maxI);
                this.use_g = use_g;
            }

            public Vector3d Update(Vector3d error, double dt, double g, double maxAccel)
            {
                double scale = g / maxAccel;   // higher accel capability → lower gains

                Vector3d scaledKp = Kp * scale;
                Vector3d scaledKi = Ki * scale;
                Vector3d scaledKd = Kd * scale;

                integral = Vector3d.Max(MinI, Vector3d.Min(MaxI, integral + error * dt));

                Vector3d deriv = (error - prevError) / dt;
                filteredDeriv = FilterAlpha * filteredDeriv + (1 - FilterAlpha) * deriv;

                Vector3d p = Vector3d.Scale(scaledKp, error);
                Vector3d i = Vector3d.Scale(scaledKi, integral);
                Vector3d d = Vector3d.Scale(scaledKd, filteredDeriv);

                Vector3d output = p + i + d + ((use_g) ? vessel.up * g : Vector3d.zero);

                prevError = error;
                return output;
            }

            public void Reset() { integral = prevError = filteredDeriv = Vector3.zero; }
        }

        public class MoveToTargetV : AutopilotStep
        {
            enum MoveSequence
            {
                Disabled,           // Disabled - do nothing - When Land at Target is selected, this will transition to BurnToApoA
                BurnToApoA,         // Perform Burn to Acend To Suborbital Apoapsis - After this step, either go to CoastToApoA or CoastToLandingBurn
                CoastToApoA,        // Wait for Apoapsis - Apoapsis set, coast to Apoapsis or Warp to Apoapsis - At the end of this step, either go to TargetSubObit or CoastToLandingBurn
                CoastToLandingBurn, // Coast to Landing Burn - Wait for Landing Burn, or Warp to Landing Burn - After this step, either go to LandingBurn or FinalLanding
                LandingBurn,        // Perform Landing Burn - Burn to Landing Burn - After this step, either go to FinalLanding
                FinalLanding        // Move to Final Landing and Land - this will transition to Disabled when the landing is complete
            }

            private Vessel vessel;
            private Vector3d PosKp = new Vector3d(0.6, 0.6, 0.6);
            private Vector3d PosKi = new Vector3d(0.08, 0.08, 0.08);
            private Vector3d PosKd = new Vector3d(1.2, 1.2, 1.2);
            private double PosFilterAlpha = 0.82;
            private VectorPID positionPID; // terminal

            private Vector3d VelKp = new Vector3d(0.6, 0.6, 0.6);
            private Vector3d VelKi = new Vector3d(0.08, 0.08, 0.08);
            private Vector3d VelKd = new Vector3d(1.2, 1.2, 1.2);
            private double VelFilterAlpha = 0.82;
            private VectorPID velocityPID;

            private MoveSequence sequence = MoveSequence.Disabled; // Initialize to disabled, will determine next step based on conditions

            public MoveToTargetV(MechJebCore core) : base(core)
            {
                vessel = Core.part.vessel;
                positionPID = new VectorPID(ref vessel, ref PosKp, ref PosKi, ref PosKd, ref PosFilterAlpha, 1.0);
                velocityPID = new VectorPID(ref vessel, ref VelKp, ref VelKi, ref VelKd, ref VelFilterAlpha, 1.0, true);
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // ... your setup

                // your phase / suborbital logic here
                // In Drive / UpdateControl
                Vector3d posError = Core.Target.GetPositionTargetPosition() - Vessel.CoM;
                double altitude = Vector3d.Dot(posError, Vessel.up);
                Vector3d vel = Vessel.obt_velocity; // or surface
                Vector3d velError = CalculateVelocityError(posError, vel, Vessel.up, altitude); // your ratio logic

                // 1. Get ZEM/ZEV feedforward (your existing method)
                Vector3d zemZevAccel = ComputeZEMZEV(posError, vel, landingVelocity, t_go); // returns desired accel

                // 2. PID correction on velocity error
                Vector3d pidCorrection = velocityPID.Update(velError, Time.fixedDeltaTime, VesselState.localg, VesselState.limitedMaxThrustAccel);

                // 3. Combined desired acceleration
                Vector3d desiredAccel = zemZevAccel + pidCorrection;

                // 4. Terminal positionPID blend (close range)
                if (altitude < 50.0)
                {
                    Vector3d posCorr = positionPID.Update(posError, Time.fixedDeltaTime,  VesselState.localg, VesselState.limitedMaxThrustAccel) * 0.5;
                    desiredAccel += posCorr; // or replace velocity target
                }
                else
                {
                    // rely on ZEM/ZEV and velocity PID, mostly ZEM/ZEV feedforward, less velocity PID correction
                }

                // Limit, gravity already in ZEM/ZEV or add here
                ExecuteThrust(desiredAccel);

                return this;
            }

            private Vector3d CalculateVelocityError(Vector3d posError, Vector3d currentVel, Vector3d up, double altitude)
            {
                double hTargetError = posError.magnitude; // or your targetingResult.hTargetError
                float ratio = (float)(hTargetError / altitude);

                // Vertical desired speed (your steepness logic)
                float fraction = (ratio >= maxRatio) ? 0f : Mathf.Max(0f, Mathf.Min(1f, (maxRatio - ratio) / (maxRatio - minRatio)));
                double desiredVerticalSpeed = (fraction < 0.0001f) ? 0 : -Math.Abs(desiredVerticalSpeed) * ((Mathf.Pow(steepness, fraction) - 1f) / (steepness - 1f));

                // Horizontal (your divisor + max speed logic)
                double divisor = (hTargetError > hMidLimit) ? divFar : (hTargetError > hCloseLimit) ? divMid : divClose;
                double maxH = Math.Min(hTargetError * 2 * maxHThrust / divisor, Core.Landing.GetCircularOrbitSpeed(altitude, MainBody));
                Vector3d desiredHorizontalVel = maxH * courseCorrection.normalized;

                Vector3d desiredVel = (desiredVerticalSpeed * up) + desiredHorizontalVel;

                // Lateral cancel (your gainCancelHVelocity)
                Vector3d lateral = Core.Landing.debug12 * gainCancelHVelocity * Vector3d.Exclude(desiredHorizontalVel, currentVel);
                desiredVel -= lateral;

                // Terminal positionPID blend
                if (altitude < 50.0)
                {
                    Vector3d posCorr = positionPID.Update(posError, dt, g, maxAccel) * 0.5;
                    desiredVel += posCorr;
                }

                return desiredVel - currentVel; // velocity error
            }

            //
            // Determine Direction and Magnitude of Thrust to Apply
            //
            private void ExecuteThrust(Vector3d desiredAccel)
            {
                // your thrust / attitude logic adapted to vector
                Vector3d dir = desiredAccel.normalized;
                Quaternion targetRot = QuaternionD.LookRotation(dir, -Vessel.GetTransform().forward);
                Core.Attitude.attitudeTo(targetRot, AttitudeReference.INERTIAL, this);

                // Set Debug Vectors as horizontal and vertical components of desired thrust vector
                MechJebModuleDebugArrows.debugVector = Vector3d.Dot(desiredAccel, Vessel.upAxis) * Vessel.upAxis;
                MechJebModuleDebugArrows.debugVector2 = desiredAccel - MechJebModuleDebugArrows.debugVector;

                if (MechJebModuleDebugArrows.debugVector.magnitude < 0.01)
                {
                    MechJebModuleDebugArrows.debugVector *= 1000;
                }
                if (MechJebModuleDebugArrows.debugVector2.magnitude < 0.01)
                {
                    MechJebModuleDebugArrows.debugVector2 *= 1000;
                }

                // If angle between current and desired thrust vector is too large then set thrust to zero. Thrust will only
                // occur when in the ball park.
                bool disableThrust = false;

                // Angle is small enough, allow thrust for correct sequence
                if (Core.Attitude.attitudeAngleFromTarget() < throttleAngle)
                {
                    // Not actively performing collision avoidance, check the sequence
                    if ((vCollisionCounter == 0))
                    {
                        switch (sequence)
                        {
                            // Either disabled or coasting, disable thrust for these sequences.
                            case MoveSequence.Disabled:
                            case MoveSequence.CoastToApoA:
                            case MoveSequence.CoastToLandingBurn:
                                disableThrust = true;
                                break;

                            // Enable thrust for these sequences.
                            default:
                            case MoveSequence.BurnToApoA:
                            case MoveSequence.LandingBurn:
                            case MoveSequence.FinalLanding:
                                break;
                        }
                    }

                    // Angle is too large, disable thrust
                    else
                    {
                        disableThrust = true;
                    }

                    // Check if we are in the right attitude to thrust
                    double attitudeAngleFromThrust = Vector3d.Angle(Vessel.transform.up, dir);
                    if (disableThrust == true)
                    {
                        Core.Thrust.ThrustOff();
                        Debug.Log("thrust  OFF");
                    }
                    else
                    {
                        Vector3d thrust = dir * Mathf.Clamp01((float)(desiredAccel.magnitude / VesselState.limitedMaxThrustAccel));
                        Core.Thrust.RequestActiveThrottle((float)(thrust.magnitude));
                        Vector3d vertical = Vector3d.Dot(thrust, Vessel.up) * Vessel.up; // up component (radial)
                        Vector3d horizontal = thrust - vertical;                         // horizontal component (tangent plane)
                        Debug.Log("thrust  v:" + vertical.magnitude.ToString("F3") + "  h:" + horizontal.magnitude.ToString("F3"));
                    }
                }
            }
        }
    }
}
