using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;
using static alglib;
using static SoftMasking.SoftMask;


namespace MuMech
{
    namespace Landing
    {
        public class DecelerationBurn : AutopilotStep
        {
            enum Ascend2Target
            {
                Disabled,
                InitialBurn,
                FineTune,
                FineTune2,
                SetTarget,
                Completed
            }

            const double SPEED_CORRECTION_TIME_CONSTANT = 5.0;//2.0;//0.5;//0.3(thrust wobble);//0.4;//0.5; //0.3; // orig 0.3
            const double MAX_CORRECTION_ANGLE = 3;//1.5(works);//2;//1;  // orig 0.1
            const double CORRECT_ONLY_ANGLE_LIMIT = 5;//6(wobble);// 10(wobbly);// 20(wobbly); 
            const double WARP_END_TIME = 5;  // 5<=3<=orig 5
            const double WARP_START_ANGLE = 0.5;//5;  // orig 5 degrees
            const double CORRECT_ANGLE_FACTOR = 10;//4(overcorrect);// 2(overcorrect);//10(works pretty good);//8;//10;//2.0; // orig 2
            const double CORRECT_ONLY_ANGLE_FACTOR = 5.0;//4.0(wobble);//5.0;//4.0;//5.0;//2.0;//0.4;//0.2; // 0.1;
            const float CORRECT_ONLY_FACTOR = 0.035f; //0.02f(too little); // 0.05f(still too much); // 0.1f(too much);
            const float CORRECT_ONLY_TFACTOR = 0.1f; // Retro Burn - used to maintain minimal thrust
            private const float MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT = 4000;// 4000(eve-die);//3000(landing hot earth);//4000; //7000.0F;
            private const float SAFE_MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT = 8000;// 4000(eve-die);//3000(landing hot earth);//4000; //7000.0F;
            private const float ALT_LOWER_ACCEL_CONSTANT = 7000;//6000; // Lower TWR Max below this altitude to gain more control.
            private const float SAFE_ALT_MOVE_TO_TARGET_THRESHOLD_CONSTANT = 9000;//12000;//12000;//9000;//10000;//9000;
            private const float SAFE_ALT_SPEED_SLOW_CONSTANT = 23000;//13000;//12000;//15000(works-short);
            private const float ALT_MOVE_TO_TARGET_THRESHOLD_CONSTANT = 9000;//12000;//12000;//9000;//10000;//9000;
            private const float ALT_SPEED_SLOW_CONSTANT = 18000;//17000;//17000;//13000;//12000;//15000(works-short);
            private const float ALT_MIN_WARP_CONSTANT = 19000; // 19000 <= 50000 Do not warp below this altitude
            private const float H_MOVE_TO_TARGET_DISTANCE_THRESHOLD_CONSTANT = 245000; // only move to target below this horizontal limit
            private const float ANGLE_OVERSHOOT_P_CONSTANT = 40.0f; // 40(mun needs this)<=20.00000f; 
            private const float LANDING_GEAR_ALT_CONSTANT = 200.0F; // Drop landing gear below this altitude
            private const float FINAL_SPEED_FACTOR_CONSTANT = 0.95F; // To avoid crashing in terrain in final descent
            private const float SAFE_FINAL_SPEED_FACTOR_CONSTANT = 0.8F; // To avoid crashing in terrain in final descent
            private const double OVERSHOOT_ANGLE_FRACTION = 0.9; // Small margin to avoid losing vertical control
            private const double LIMITED_MAX_THRUST_G_RATIO = 3.125; // this is multiplied with Mainbody g
            private const double LIMITED_SLOW_THRUST_G_RATIO = 1.25; // this is multiplied with Mainbody g
            private const double TWR_REFERENCE = 1.5; // used by DecelerationBurn() to calculate thrust
            private const double LOW_GRAVITY_THRUST_MAX = 10;// 10 <= (bad)100.0; // 10.0
            private const int LAND_STABILIZE_COUNT = 500; // controls how long to stabilize landing.
            private const int IGNORE_LANDING_COUNT = 100; // controls how long to ignore landing check.
            private const int H_THRUST_INTEGRATE_COUNT = 20; // Integration count before allowing horizontal thrust control
            private const int IGNORE_ERR_INTEGRATE_COUNT = 40;  // Initial ignore count when ascending in a hop
            private const int DECREASE_ERR_INTEGRATE_COUNT = 6; // Integration count to determine when closest approach reached
            private const int INCREASE_ERR_INTEGRATE_COUNT = 6; // Integration count to determine when cannot improve approach

            private bool moveToTarget = false; // If overshoot detected or low grav world this is set to will burn toward target while maintaining altitude
            private bool stopIncrease = false; // Set to true when need to stop vertical increase
            private bool checkWarp = true; // Only enabled for retro burn to wait for right moment to burn
            private int suppressThrottle;
            private bool enableRetroCorrection;
            private bool emergency;
            private int _deployedGears = 1;  // 0->1->2  - deploy gears twice   2->1->0 store gears twice

            private double overshootUpAngle;
            private double overshootUpAngleSlow;
            private double velaccum = 0;
            private double limitedMaxThrustAccel = 0;
            private double actualLimitedMaxThrustAccel = 0;
            double TWR = 1;
            private IDescentSpeedPolicy _aggressivePolicy;
            private uint ThrottleCount = 0;
            private float altMoveToTargetThreshold;
            private float altSpeedSlow;
            private float altLandThreshold;
            private float speedFactor = 1.0F;
            private double vCorrectionAngle = 0.0;  // range is -1.0 to +1.0
            private double hAngleFactor = 1.0;      // range is 0.0 to +1.0
            private int landStabilizeCounter = 0;   // Used to Control how long to stabilize landing.
            private int ignoreLandingCounter = 0;   // Used to Control how long to ignore landing check
            private int hControlCounter;            // Allows horizontal pid after integration if vertical control is not operating.
            private int hThrustIntegrateCount = 200;// Initial count to ignore when starting up.
            private int skipCounter = 0;
            double currentError = 0;
            double prevError = 0;
            double startTiltAlt = 0;
            double tiltDelta = 0;
            double subOrbitalApA;
            double apaFactor;
            double hCloseLimit = 50;   // 50 <=(baseline)20
            double hMidLimit = 1500; // 1500<=1000(baseline)<=500<=100
            double hFarLimit = 10000;  // (baseline)10000
            double divClose;
            double divMid;
            double divFar;

            double baseGain; // Calibrated gain for horizontal angle and to correct movement that would diverge from target.

            // The steepness controls the ratio curvature - When horizontal distance to target is small we
            // dont want to modify the desired vertical velocity.
            // When horizontally far from target we want to maintain altitude, but slower allow vertical drop
            // as approach target.
            float steepness;// Defines the steepness to final descent ratio curvature to target.
            float maxRatio; // Desired Vertical speed is zero at this or larger ratios 
            float minRatio; // Desired Vertical speed is unmodified at this or smaller ratios

            // Try with course correction.
            private Ascend2Target errStep = Ascend2Target.Disabled;
            private int ignoreCount = IGNORE_ERR_INTEGRATE_COUNT;
            private int decreaseCount = DECREASE_ERR_INTEGRATE_COUNT;
            private int increaseCount = INCREASE_ERR_INTEGRATE_COUNT;

            private double throttleAngle;
            string trace;

            Vector3d perror = Vector3d.zero;


            public DecelerationBurn(MechJebCore core) : base(core)
            {
                double tempFactor = (Core.Landing.FlySafe == false) ? 20.0 : 10.0; // 10(baseline)<=100<=1000 Allow large angles - if angles are too small there will not be enough horizontal thrust
                double baseGainLo = 0.03136 / (Core.Landing.g) * Core.Landing.debug1; // 0.03136<=0.1568 <= 0.196
                double baseGainHi = 0.4704 / (Core.Landing.g) * Core.Landing.debug1; // 0.1568 <= 0.196
                baseGain = Math.Max(baseGainLo, Math.Min(baseGainHi, baseGainLo + (baseGainHi - baseGainLo) * (Core.Landing.g - 0.05) / (2 - 0.05)));

                // The steepness controls the ratio curvature - When horizontal distance to target is small we
                // dont want to modify the desired vertical velocity.
                // When horizontally far from target we want to maintain altitude, but slower allow vertical drop
                // as approach target.
                steepness = Core.Landing.BASE_STEEPNESS * (float)Core.Landing.steepness;
                // Ratio ranges
                maxRatio = (float)Core.Landing.maxRatio; // Desired Vertical speed is zero at this or larger ratios 
                minRatio = (float)Core.Landing.minRatio; // Desired Vertical speed is unmodified at this or smaller ratios
                double hTargetError = Core.Landing.getHDistanceToTarget();

                _deployedGears = 1; // This will trigger both store and deploy gears
                errStep = Ascend2Target.Disabled; // disable far target fine tune
                ignoreCount = IGNORE_ERR_INTEGRATE_COUNT;
                decreaseCount = DECREASE_ERR_INTEGRATE_COUNT;
                increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                startTiltAlt = 0; // Initialize to zero so it gets set when tilting towards target within the atmosphere
                throttleAngle = 30.0; // Allow throttling if within 30 degrees - in final landing angle will be wider.
                enableRetroCorrection = false;
                suppressThrottle = 0;
                landStabilizeCounter = 0;
                apaFactor = Math.Max(0.7, Math.Min(0.9, 0.7 + 0.2* ((actualLimitedMaxThrustAccel / Core.Landing.g) - 1.2) /(5 - 1.2)));

                if ( MainBody.atmosphere == true )
                {
                    subOrbitalApA = 1.6 * MainBody.atmosphereDepth;
                }
                else
                {
                    subOrbitalApA = Math.Max(24000.0,0.2 * MainBody.Radius);
                }

                if ((Core.Landing.UseOnlyMoveToTarget) || (hTargetError <= H_MOVE_TO_TARGET_DISTANCE_THRESHOLD_CONSTANT))
                {
                    checkWarp = false; // bypassing retoburn to move to target
                    moveToTarget = true; // force move to target
                    Core.Landing.UseOnlyMoveToTarget = false;
                }
                else
                {
                    checkWarp = true; // enable check warp by default
                    moveToTarget = false; // start with retroburn
                }
                limitedMaxThrustAccel = Math.Min(VesselState.limitedMaxThrustAccel, tempFactor * LIMITED_MAX_THRUST_G_RATIO * Core.Landing.g);
                overshootUpAngle = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(limitedMaxThrustAccel, 2) - Math.Pow(Core.Landing.g, 2)) / Core.Landing.g;
                overshootUpAngleSlow = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(Math.Min(limitedMaxThrustAccel, tempFactor * LIMITED_SLOW_THRUST_G_RATIO * Core.Landing.g), 2) - Math.Pow(Core.Landing.g, 2)) / Core.Landing.g;
                TWR = (limitedMaxThrustAccel / Core.Landing.g) / TWR_REFERENCE;

                hCloseLimit = 50;   // 50 <=(baseline)20
                hMidLimit = 1500; // 1500<=1000(baseline)<=500<=100
                hFarLimit = 10000;  // (baseline)10000

                if (Core.Landing.FlySafe)
                {
                    altMoveToTargetThreshold = SAFE_ALT_MOVE_TO_TARGET_THRESHOLD_CONSTANT;
                    altSpeedSlow = SAFE_ALT_SPEED_SLOW_CONSTANT;
                    speedFactor = SAFE_FINAL_SPEED_FACTOR_CONSTANT;
                    altLandThreshold = SAFE_MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT;
                    divClose = 66.55;
                    divMid = 33.28;
                    divFar = 16.63;
                }
                else
                {
                    altMoveToTargetThreshold = ALT_MOVE_TO_TARGET_THRESHOLD_CONSTANT;
                    altSpeedSlow = ALT_SPEED_SLOW_CONSTANT;
                    speedFactor = ((MainBody.atmosphere == false) && (Core.Landing.g > Core.Landing.LOW_GRAVITY)) ? (FINAL_SPEED_FACTOR_CONSTANT - 0.05f) : FINAL_SPEED_FACTOR_CONSTANT;
                    altLandThreshold = MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT;
                    divClose = 30;
                    divMid = 20;
                    divFar = (MainBody.atmosphere == false) ? 1.10 : 1;  // noatmos:1.25  atmos:1 <= 2
                }
            }
            private double GetMaxSpeed(bool updatePolicy, double dt = 0)
            {
                double maxSpeed;

                // Set Acceleration Limit - At approach to target limit it to 
                // a lower level to gain more control
                actualLimitedMaxThrustAccel = VesselState.limitedMaxThrustAccel;

                if ((Core.Landing.FlySafe == false) && (VesselState.altitudeTrue < ALT_LOWER_ACCEL_CONSTANT))
                {
                    actualLimitedMaxThrustAccel = limitedMaxThrustAccel;
                }

                if (VesselState.altitudeTrue < altLandThreshold)
                {
                    float alt = (float)VesselState.altitudeTrue;
                    if (Core.Landing.g > Core.Landing.EARTH_GRAVITY * 0.9) alt -= 10.0f;  // This gives more time at landing speed.
                    maxSpeed = -Mathf.Clamp01(alt / altLandThreshold) *
                        (float)Math.Sqrt((actualLimitedMaxThrustAccel - Core.Landing.g) * 2 * altLandThreshold) * speedFactor;
                }
                else
                {
                    if ((Core.Landing.FlySafe == false) || (VesselState.altitudeASL > 1.1 * MainBody.RealMaxAtmosphereAltitude()))
                    {
                        double alt = VesselState.altitudeTrue;
                        if (dt > 0.000001)
                        {
                            alt += VesselState.speedVertical * dt;
                        }

                        if (updatePolicy || _aggressivePolicy == null)
                        {
                            Vector3d estimatedLandingPosition = VesselState.CoM + VesselState.surfaceVelocity.sqrMagnitude / (2 * VesselState.limitedMaxThrustAccel) * VesselState.surfaceVelocity.normalized;
                            double terrainRadius = MainBody.Radius + MainBody.TerrainAltitude(estimatedLandingPosition);
                            _aggressivePolicy = new GravityTurnDescentSpeedPolicy(terrainRadius, MainBody.GeeASL * 9.81, VesselState.limitedMaxThrustAccel); // this constant policy creation is wastefull...
                        }
                        maxSpeed = _aggressivePolicy.MaxAllowedSpeed(VesselState.CoM + VesselState.orbitalVelocity * dt - MainBody.position,
                            VesselState.surfaceVelocity + dt * VesselState.gravityForce);
                        maxSpeed = speedFactor * Math.Max(maxSpeed, Math.Sqrt((VesselState.limitedMaxThrustAccel - VesselState.localg) * 2 * alt));
                    }
                    else
                    {
                        if (dt > 0.000001)
                        {
                            maxSpeed = Core.Landing.MaxAllowedSpeedAfterDt(dt);
                        }
                        else
                        {
                            maxSpeed = Core.Landing.MaxAllowedSpeed();
                        }
                    }
                }

                if ((VesselState.altitudeASL < MainBody.RealMaxAtmosphereAltitude()))
                {
                    maxSpeed = Math.Min(Core.Landing.ATMOS_FAST_SPEED, Math.Abs(maxSpeed));
                };

                if ((Core.Landing.FlySafe == false) && ((VesselState.altitudeTrue < altSpeedSlow) || (Core.Landing.g < Core.Landing.LOW_GRAVITY)))
                {
                    maxSpeed *= limitedMaxThrustAccel / VesselState.limitedMaxThrustAccel;
                }

                return (float)Math.Min(-Core.Landing.TouchdownSpeed, -Math.Abs(maxSpeed));
            }

            public void pidVelocity(double desiredSpeed, double currentSpeed, bool down = false)
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
                double gComp = (down == true) ? VesselState.localg * Core.Landing.debug3 : 0;
                PID_KI *= Core.Landing.debug4;
                PID_KP *= Core.Landing.debug5;
                velaccum = Math.Max(0, Math.Min(PID_ACCUM_MAX * Core.Landing.debug6, velaccum));
                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;


                Vessel.ctrlState.mainThrottle = Core.Thrust.TargetThrottle = (float)
                    (Math.Max(0, Math.Min(_limitedMaxThrustAccel, (gComp + PID_KP * (PID_KI * velaccum + verror))) / VesselState.maxThrustAccel));
            }

            public void setThrustVector(ref Vector3d desiredThrustVector, Vector3d hError, double hCorrectionAngle, double desiredVerticalSpeed, double verror)
            {
                if (vCorrectionAngle != 0 && ((VesselState.altitudeTrue < 250) || (Core.Landing.increaseVertical)))
                {
                    // The following ensures that the vertical herror is not too large by adjusting the horizontal angle.
                    if (desiredVerticalSpeed < 10)
                    {
                        if (verror > (0.45 + Math.Min(10, VesselState.altitudeTrue / 100)))
                        {
                            hAngleFactor = Math.Max(0.15, hAngleFactor * 0.7);
                        }
                        else if (verror < (0.35 + Math.Min(10, VesselState.altitudeTrue / 100)))
                        {
                            hAngleFactor = Math.Min(1, Math.Max(0.15, hAngleFactor * 1.1));
                        }
                    }
                    else
                    {
                        if (VesselState.speedVertical <= 15)
                        {
                            hAngleFactor = Math.Max(0.15, hAngleFactor * 0.95);
                        }
                        else
                        {
                            hAngleFactor = Math.Min(1, Math.Max(0.15, hAngleFactor * 1.005));
                        }
                    }
                }
                else
                {
                    hAngleFactor = 1.0;
                }

                // NOTES:
                // At Gilly need hCorrectionAngle gain at 4.0, but at Pol we need 1.0
                // At Gilly gainCancelHVelocity works with 4.0, but at Pol appears to be overgained. TODO: Try 1.0 at Pol.
                // Pattern: Gilly=4  Pol=1    ratio=GillyGravity/Gravity where Gilly/Gilly = 4.0*1.0, Pol = 4.0*(0.049/0.373) = 0.525
                //  or gain = 0.196/gravity
                // Calculate desired thrust vector.
                desiredThrustVector = (vCorrectionAngle * VesselState.up + hAngleFactor * hCorrectionAngle * hError.normalized).normalized;  // herror was not normalized before

                // At low altitude we want to ensure that the vessel does not drop to the ground when attitude change is large - a problem with high gravity worlds
                if (vCorrectionAngle != 0 && (Core.Landing.increaseVertical == false) && (VesselState.altitudeTrue < 1000))
                {
                    throttleAngle = Math.Max(30, Core.Attitude.attitudeAngleFromTarget() + 0.1);
                }

                // DEBUG Vectors
                MechJebModuleDebugArrows.debugVector = vCorrectionAngle * VesselState.up;
                if (hError.magnitude < 5.0)
                {
                    MechJebModuleDebugArrows.debugVector2 = 1000 * hCorrectionAngle * hError.normalized;
                }
                else
                {
                    MechJebModuleDebugArrows.debugVector2 = hCorrectionAngle * hError.normalized;
                }
            }

            // MoveToTarget 
            /// <summary>
            ///  Handles Low Gravity - Since the gravity is almost a non factor, most of the vertical velocity control requires
            ///  pitching the craft towards the surface facing forward and back.
            ///  For zero vertical velocity the following is expected:
            ///      1. No horizontal velocity change - facing up
            ///      2. Forward acceleration - facing forward with downward pitch
            ///      3. Forward deceleration - facing backward with slightly upward pitch.
            ///   Since there will be wild swings in orientation well need to keep the thrust at zero until within
            ///   range of desired orientation.
            /// </summary>
            /// <param name="desiredVerticalSpeed"></param>
            /// <param name="desiredThrustVector"></param>
            public void MoveToTarget(ref double desiredVerticalSpeed, ref Vector3d desiredThrustVector)
            {
                // Get the Horizontal distance to target. This will be used to calculate the horizontal speed.
                double hTargetError = Core.Landing.getHDistanceToTarget();

                // Get Horizontal direction vector towards the target. This will be combined later with
                // the vertical vector to get the thrust direction.
                Vector3d courseCorrection = Core.Landing.getHDirectionToTarget();

                // Using the current horizontal velocity and desired horizontal velocity calculate the desired
                // horizontal thrust vector.  An velocity herror vector will be calculated.
                // A speed up would be an herror vector in the direction of target
                // A speed down would be an herror vector opposite in the direction of target.
                float ratio = (float)(hTargetError / VesselState.altitudeTrue);
                float maxverror = 0.1f; // hysterisis in the negative direction.
                float minverror = -3f;  // hysterisis in the positive direction.

                // Set the desired vertical speed - it can be overriden to achieve suborbital flight
                setVerticalSpeed(ref desiredVerticalSpeed, ref hTargetError, ref ratio, ref minverror, ref maxverror);

                // Set the desired Horizontal Velocity to reach target. Error to subtract the vessels current horizontal velocity.
                double hCorrectionAngle = setHorizontalSpeed(out Vector3d desiredHorizontalVel, out Vector3d herror, ref hTargetError, ref ratio, ref courseCorrection);

                // Set the thrust based on primarily vertical herror, with horizontal as an alternate for thrust calculation when vertical is not needed.
                vCorrectionAngle = setThrustAndVerticalAngle(out double verror, ref desiredVerticalSpeed, ref herror, ref desiredHorizontalVel, ref hTargetError, ref minverror, ref maxverror);

                // Set horizontal vector based on positive vertical vector: can be suborbital or local vertical increase
                if (Core.Landing.increaseVertical == true)
                {
                    setSubOrbitalHorizontal(ref herror, ref hCorrectionAngle);
                }

                // Set thrust vector based on vertical and horizontal inputs
                setThrustVector(ref desiredThrustVector, herror, hCorrectionAngle, desiredVerticalSpeed, verror);

                // This is a long distance target requireing sub orbital targeting
                if ((currentError > 0) && (errStep > Ascend2Target.Disabled) && (errStep < Ascend2Target.Completed))
                {
                    TargetSubOrbital(ref desiredThrustVector);
                }

                // STOP Increase Vertical
                if (stopIncrease == true)
                {
                    stopIncrease = false;
                    Core.Landing.increaseVertical = false; // max ratio is less than max ratio  - no need to increase altitude.

                    // If in range of suborbital target then do retro burn.
                    if ((Core.Landing.g > Core.Landing.LOW_GRAVITY) && (((currentError > 0) && (currentError < (Core.Landing.POST_TARGET_THRESHOLD+1000)))
                        || (VesselState.orbitPeA >= 9000)) )
                    {
                        if (moveToTarget == true)
                        {
                            trace += 'r';
                            checkWarp = true;     // Exiting ascent phase at high altitude - warp to retro burn
                            moveToTarget = false; // go to retroburn
                        }
                    }

                    // Otherwise use move to target to get there.
                    else
                    {
                        checkWarp = false;    // Exiting ascent phase at low altitude - just move to target
                        moveToTarget = true;  // Stay in move to target
                        trace += 'm';
                    }
                }

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status17", // Moving to target: speed(V:<<1>> , H:<<2>>) m/s
                    desiredVerticalSpeed.ToString("F1"), trace);// desiredHorizontalVel.magnitude.ToString("F1"));
            }

            public void RetroBurn(ref double desiredSpeed, ref Vector3d courseCorrection, ref Vector3d desiredThrustVector)
            {
                // Get the Horizontal distance to target. This will be used to calculate the horizontal
                // speed.
                float ratio = (float)(Core.Landing.getHDistanceToTarget() / VesselState.altitudeTrue);
                double controlledSpeed = VesselState.speedSurface * Math.Sign(Vector3d.Dot(VesselState.surfaceVelocity, VesselState.up)); //positive if we are ascending, negative if descending
                double desiredSpeedAfterDt = GetMaxSpeed(true, VesselState.deltaT);
                double minAccel = -VesselState.localg * Math.Abs(Vector3d.Dot(VesselState.surfaceVelocity.normalized, VesselState.up));
                double maxAccel = VesselState.limitedMaxThrustAccel * Vector3d.Dot(VesselState.forward, -VesselState.surfaceVelocity.normalized) -
                                  VesselState.localg * Math.Abs(Vector3d.Dot(VesselState.surfaceVelocity.normalized, VesselState.up));

                double speedError;
                double desiredAccel;

                speedError = desiredSpeed - controlledSpeed;
                desiredAccel = speedError / SPEED_CORRECTION_TIME_CONSTANT + (desiredSpeedAfterDt - desiredSpeed) / VesselState.deltaT;

                double _currentError = Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite);
                if ((desiredAccel - minAccel) > 0)
                {
                    // Apply correction if angle is not too steep towards the ground
                    if (((MainBody.atmosphere == false || (VesselState.speedSurface < Core.Landing.ATMOS_FAST_SPEED)) && (Math.Abs(Vector3d.Angle(courseCorrection, VesselState.up)) < 100)))
                    {
                        double _correctionAngle = courseCorrection.magnitude / (CORRECT_ANGLE_FACTOR * TWR);
                        double correctionAngle = Math.Min(MAX_CORRECTION_ANGLE/* TWR*/, _correctionAngle);

                        desiredThrustVector = (desiredThrustVector + correctionAngle * courseCorrection.normalized).normalized;
                    }
                    Core.Thrust.TargetThrottle = Mathf.Clamp((float)((desiredAccel - minAccel) / (maxAccel - minAccel)), 0.0F, 1.0F);
                }
                else if (VesselState.altitudeASL > Core.Landing.DecelerationEndAltitude() + 5 && (enableRetroCorrection == true))
                {
                    // perform correction for high altitude
                    if (VesselState.altitudeTrue > 30000)
                    {
                        double _correctionAngle = courseCorrection.magnitude / (2 * TWR);
                        double correctionAngle = Math.Min(CORRECT_ONLY_ANGLE_LIMIT, _correctionAngle);
                        desiredThrustVector = courseCorrection.normalized;
                        Core.Thrust.TargetThrottle = Mathf.Min(1, (float)(correctionAngle * 0.2 * Core.Landing.g / 23));
                    }

                    // Approaching target thru atmosphere - perform small corrections since otherwise thrusters would be off.
                    else
                    {
                        double _correctionAngle = courseCorrection.magnitude / (CORRECT_ONLY_ANGLE_FACTOR * TWR);
                        double correctionAngle = Math.Min(CORRECT_ONLY_ANGLE_LIMIT, _correctionAngle);
                        desiredThrustVector = (-VesselState.surfaceVelocity.normalized + correctionAngle * courseCorrection.normalized).normalized;
                        Core.Thrust.TargetThrottle = Mathf.Min(CORRECT_ONLY_TFACTOR, (float)(correctionAngle * CORRECT_ONLY_FACTOR * Core.Landing.g / 23));
                    }

                    if (_currentError < 600)
                    {
                        enableRetroCorrection = false;
                    }
                }
                else
                {
                    Core.Thrust.TargetThrottle = 0;
                    if (_currentError > 3000)
                    {
                        //enableRetroCorrection = true;
                    }
                }

                if (Core.Thrust.TargetThrottle > 0.01)
                {
                    ThrottleCount = 1000;
                }
                else if (ThrottleCount > 0)
                {
                    ThrottleCount--;
                }

                if ((ThrottleCount > 0) && (Core.Thrust.TargetThrottle < 0.0001))
                {
                    Core.Thrust.TargetThrottle = 0.001f;
                }

                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status6",
                         Math.Abs(desiredSpeed) >= double.MaxValue ? "∞" : desiredSpeed.ToString("F1")); //"Braking: target speed = " +  + " m/s"

                // Entering landing area - switch to move to target.
                if (ratio < 0.5 * Core.Landing.maxRatio || VesselState.altitudeTrue < 1000)
                {
                    if (MainBody.atmosphere == false || (VesselState.speedSurface < Core.Landing.ATMOS_FAST_SPEED))
                    {
                        checkWarp = false;    // Exiting ascent phase at low altitude - just move to target
                        moveToTarget = true; // exit retroburn - go to move to target
                    }
                }

                // If in range of suborbital target then do retro burn.
                else if ((Core.Landing.g < Core.Landing.LOW_GRAVITY) || 
                    ((_currentError > (Core.Landing.POST_TARGET_THRESHOLD + 1000)) && (VesselState.orbitPeA < -1000)))
                {
                    checkWarp = false;    // Exiting ascent phase at low altitude - just move to target
                    moveToTarget = true; // exit retroburn - go to move to target
                    if (_currentError > (Core.Landing.POST_TARGET_THRESHOLD + 1000) ) Core.Landing.increaseVertical = true;
                }
            }

            public void setVerticalSpeed(ref double desiredVerticalSpeed, ref double hTargetError, ref float ratio, ref float minverror, ref float maxverror)
            {
                // Activate Emergency vertical speed increase if far from target and altitude is less than 100 meters and far away.
                // The reasoning is that the algorithm should push for a small ratio to begin with so something
                // went wrong.
                emergency = false;
                if ((ratio > maxRatio) && (VesselState.altitudeTrue < 400.0) && (hTargetError > 500.0))
                {
                    Core.Landing.increaseVertical = true; // Emergency - avoid terrain collision
                    emergency = true;
                }

                // Increase Vertical so that maxRatio is reached - this can be due to a hop or emergency crash avoidance
                if (Core.Landing.increaseVertical == true)
                {
                    if (ratio > maxRatio)
                    {
                        desiredVerticalSpeed = Math.Max(2.0, (ratio - maxRatio) * (1 + hTargetError / 2000.0));
                        if ((Math.Abs(VesselState.orbitApA) > subOrbitalApA) && (VesselState.altitudeTrue>2000) )
                        {
                            desiredVerticalSpeed = 0;
                        }
                        ignoreLandingCounter = 0; // reset ignore landing count to allow start from a landing
                    }
                    else stopIncrease = true; // Trigger STOP vertical increase
                }
                else
                {
                    // at low altitudes decrease the hysterisis.
                    if ((VesselState.altitudeTrue < altLandThreshold) || (Core.Landing.g <= Core.Landing.LOW_GRAVITY))
                    {
                        minverror = -maxverror;
                    }

                    // Smooth Function to calculate desired vertical speed.
                    float fraction = (ratio >= maxRatio) ? 0.0f : Mathf.Max(0.0f, Mathf.Min(1.0f, (maxRatio - ratio) / (maxRatio - minRatio)));
                    if (Double.IsNaN(desiredVerticalSpeed)) return;
                    desiredVerticalSpeed = (fraction < 0.0001) ? 0 : -(float)Math.Abs(desiredVerticalSpeed) * ((Mathf.Pow(steepness, fraction) - 1.0f) / (steepness - 1.0f));
                    if (Double.IsNaN(desiredVerticalSpeed)) return;
                }
            }

            public double setHorizontalSpeed(out Vector3d desiredHorizontalVel, out Vector3d herror, ref double hTargetError, ref float ratio, ref Vector3d courseCorrection)
            {

                // div = baseDiv*(20.408*g)                                      safe                                 notsafe
                // bop    = 0.589   bop/gilly    = 12.020   /2 = 6.010           100/200/400                          12.02/120.20/180.6
                // pol    = 0.373   pol/gilly    =  7.612   /2 = 3.806
                // gilly  = 0.049   gilly/gilly  = 1        /2 = 0.5             16.63/33.28/66.55                    2/20/30
                // phobos = 0.0057  phobos/gilly = 0.116    /2 = 0.116 
                // deimos = 0.003   deimos/gilly = 0.0612   /2 = 0.0306
                double divisor = (hTargetError > hMidLimit) ? divFar : (hTargetError > hCloseLimit) ? divMid : divClose;  // 100-200-400(pol)<=(baseline2-gilly)2-20-30<=(baseline1)2-50-90
                double _overshootUpAngle;
                double maxHThrust = Math.Min(LOW_GRAVITY_THRUST_MAX * Core.Landing.g, limitedMaxThrustAccel);
                double gainCancelHVelocity = (hTargetError <= hCloseLimit) ? baseGain : baseGain / 13.0; // gilly=4(hTargetError < 500)

                // Scale divisor based on local gravity and cap the values.
                // 0.5 <= 2.0  ? Divisor is bigger for larger gravity - not sure this is right
                divisor = Math.Max(0.5, Math.Min(800, divisor * (0.4 * 20.408) * Core.Landing.g));

                // Controls Max angle to move horizontally.
                if (hTargetError < 1.25)
                {
                    _overshootUpAngle = 0.025 * 50.4 * Core.Landing.debug2; //  (baseline)0.025 * 36<=0.025 * 18; //  0.025 * 9; // 0.025*3;
                }
                else if (hTargetError < hCloseLimit)
                {
                    _overshootUpAngle = 0.025 * 25.2 * Core.Landing.debug2; //  (baseline)0.025 * 36<=0.025 * 18; //  0.025 * 9; // 0.025*3;
                }
                else if (hTargetError < hFarLimit)
                {
                    _overshootUpAngle = overshootUpAngleSlow;
                }
                else
                {
                    _overshootUpAngle = overshootUpAngle;
                }


                // If horizontal herror is really large set desired horizontal velocity based on reaching target
                // If target within reach then avoid changing the horizontal velocity.
                if ((Core.Landing.g > Core.Landing.LOW_GRAVITY) && (VesselState.altitudeTrue > 1000) && (hTargetError > H_MOVE_TO_TARGET_DISTANCE_THRESHOLD_CONSTANT) && (ratio > maxRatio))
                {
                    if (Core.Landing.PredictionReady)
                    {
                        double _currentError = Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite);
                        if (Double.IsNaN(_currentError) || Double.IsInfinity(_currentError) || _currentError < 4)
                        {
                            // don't set it.
                        }
                        else
                        {
                            currentError = _currentError;
                            if (errStep == Ascend2Target.Disabled)
                            {
                                errStep++; trace += ((int)errStep).ToString(); // Will execute fine tune to far target 
                            }
                        }
                    }

                    // Set horizontal velocity to reach predicted target
                    // Use max expected velocity and scale it based on  
                    desiredHorizontalVel = (currentError * Core.Landing.g / 70.0) * courseCorrection.normalized;
                    herror = desiredHorizontalVel - Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                    _overshootUpAngle = 0.005;
                }

                // Use default method to calculate horizontal velocity
                else
                {
                    prevError = currentError = 0; // Dont allow fine tune as not needed
                    if (hTargetError < 0.80)
                    {
                        desiredHorizontalVel = (Math.Sqrt((1 + Core.Landing.g) * hTargetError * maxHThrust / divisor) * 2.0 / 1.5) * courseCorrection.normalized;
                        herror = 0.25 * desiredHorizontalVel - Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                    }
                    else
                    {
                        desiredHorizontalVel = (Math.Sqrt((1 + Core.Landing.g) * hTargetError * maxHThrust / divisor) * 2.0 / 1.5) * courseCorrection.normalized;
                        herror = desiredHorizontalVel - Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                    }
                }

                // Zero out the horizontal velocity that is not part of the desired horizontal velocity - avoids a limit cycle.
                if (Core.Landing.g <= Core.Landing.LOW_GRAVITY)
                {
                    herror -= 0.5 * gainCancelHVelocity * Vector3d.Exclude(desiredHorizontalVel, Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity));
                }
                else
                {
                    herror -= gainCancelHVelocity * Vector3d.Exclude(desiredHorizontalVel, Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity));
                }

                // Calculate horizontal correction angle using horizontal herror.
                return Math.Max(0, Math.Min(_overshootUpAngle, (baseGain * ANGLE_OVERSHOOT_P_CONSTANT * herror.magnitude) / 100.0));  // 4<=1<=10<=100<=3   gilly:4
            }

            public void setSubOrbitalHorizontal(ref Vector3d herror, ref double hCorrectionAngle)
            {
                // If vertical increase is enabled then disable horizontal angle at small altitude.
                if (VesselState.altitudeTrue < 25.0 || emergency == true) // TODO - account for bottom of craft
                {
                    hCorrectionAngle = vCorrectionAngle * 0.1;
                }

                else if (((errStep == Ascend2Target.Disabled) || (currentError > 0)))
                {
                    if (MainBody.atmosphere == false)
                    {
                        //double apaFactor = ((actualLimitedMaxThrustAccel / Core.Landing.g) < 5) ? 1.42 : 1.1;
                        hCorrectionAngle = Math.Min(1.75, Math.Sqrt(Math.Pow(VesselState.limitedMaxThrustAccel,2) - Math.Pow(Core.Landing.g*1.1, 2)) ); // Always have some angle towards toward
                    }
                    else
                    {
                        hCorrectionAngle = 0.2; // Always have some angle towards toward
                    }

                    // Eve      35k alt   atmos density = 125 g/m^3   drag = 30 m/s^2   - FAIL
                    // kerbin   35k alt   atmos density = 2.2 g/m^3   drag = 1.3 m/s^2  - PASS
                    // kerbin 17.5k alt   atmos density =  66 g/m^3 - start of tilt.
                    // kerbin    0k alt   atmos density = 1.2 kg/m^3
                    // for atmospheric planets we taper starting at a specific altitude and increase tilt at higher altitudes
                    // Also qualify it with a minimum vertical velocity before we start tilting.
                    if (MainBody.atmosphere == true && VesselState.atmosphericDensityGrams < 66.0)
                    {
                        if (tiltDelta < 0.1) tiltDelta = MainBody.atmosphereDepth - VesselState.altitudeASL;
                        else if (VesselState.speedVertical > Core.Landing.g * 2 * 30)   // 30<=20(too shallow)<=30
                        {
                            if (startTiltAlt < 0.1) startTiltAlt = VesselState.altitudeASL;
                        }

                        if ((startTiltAlt >= 0.1) && (VesselState.altitudeASL > startTiltAlt))
                        {
                            hCorrectionAngle += 10 * Mathf.Clamp01((float)((VesselState.altitudeASL - startTiltAlt) / tiltDelta));
                            if (hCorrectionAngle >= 9.8)
                            {
                                vCorrectionAngle = -0.5; // This reduces the apoapsis
                            }
                        } // else - dont add any tilt
                    }

                    herror = herror.normalized; // normalizing allows control override
                } // else - no atmosphere - leave horizontal vector unmodified.
            }

            public void TargetSubOrbital(ref Vector3d desiredThrustVector)
            {
                double _currentError = currentError;

                if (skipCounter > 0 ) skipCounter--;
                else
                {
                    skipCounter = 5;
                    if ((ignoreCount > 0) || (suppressThrottle == 0)) ignoreCount--;
                    else if (prevError > 0)
                    {
                        if (decreaseCount > 0)
                        {
                            if (_currentError < prevError)
                            {
                                decreaseCount -= 2;
                                if (decreaseCount <= 0) trace += 'd';
                            }
                            else decreaseCount = Math.Min(DECREASE_ERR_INTEGRATE_COUNT, decreaseCount + 1);
                        }
                        else if (increaseCount <= 0)
                        {
                            // If the first fine tune was not successful then wait till reach new Apoapsis
                            // if not already reached it
                            if ((errStep == Ascend2Target.FineTune) && (_currentError > 550))
                            {
                                apaFactor = 0.9;
                                if ((VesselState.altitudeASL < (apaFactor * VesselState.orbitApA)) && (VesselState.speedVertical > 0))
                                {
                                    suppressThrottle = 1;
                                    trace += 'S';
                                }
                            }

                            // go to next step and reset all counters.
                            errStep++; trace += ((int)errStep).ToString();
                            ignoreCount = IGNORE_ERR_INTEGRATE_COUNT;
                            increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                        }
                        else if (_currentError > prevError)
                        {
                            increaseCount -= 2;
                            if (increaseCount <= 0) trace += 'u';
                        }
                        else increaseCount = Math.Min(INCREASE_ERR_INTEGRATE_COUNT, increaseCount + 1);

                    }
                    prevError = _currentError;
                }

                bool overrideAttitude = false;

                // Use correction attitude if apoapsis is high enough.
                // Determine when we can stop burning when target apoapsis has been reached
                // Once stopped a secondary burn will be performed to reach target
                if (suppressThrottle == 0)
                {
                    if (Math.Abs(VesselState.orbitApA) >= subOrbitalApA)
                    {
                        overrideAttitude = true; // with atmosphere
                        if ((VesselState.altitudeASL < (apaFactor * Math.Abs(VesselState.orbitApA))) && (VesselState.speedVertical > 0))
                        {
                            suppressThrottle = 1;
                            trace += 'S';
                        }
                    }
                }

                // While throttle is disabled monitor it until a burn can start
                if (suppressThrottle == 1)
                {
                    // Hold on until we get to point where we can burn to target.
                    if ((VesselState.altitudeASL < (apaFactor * Math.Abs(VesselState.orbitApA))) && (VesselState.speedVertical > 0) )
                    {
                        increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                    }
                    else
                    {
                        suppressThrottle = 2; // latch it off - throttle will no longer be suppressed.
                        trace += 'U';
                    }
                }

                // If Periapsis is positive then we are done.
                if (VesselState.orbitPeA >= 10000 )
                {
                    errStep = Ascend2Target.Completed;
                }
                // For initial burn the first burn will not override attitude.
                else if (errStep == Ascend2Target.InitialBurn)
                {
                    if (suppressThrottle >= 2)
                        overrideAttitude = true; // only after throttle suppression ends - enable fine tune - use correction attitude
                    if (_currentError <= 500)
                    {
                        errStep++;
                        trace += ((int)errStep).ToString();
                    };
                }

                // If target reached in first fine tune then set target
                // If target reached in second fine tune then skip set target and complete
                else if (errStep == Ascend2Target.FineTune || errStep == Ascend2Target.FineTune2)
                {
                    overrideAttitude = true; // enable fine tune - use correction attitude
                    if (_currentError <= 500)
                    {
                        errStep += 2;
                        trace += ((int)errStep).ToString();
                    };
                }
                // Set target will overshoot target so that retrograde burn will decrease closer to target.
                // Without this the final landing spot will move further away from target.
                else if (errStep == Ascend2Target.SetTarget)
                {
                    Core.Landing.increaseVertical = false; // We are done increasing altitude
                    overrideAttitude = true; // enable fine tune - use correction attitude
                    _currentError = Core.Landing.POST_TARGET_THRESHOLD + 500 - currentError;
                    ignoreCount = IGNORE_ERR_INTEGRATE_COUNT;   // stay in this state until currentError has reached target offset
                    increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                    if (currentError > Core.Landing.POST_TARGET_THRESHOLD)
                    {
                        errStep++;
                        trace += ((int)errStep).ToString();
                    };
                }

                // done - exit and go to warp to retroburn
                if (errStep >= Ascend2Target.Completed)
                {
                    errStep = Ascend2Target.Completed;          // Exit fine tuning - will not return
                    ignoreCount = IGNORE_ERR_INTEGRATE_COUNT;
                    decreaseCount = DECREASE_ERR_INTEGRATE_COUNT;
                    increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                    suppressThrottle = 2;     // disable throttle suppression
                    overrideAttitude = false; // Disable attitude override
                    checkWarp = true;     // Exit ascent phase and go to retro burn
                    moveToTarget = false; // Go to retroburn
                    _currentError = 0;
                    trace += 'R';
                }

                // Override throttle based on distance to target.
                Vessel.ctrlState.mainThrottle = Core.Thrust.TargetThrottle = Mathf.Clamp01((float)_currentError / 450000.0f);

                //If attitude override enabled then set it
                if (overrideAttitude == true)
                {
                    if (errStep == Ascend2Target.InitialBurn)
                    {
                        desiredThrustVector = desiredThrustVector.normalized + 
                            0.2*Core.Landing.ComputeCourseCorrection(true, 20.0 * Core.Landing.debug7).normalized;
                    }
                    if (errStep == Ascend2Target.SetTarget)
                    {
                        desiredThrustVector = VesselState.surfaceVelocity.normalized;
                    }
                    else
                    {
                        desiredThrustVector = 0.2*desiredThrustVector.normalized +
                            0.8 * Core.Landing.ComputeCourseCorrection(true, 20.0 * Core.Landing.debug7).normalized;
                    }
                }
            }

            public double setThrustAndVerticalAngle(out double verror, ref double desiredVerticalSpeed, ref Vector3d herror, ref Vector3d desiredHorizontalVel, ref double hTargetError, ref float minverror, ref float maxverror)
            {
                // The desired thrust vector can pitch up or down depending on desired vertical velocity since gravity is not much 
                // help here.
                verror = desiredVerticalSpeed - VesselState.speedVertical;

                if ((verror < minverror) || (verror > maxverror)) // hysterisis to minimize changing vertical direction
                {
                    double sign = Math.Sign(vCorrectionAngle);
                    vCorrectionAngle = Math.Max(-1.0, Math.Min(1.0, vCorrectionAngle + verror / 4000.0));
                    if (sign != Math.Sign(vCorrectionAngle))
                    {
                        vCorrectionAngle -= (Core.Landing.g <= Core.Landing.LOW_GRAVITY) ? 0.2 * sign : 0.1 * sign;
                    }
                    velaccum = 0;
                }

                if (Core.Landing.g <= Core.Landing.LOW_GRAVITY)
                {
                    if ((hTargetError < hCloseLimit) && (VesselState.altitudeTrue < 15))
                    {
                        vCorrectionAngle = 0.2; // Set vertical component for lander to be upright
                    }
                }

                // If within the atmosphere or close to the ground in high gravity try to point up to avoid collision issues 
                else
                {
                    if ((hTargetError < hCloseLimit))
                    {
                        vCorrectionAngle = (VesselState.altitudeTrue < 15) ? 0.1 : 0.2; // point up - making this small allows horizontal angle to have more effect
                    }
                    else if (vCorrectionAngle < 0.5)
                    {
                        if ((VesselState.altitudeASL < MainBody.RealMaxAtmosphereAltitude()) ||
                             ((MainBody.atmosphere == false) && (Core.Landing.g > Core.Landing.EARTH_GRAVITY * 0.5)) ||
                             (VesselState.altitudeTrue < 500))
                        {
                            vCorrectionAngle = 0.5; // point up - a larger vertical component filters out large horizontal change
                        }
                    }
                }

                // Horizontal Velocity Control is only activated after a integration delay.
                if (hControlCounter > hThrustIntegrateCount)
                {
                    hControlCounter--;
                    vCorrectionAngle = 0; // while horizontal is being corrected disable vertical angle

                    // Only exclude desired horizontal velocity when vertical velocity is being maintained in orbit - this has the effect of correcting 
                    // plane to target . Otherwise, allow total horizontal velocity reduction control.
                    if ((desiredVerticalSpeed >= 0.0) && (VesselState.orbitPeA > 0.9 * VesselState.orbitApA))
                    {
                        herror = -Vector3d.Exclude(desiredHorizontalVel, Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity));
                        pidVelocity(0.0, (herror.magnitude > 4) ? -herror.magnitude : 0); // Only correct if the herror is large.
                    }
                    else
                    {
                        double angle = Vector3d.Angle(VesselState.forward, desiredHorizontalVel);
                        pidVelocity(0.0, ((angle <= 10) || (angle >= 170)) ? -0.5 * herror.magnitude : 0); // Correct horizontal velocity.
                    }
                }
                else
                {
                    // Use local PI control - more damped and works over wide range of parameters
                    if (vCorrectionAngle >= 0)
                    {
                        // Only applies thrust if need to increase vertical speed
                        pidVelocity(desiredVerticalSpeed, VesselState.speedVertical, true);
                    }
                    else
                    {
                        // Only applies thrust if need to decrease vertical speed
                        pidVelocity(-desiredVerticalSpeed, -VesselState.speedVertical);
                    }

                    if ((Core.Thrust.TargetThrottle < 0.001) && (hTargetError > hCloseLimit))
                    {
                        hControlCounter++; // No vertical control needed - integrate to allow horizontal thrust.
                        if (hControlCounter > hThrustIntegrateCount)
                        {
                            hControlCounter += H_THRUST_INTEGRATE_COUNT;
                            hThrustIntegrateCount = H_THRUST_INTEGRATE_COUNT;
                            velaccum = 0;
                            vCorrectionAngle = 0; // while horizontal is being corrected disable vertical angle
                        }
                    }
                    else
                    {
                        hControlCounter = 0;
                        if (hThrustIntegrateCount > H_THRUST_INTEGRATE_COUNT) hThrustIntegrateCount--;
                    }
                }

                return vCorrectionAngle;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (checkWarp == true)
                {
                    return this;
                }

                // Vertical Speed in progress
                if (Core.Landing.increaseVertical == true || ignoreLandingCounter < IGNORE_LANDING_COUNT)
                {
                    // increment counter
                    ignoreLandingCounter++;
                }

                // Landing Completed - exit
                else if (Vessel.LandedOrSplashed || (landStabilizeCounter != 0))
                {
                    Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                    Core.Thrust.TransKillH = false;
                    Core.Thrust.TargetThrottle = 0;
                    Core.Thrust.TransSpdAct = 0;
                    if (landStabilizeCounter < LAND_STABILIZE_COUNT)
                    {
                        landStabilizeCounter++;
                        Core.Attitude.attitudeTo(VesselState.forward, AttitudeReference.INERTIAL, Core.Landing);
                        return this;
                    }
                    else
                    {
                        Core.Landing.StopLanding();
                        return null;
                    }
                }

                // Consider storing the landing gear when performing a long hop
                if (currentError >= 10 || Core.Landing.increaseVertical == true)
                {
                    if ((_deployedGears > 0) && (VesselState.altitudeTrue >= LANDING_GEAR_ALT_CONSTANT))
                    {
                        Vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
                        _deployedGears--;
                    }
                }
                // Consider lowering the landing gear
                else
                {
                    if ((_deployedGears < 2) && (VesselState.altitudeTrue < LANDING_GEAR_ALT_CONSTANT))
                    {
                        Vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
                        _deployedGears++;
                    }
                }

                // Set desired speed
                // As deorbit approaches end scale it down to lower speed based on
                // lower acceleration limit, but do this at a higher altitude
                // to take advantage of available thrust so it can reach desired speed quickly.
                double desiredSpeed = GetMaxSpeed(true);

                Vector3d desiredThrustVector = -VesselState.surfaceVelocity.normalized;

                // Move to target strategy
                if (moveToTarget == true)
                {
                    MoveToTarget(ref desiredSpeed, ref desiredThrustVector);
                }
                // Perform Retroburn
                else
                {
                    Vector3d courseCorrection = Core.Landing.ComputeCourseCorrection(true, 20.0 * Core.Landing.debug7);
                    RetroBurn(ref desiredSpeed, ref courseCorrection, ref desiredThrustVector);
                }

                Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, Core.Landing);

                // If angle between current and desired thrust vector is too large then set thrust to zero. Thrust will only
                // occur when in the ball park.
                if ((suppressThrottle == 1) || (checkWarp == true) || Core.Attitude.attitudeAngleFromTarget() > throttleAngle) // 30<=40(overcorrect)<=30<=15(jittery)<=30<=5<=15
                {
                    Vessel.ctrlState.mainThrottle = Core.Thrust.TargetThrottle = 0;
                    velaccum = 0;
                }

                if (double.IsNaN(desiredSpeed)) return new DecelerationBurn(Core);
                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                double hTargetError = Core.Landing.getHDistanceToTarget();
                float ratio = (float)(hTargetError / VesselState.altitudeTrue);

                if ( (ratio < 1.5*Core.Landing.maxRatio) || 
                    ((VesselState.speedVertical < 0) && ((VesselState.altitudeTrue < ALT_MIN_WARP_CONSTANT) ||
                    ((VesselState.altitudeASL < MainBody.RealMaxAtmosphereAltitude()) && (VesselState.speedSurface > Core.Landing.ATMOS_FAST_SPEED)))) )
                {
                    checkWarp = false; // Can't warp under these circumstances
                }

                if (checkWarp == true)
                {
                    double decelerationStartTime =
                            Core.Landing.Prediction.Trajectory.Any() ? Core.Landing.Prediction.Trajectory.First().UT : VesselState.time;
                    double warpEndTime = WARP_END_TIME;
                    double desiredSpeed = 1.2*GetMaxSpeed(true);

                    if (VesselState.speedVertical > 0)
                    {
                        warpEndTime += VesselState.speedVertical / VesselState.localg;
                        if (decelerationStartTime <= (VesselState.time + 1))
                        {
                            //decelerationStartTime += warpEndTime * 1.4;
                            decelerationStartTime = VesselState.orbitTimeToAp;
                        }
                    }
                    else if (VesselState.speedSurface > Math.Abs(desiredSpeed))
                    {
                        warpEndTime += ((VesselState.speedSurface - Math.Abs(desiredSpeed)) / Math.Abs(desiredSpeed)) * VesselState.localg;
                    }

                    if ((decelerationStartTime - VesselState.time) > warpEndTime)
                    {
                        Core.Thrust.TargetThrottle = 0;

                        if (Core.Node.Autowarp)
                        {
                            Status = Localizer.Format("#MechJeb_LandingGuidance_Status4"); //"Warping to start of braking burn."
                        }
                        else
                        {
                            Status = Localizer.Format("#MechJeb_LandingGuidance_Status1"); //"Coasting toward deceleration burn."
                        }

                        //warp to deceleration start
                        Vector3d decelerationStartAttitude = -Orbit.WorldOrbitalVelocityAtUT(decelerationStartTime);
                        decelerationStartAttitude += MainBody.getRFrmVel(Orbit.WorldPositionAtUT(decelerationStartTime));
                        decelerationStartAttitude = decelerationStartAttitude.normalized;
                        Core.Attitude.attitudeTo(decelerationStartAttitude, AttitudeReference.INERTIAL, Core.Landing);

                        bool warpReady = ((Vessel.angularVelocity.magnitude < 0.005f) &&
                                            (Core.Attitude.attitudeAngleFromTarget() < WARP_START_ANGLE));

                        if (warpReady && Core.Node.Autowarp)
                            Core.Warp.WarpToUT(decelerationStartTime - warpEndTime);
                        else if (!MuUtils.PhysicsRunning())
                            Core.Warp.MinimumWarp();
                    }
                    else
                    {
                        checkWarp = false; // Time to burn - no more warping needed.
                    }
                }

                return this;
            }
        }
    }
}
