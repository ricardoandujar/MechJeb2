using System;
using System.Linq;
using EdyCommonTools;
using KSP.Localization;
using MechJebLib.PVG.Integrators;
using UnityEngine;
using VehiclePhysics;
using static alglib;
using static EdyCommonTools.Spline;
using static FlightCamera;

namespace MuMech
{
    namespace Landing
    {
        public class DecelerationBurn : AutopilotStep
        {
            const double SPEED_CORRECTION_TIME_CONSTANT = 5.0;//2.0;//0.5;//0.3(thrust wobble);//0.4;//0.5; //0.3; // orig 0.3
            const double MAX_CORRECTION_ANGLE = 3;//1.5(works);//2;//1;  // orig 0.1
            const double MAX_CORRECTION_ANGLE_POS = 1;//2;//1;//2;//1;  // orig 0.1
            const double CORRECT_ONLY_ANGLE_LIMIT = 5;//6(wobble);// 10(wobbly);// 20(wobbly); 
            const double WARP_END_TIME = 5;  // orig 5
            const double WARP_START_ANGLE = 0.5;//5;  // orig 5 degrees
            const double CORRECT_ANGLE_FACTOR = 10;//4(overcorrect);// 2(overcorrect);//10(works pretty good);//8;//10;//2.0; // orig 2
            const double CORRECT_ONLY_ANGLE_FACTOR = 5.0;//4.0(wobble);//5.0;//4.0;//5.0;//2.0;//0.4;//0.2; // 0.1;
            const float CORRECT_ONLY_FACTOR = 0.035f; //0.02f(too little); // 0.05f(still too much); // 0.1f(too much);
            const float CORRECT_ONLY_TFACTOR = 0.1f;
            private const float MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT = 4000;//3000(landing hot earth);//4000; //7000.0F;
            private const float ALT_LOWER_ACCEL_CONSTANT = 7000;//6000;
            private const float ALT_MOVE_TO_TARGET_THRESHOLD_CONSTANT = 9000;//10000;//9000;
            private const float ALT_SPEED_SLOW_CONSTANT = 13000;//12000;//15000(works-short);
            private const float ALT2_MOVE_TO_TARGET_THRESHOLD_CONSTANT = 9000;//10000;//9000;
            private const float ALT2_SPEED_SLOW_CONSTANT = 17000;//17000;//13000;//12000;//15000(works-short);
            private const float ALT_MIN_WARP_CONSTANT = 50000;
            private const float ANGLE_MIN_SPEED_RATIO_CONSTANT = 1.3F;
            private const float ANGLE_ADJUST_I_CONSTANT = 0.003f;//1.0f;//0.1f;0.01f;//0.001f;//0.0001f;//0.00001f;//0.01f;//0.00002f; // 0.00001f;
            private const float ANGLE_ADJUST_P_CONSTANT = 2.0f;//20.0f;//0.2f;0.02f;//0.002f;//5.0f;//0.015f;   // 0.010f;
            private const float ANGLE_OVERSHOOT_I_CONSTANT = 0.01000f;
            private const float ANGLE_OVERSHOOT_P_CONSTANT = 20.00000f; 
            private const float LANDING_GEAR_ALT_CONSTANT = 200.0F;
            private const float FINAL_SPEED_FACTOR_CONSTANT = 0.8F;
            private const int MAX_OVERSHOOT_COUNT = 100;
            private const double OVERSHOOT_ANGLE_FRACTION = 0.9; // Small margin to avoid losing vertical control
            private const double LIMITED_MAX_THRUST_G_RATIO = 3.125; // this is multiplied with Mainbody g
            private const double LIMITED_SLOW_THRUST_G_RATIO = 1.25; // this is multiplied with Mainbody g
            private const double TWR_REFERENCE = 1.5;
            private const double LOW_GRAVITY_THRUST_MAX = 10;// 10 <= (bad)100.0; // 10.0
            private const int LAND_STABILIZE_COUNT = 500; // controls how long to stabilize landing.

            private bool maintainAltitude = true; // While going to fast relative to max speed maintain altitude during burn
            private bool moveToTarget = false; // If overshoot detected or low grav world this is set to will burn toward target while maintaining altitude
            private bool checkWarp = true;

            private double accum = 0;   // Used for integral accumulator
            private bool _deployedGears = false;

            private double overshootUpAngle;
            private double overshootUpAngleSlow;
            private double hPrevTargetError = 0;   // Used to calculate horizontal speed sign.
            private double velaccum = 0;
            private double limitedMaxThrustAccel = 0;
            private double actualLimitedMaxThrustAccel = 0;
            private int hOvershootCount = 0;
            double TWR = 1;
            private IDescentSpeedPolicy _aggressivePolicy;
            private uint ThrottleCount = 0;

            private float altMoveToTargetThreshold;
            private float altSpeedSlow;
            private float speedFactor = 1.0F;

            private bool lowGravMode = false;
            private bool boostTorque = false;
            private double vCorrectionAngle = 0.0;  // range is -1.0 to +1.0
            private int landStabilizeCounter = 0;   // Used to Control how long to stabilize landing.
            private int hControlCounter;            // Allows horizontal pid after integration if vertical control is not operating.
            double g;

            public DecelerationBurn(MechJebCore core) : base(core)
            {
                g = MainBody.GeeASL * Core.Landing.EARTH_GRAVITY;  // Used to determine if in low gravity world and to calculate minimum angle to maintain vertical velocity

                if ( g < Core.Landing.LOW_GRAVITY) // m/sec^2
                {
                    double tempFactor = 10.0; // 10(baseline)<=100<=1000 Allow large angles - if angles are too small there will not be enough horizontal thrust
                                              // DEBUG
                    tempFactor *= Core.Landing.debug6;
                    lowGravMode = true;
                    maintainAltitude = false;
                    checkWarp = false;
                    moveToTarget = true;
                    limitedMaxThrustAccel = Math.Min(VesselState.limitedMaxThrustAccel, tempFactor * LIMITED_MAX_THRUST_G_RATIO * g);
                    overshootUpAngle = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(limitedMaxThrustAccel, 2) - Math.Pow(g, 2)) / g;
                    overshootUpAngleSlow = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(Math.Min(limitedMaxThrustAccel, tempFactor * LIMITED_SLOW_THRUST_G_RATIO * g), 2) - Math.Pow(g, 2)) / g;
                }
                else
                {
                    limitedMaxThrustAccel = Math.Min(VesselState.limitedMaxThrustAccel, LIMITED_MAX_THRUST_G_RATIO * g);
                    overshootUpAngle = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(limitedMaxThrustAccel, 2) - Math.Pow(g, 2)) / g;
                    overshootUpAngleSlow = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(Math.Min(limitedMaxThrustAccel, LIMITED_SLOW_THRUST_G_RATIO * g), 2) - Math.Pow(g, 2)) / g;
                }
                TWR = (VesselState.limitedMaxThrustAccel / VesselState.localg) / TWR_REFERENCE;

                if ( Core.Landing.FlySafe )
                {
                    altMoveToTargetThreshold = ALT_MOVE_TO_TARGET_THRESHOLD_CONSTANT;
                    altSpeedSlow = ALT_SPEED_SLOW_CONSTANT;
                }
                else
                {
                    altMoveToTargetThreshold = ALT2_MOVE_TO_TARGET_THRESHOLD_CONSTANT;
                    altSpeedSlow = ALT2_SPEED_SLOW_CONSTANT;
                }
            }

            public override AutopilotStep OnFixedUpdate()
            {
                if (VesselState.altitudeTrue < ALT_MIN_WARP_CONSTANT)
                {
                    checkWarp = false;
                }

                if (checkWarp == true)
                {
                    double decelerationStartTime =
                         Core.Landing.Prediction.Trajectory.Any() ? Core.Landing.Prediction.Trajectory.First().UT : VesselState.time;
                    double warpEndTime = WARP_END_TIME;
                    double desiredSpeed = GetMaxSpeed(true); 

                    if (VesselState.speedSurface > Math.Abs(desiredSpeed))
                    {
                        warpEndTime += ((VesselState.speedSurface - Math.Abs(desiredSpeed)) / Math.Abs(desiredSpeed)) * VesselState.localg;
                    }

                    if ((decelerationStartTime - VesselState.time) > warpEndTime)
                    {
                        Core.Thrust.TargetThrottle = 0;

                        Status = Localizer.Format("#MechJeb_LandingGuidance_Status4"); //"Warping to start of braking burn."
                        Localizer.Format("#MechJeb_LandingGuidance_Status6",
                            Math.Abs(desiredSpeed) >= double.MaxValue ? "∞" : Math.Abs(desiredSpeed).ToString("F1")); //"Braking: target speed = " +  + " m/s"

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

            private double GetMaxSpeed(bool updatePolicy, double dt = 0)
            {
                double maxSpeed;

                if (VesselState.altitudeTrue > MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT)
                {
                    if (Core.Landing.FlySafe)
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
                    else
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
                }
                else
                {
                    maxSpeed = Mathf.Lerp(0,
                        (float)Math.Sqrt((limitedMaxThrustAccel - VesselState.localg) * 2 * MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT) * FINAL_SPEED_FACTOR_CONSTANT, (float)VesselState.altitudeTrue / MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT);
                }

                return -maxSpeed;
            }

            public void pidVelocity(double desiredSpeed, double currentSpeed )
            {
                const double PID_ACCUM_MAX = 20;
                const double PID_KP = 20;
                const double PID_KI = 0.15;// 0.2;// 0.1;
                double verror = (desiredSpeed - currentSpeed);
                double _limitedMaxThrustAccel;

                if ( boostTorque )
                {
                    _limitedMaxThrustAccel = VesselState.limitedMaxThrustAccel;
                }
                else
                {
                    _limitedMaxThrustAccel = actualLimitedMaxThrustAccel;
                }
                verror /= _limitedMaxThrustAccel;

                velaccum += verror;
                velaccum = Math.Max(0, Math.Min(PID_ACCUM_MAX, velaccum));
                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;

                Vessel.ctrlState.mainThrottle = Core.Thrust.TargetThrottle = (float)
                    ( Math.Max(0,Math.Min(_limitedMaxThrustAccel, PID_KI * velaccum + PID_KP * verror)) / VesselState.maxThrustAccel );
            }

            public void MoveToTarget(ref double desiredVerticalSpeed, ref Vector3d desiredThrustVector )
            {
                // Get Horizontal vector towards the target and combine with the up vector. The resultant vector
                // will start at a 45 degree angle (overhootUpAngle = 1.0) and will be adjusted to ensure
                // vertical velocity can be maintained.
                Vector3d courseCorrection = (Core.Target.GetPositionTargetPosition() - VesselState.CoM);
                courseCorrection = Vector3d.Exclude(VesselState.up, courseCorrection);

                // Using the current horizontal velocity and desired horizontal velocity calculate the desired
                // thrust vector.  An velocity error vector will be calculated.
                // A speed up would be an error vector in the direction of target
                // A speed down would be an error vector opposite in the direction of target.
                double hTargetError = courseCorrection.magnitude;
                double ratio = hTargetError / VesselState.altitudeTrue;

                if (VesselState.altitudeTrue > MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT)
                {
                    const float MAX_RATIO = 1.2f;
                    const float MIN_RATIO = 0.02f;

                    if ((ratio > MAX_RATIO) )
                    {
                        desiredVerticalSpeed = 0;
                    }
                    else if (ratio > MIN_RATIO)
                    {
                        desiredVerticalSpeed = -(float)Math.Abs(desiredVerticalSpeed) * (MAX_RATIO - (float)ratio) / (MAX_RATIO - MIN_RATIO);
                    }
                    else
                    {
                        desiredVerticalSpeed = -(float)Math.Abs(desiredVerticalSpeed);
                    }
                }
                else
                {
                    const float MAX_RATIO = 0.15f;
                    const float MIN_RATIO = 0.01f;

                    FinalDescent(ref desiredVerticalSpeed);
                    if ((ratio > MAX_RATIO) )
                    {
                        desiredVerticalSpeed = 0;
                    }
                    else if (ratio > MIN_RATIO)
                    {
                        desiredVerticalSpeed = -(float)Math.Abs(desiredVerticalSpeed) * (MAX_RATIO - (float)ratio) / (MAX_RATIO - MIN_RATIO);
                    }
                    else
                    {
                        desiredVerticalSpeed = -(float)Math.Abs(desiredVerticalSpeed);
                    }
                }

                // Use local PI control - more damped and works over wide range of parameters
                desiredVerticalSpeed = -Math.Abs(desiredVerticalSpeed);
                pidVelocity(desiredVerticalSpeed, VesselState.speedVertical);

                double divisor = (hTargetError > 100) ? 10 : (hTargetError > 10.0) ? 200 : 400;
                double _overshootUpAngle;
                double verror = desiredVerticalSpeed - VesselState.speedVertical;
                double maxHThrust = Math.Min(5, limitedMaxThrustAccel);
                if ( hTargetError < 10 )
                {
                    _overshootUpAngle = 0.025;
                    maxHThrust = VesselState.localg * 1.25;
                }
                else if (hTargetError < 10000)
                {
                    maxHThrust = VesselState.localg * 1.25;
                    _overshootUpAngle = overshootUpAngleSlow;
                    if (verror > 10)
                    {
                        _overshootUpAngle *= 0.61;
                    }
                }
                else
                {
                    if (hTargetError < 20000)
                    {
                        maxHThrust = VesselState.localg * 1.25;
                    }
                    _overshootUpAngle = overshootUpAngle;
                    if (verror > 10)
                    {
                        _overshootUpAngle *= 0.61;
                    }
                }

                Vector3d desiredHorizontalVel = (Math.Sqrt(Math.Abs(hTargetError) * maxHThrust / divisor) * 2)
                    * courseCorrection.normalized;
                Vector3d error = desiredHorizontalVel - Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);

                // The strategy here is to keep vertical velocity to zero and control the horizontal
                // velocity to move towards the target.
                if (hTargetError < 0.25)
                {
                    error = -Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                }

                double correctionAngle = Math.Max(0, Math.Min(_overshootUpAngle, (ANGLE_OVERSHOOT_P_CONSTANT * error.magnitude) / 100.0));
                desiredThrustVector = (VesselState.up + correctionAngle * error.normalized).normalized;
            }

            // Low Gravity version for MoveToTarget.
            /// <summary>
            ///  Since the gravity is almost a non factor, most of the vertical velocity control requires
            ///  pitching the craft towards the surface facing forward and backwords.
            ///  For zero vertical velocity the following is expected:
            ///      1. No horizontal velocity change - facing up
            ///      2. Forward acceleration - facing forward with downward pitch
            ///      3. Forward deceleration - facing backward with slightly upward pitch.
            ///   Since there will be wild swings in orientation well need to keep the thrust at zero until within
            ///   range of desired orientation.
            /// </summary>
            /// <param name="desiredVerticalSpeed"></param>
            /// <param name="desiredThrustVector"></param>
            public void MoveToTargetLG(ref double desiredVerticalSpeed, ref Vector3d desiredThrustVector)
            {
                double baseGain = 0.196 / (MainBody.GeeASL * Core.Landing.EARTH_GRAVITY);

                // Get Horizontal vector towards the target and combine with the up vector. The resultant vector
                // will start at a 45 degree angle (overhootUpAngle = 1.0) and will be adjusted to ensure
                // vertical velocity can be maintained.
                Vector3d courseCorrection = (Core.Target.GetPositionTargetPosition() - VesselState.CoM);
                courseCorrection = Vector3d.Exclude(VesselState.up, courseCorrection);

                // Using the current horizontal velocity and desired horizontal velocity calculate the desired
                // thrust vector.  An velocity error vector will be calculated.
                // A speed up would be an error vector in the direction of target
                // A speed down would be an error vector opposite in the direction of target.
                double hTargetError = courseCorrection.magnitude;
                double ratio = hTargetError / VesselState.altitudeTrue;
                float thresholdConstant = MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT;
                float maxRatio;
                float minRatio;

                if (lowGravMode)
                {
                    thresholdConstant *= 2.0f;  // For low gravity world double the altitude threshold.
                }

                // far above the ground - set more aggressive parameters.
                if (VesselState.altitudeTrue > thresholdConstant)
                {
                    if (lowGravMode)
                    {
                        maxRatio = 2.0f; // 2.2=3.6(causes vvel zero out at 8000)<=1.5<=2.4 desired vertical speed is set to zero above this ratio
                        minRatio = 0.3f; // 0.3<=0.7desired vertical speed is not modified below this ratio (ratio == 0 means its directly overhead) 
                    }
                    else
                    {
                        maxRatio = 1.2f;  // desired vertical speed is set to zero above this ratio
                        minRatio = 0.02f; // desired vertical speed is not modified below this ratio (ratio == 0 means its directly overhead) 
                    }
                }

                // Closer to the ground - change parameters to avoid crashing.
                else
                {
                    if (lowGravMode)
                    {
                        maxRatio = 0.35f; // 0.35(baseline) desired vertical speed is set to zero above this ratio
                        minRatio = 0.01f; // desired vertical speed is not modified below this ratio (ratio == 0 means its directly overhead) 
                    }
                    else
                    {
                        maxRatio = 0.15f; // desired vertical speed is set to zero above this ratio
                        minRatio = 0.01f; // desired vertical speed is not modified below this ratio (ratio == 0 means its directly overhead) 
                    }
                    FinalDescent(ref desiredVerticalSpeed); // Ramps desired vertical velocity based on altitude.
                }

                // hold altitude until we get closer.
                if (ratio > maxRatio)
                { 
                    desiredVerticalSpeed = 0; 
                }
                // fade desired vertical velocity to zero the farther we are from target
                else if (ratio > minRatio)
                {
                    desiredVerticalSpeed = -(float)Math.Abs(desiredVerticalSpeed) * (maxRatio - (float)ratio) / (maxRatio - minRatio);
                }
                // In the ballpark - do not change desired vertical velocity.
                else
                {
                    desiredVerticalSpeed = -(float)Math.Abs(desiredVerticalSpeed);
                }

                // div = baseDiv*(20.408*g)                                      safe                                 notsafe
                // bop    = 0.589   bop/gilly    = 12.020   /2 = 6.010           100/200/400                          12.02/120.20/180.6
                // pol    = 0.373   pol/gilly    =  7.612   /2 = 3.806
                // gilly  = 0.049   gilly/gilly  = 1        /2 = 0.5             16.63/33.28/66.55                    2/20/30
                // phobos = 0.0057  phobos/gilly = 0.116    /2 = 0.116 
                // deimos = 0.003   deimos/gilly = 0.0612   /2 = 0.0306


                double hCloseLimit = 20;   // (baseline)20
                double hMidLimit   = 1000; // 1000<=500(baseline)<=100
                double hFarLimit = 10000;  // (baseline)10000
                double divClose = ((Core.Landing.FlySafe) ? 66.55 : 30);
                double divMid   = ((Core.Landing.FlySafe) ? 33.28 : 20);
                double divFar   = ((Core.Landing.FlySafe) ? 16.63 : 2);
                double divisor = (hTargetError > hMidLimit) ? divFar : (hTargetError > hCloseLimit) ? divMid : divClose;  // 100-200-400(pol)<=(baseline2-gilly)2-20-30<=(baseline1)2-50-90
                double _overshootUpAngle;
                double maxHThrust = Math.Min(5, limitedMaxThrustAccel);
                double gainCancelHVelocity = (hTargetError <= hMidLimit) ? baseGain : baseGain/13.0; // gilly=4(hTargetError < 500)

                // Scale divisor based on local gravity and cap the values.
                divisor = Math.Max(2, Math.Min(400,divisor * (0.4*20.408) * g));

                // Controls Max angle to move horizontally.
                if (hTargetError < hCloseLimit)
                {
                    _overshootUpAngle = 0.025 * 36; //  (baseline)0.025 * 36<=0.025 * 18; //  0.025 * 9; // 0.025*3;
                    maxHThrust = VesselState.localg * LOW_GRAVITY_THRUST_MAX;
                }
                else if (hTargetError < hFarLimit)
                {
                    maxHThrust = VesselState.localg * LOW_GRAVITY_THRUST_MAX;
                    _overshootUpAngle = overshootUpAngleSlow;
                }
                else
                {
                    _overshootUpAngle = overshootUpAngle;
                }

                // DEBUG
                gainCancelHVelocity *= Core.Landing.debug3; // 4(baseline gilly(4)
                divisor             *= Core.Landing.debug4; // 0.4(baseline)  gilly(0.4)
                _overshootUpAngle   *= Core.Landing.debug5; // 1(baseline)

                // Set the desired Horizontal Velocity to reach target. Error to subtract the vessels current horizontal velocity.
                Vector3d desiredHorizontalVel = (Math.Sqrt(hTargetError * maxHThrust / divisor) * 2) * courseCorrection.normalized;
                Vector3d error = desiredHorizontalVel - Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);

                // The strategy here is to keep vertical velocity to zero and control the horizontal
                // velocity to move towards the target.
                // When the horizontal position error is really small (meters) just zero out the horizontal velocity.
                if (hTargetError < 0.25)
                {
                    error = -gainCancelHVelocity * Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                }
                else
                // Zero out the horizontal velocity that is not part of the desired horizontal velocity - avoids a limit cycle.
                {
                    error -= gainCancelHVelocity*Vector3d.Exclude(desiredHorizontalVel, Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity));
                }

                // Calculate horizontal correction angle using horizontal error.
                double hCorrectionAngle = Math.Max(0, Math.Min(_overshootUpAngle, (baseGain*ANGLE_OVERSHOOT_P_CONSTANT * error.magnitude) / 100.0));  // 4<=1<=10<=100<=3   gilly:4


                // The desired thrust vector can pitch up or down depending on desired vertical velocity since gravity is not much 
                // help here.
                double verror = desiredVerticalSpeed - VesselState.speedVertical;

                if ((verror < -3) || (verror > 0.3)) // hysterisis to minimize flopping around
                {
                    vCorrectionAngle = Math.Max(-1.0, Math.Min(1.0, vCorrectionAngle + verror / 6000.0));
                    hControlCounter = 0;
                }

                // Horizontal Control only works to reduce horizontal velocity, not to increase it.
                if (hControlCounter > 600)
                {
                    // Only exclude desired horizontal velocity when vertical velocity is being maintained in orbit - this has the effect of correcting 
                    // plane to target . Otherwise, allow total horizontal velocity reduction control.
                    if ( (desiredVerticalSpeed >= 0.0) && (VesselState.orbitPeA > 0.9* VesselState.orbitApA) )
                    {
                        error = -Vector3d.Exclude(desiredHorizontalVel, Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity));
                    }
                    pidVelocity(0.0, (error.magnitude>4) ? -error.magnitude:0);
                }
                else
                {
                    // Use local PI control - more damped and works over wide range of parameters
                    if (vCorrectionAngle >= 0)
                    {
                        // Only applies thrust if need to increase vertical speed
                        pidVelocity(desiredVerticalSpeed, VesselState.speedVertical);
                    }
                    else
                    {
                        // Only applies thrust if need to decrease vertical speed
                        pidVelocity(-desiredVerticalSpeed, -VesselState.speedVertical);
                    }
                    if ( Core.Thrust.TargetThrottle == 0 )
                    {
                        hControlCounter++; // No vertical control needed - integrate to allow horizontal thrust.
                    }
                }

                // DEBUG
                vCorrectionAngle *= Core.Landing.debug1;
                hCorrectionAngle *= Core.Landing.debug2;

                // NOTES:
                // At Gilly need hCorrectionAngle gain at 4.0, but at Pol we need 1.0
                // At Gilly gainCancelHVelocity works with 4.0, but at Pol appears to be overgained. TODO: Try 1.0 at Pol.
                // Pattern: Gilly=4  Pol=1    ratio=GillyGravity/Gravity where Gilly/Gilly = 4.0*1.0, Pol = 4.0*(0.049/0.373) = 0.525
                //  or gain = 0.196/gravity

                // Calculate final desired thrust vector.
                desiredThrustVector = (vCorrectionAngle*VesselState.up + hCorrectionAngle * error).normalized;

                // DEBUG
                MechJebModuleDebugArrows.debugVector = vCorrectionAngle * VesselState.up;
                switch(Core.Landing.selectDebugVector)
                {
                    default:
                    case 0: MechJebModuleDebugArrows.debugVector2 = courseCorrection; break;
                    case 1: MechJebModuleDebugArrows.debugVector2 = courseCorrection.normalized; break;
                    case 2: MechJebModuleDebugArrows.debugVector2 = error; break;
                    case 3: MechJebModuleDebugArrows.debugVector2 = hCorrectionAngle * error; break;
                    case 4: MechJebModuleDebugArrows.debugVector2 = error.normalized; break;
                    case 5: MechJebModuleDebugArrows.debugVector2 = desiredHorizontalVel; break;
                }

                // If angle between current and desired thrust vector is too large then set thrust to zero. Thrust will only
                // occur when in the ball park.
                if ( Core.Attitude.attitudeAngleFromTarget() > 30 ) // 30<=40(overcorrect)<=30<=15(jittery)<=30<=5<=15
                {
                    Vessel.ctrlState.mainThrottle = Core.Thrust.TargetThrottle = 0;
                }
                desiredVerticalSpeed = hTargetError;
            }

            public void RetroBurnMaintainAltitude(ref double desiredSpeed, ref Vector3d desiredThrustVector)
            {
                // If the surfaceSpeed is significantly larger then the targeted speed
                // Attempt to minimize altitude loss to allow more time to burn the horizontal speed, othwerwise
                // the vessel will crash into terrain.
                // This is is definitely needed when starting from a low orbit with a low TWR.
                if (VesselState.speedSurface > Math.Abs(desiredSpeed) * ANGLE_MIN_SPEED_RATIO_CONSTANT)
                {
                    Vector3d courseCorrection = Core.Landing.ComputeCourseCorrection(false, 20.0);
                    double _correctionAngle = (courseCorrection.magnitude / (CORRECT_ANGLE_FACTOR*TWR));
                    double correctionAngle = Math.Min(MAX_CORRECTION_ANGLE_POS/10, _correctionAngle);
                    double targetVerticalSpeed = -VesselState.altitudeTrue * 0.00005; //0.0005;// 0.00111(closecall);// 0.00333(crash);
                    double error = targetVerticalSpeed - VesselState.speedVertical;
                    desiredThrustVector = (desiredThrustVector + correctionAngle * courseCorrection.normalized).normalized;
                    accum = Math.Min(10000, Math.Max(0, accum+error));
                    double angle = (ANGLE_ADJUST_I_CONSTANT*accum + ANGLE_ADJUST_P_CONSTANT*error)/VesselState.limitedMaxThrustAccel;
                    desiredThrustVector += Math.Min(12, Math.Max(0, angle)) * VesselState.up.normalized;

                    Core.Thrust.TargetThrottle = 1.0F;

                    // Determine if an overshoot occurs.
                    Vector3d hTargetDirection = Vector3d.Exclude(VesselState.up, Core.Target.GetPositionTargetPosition() - VesselState.CoM);
                    double hTargetError = hTargetDirection.magnitude;
                    double hHorizontalSign = Math.Sign(hTargetError - hPrevTargetError);
                    hPrevTargetError = hTargetError;
                    if ( hHorizontalSign > 0 )
                    {
                        hOvershootCount++;
                        if ( hOvershootCount >= MAX_OVERSHOOT_COUNT )
                        {
                            moveToTarget = true; // Overshoot correction will occur once we leave this state.
                            hOvershootCount = 0;
                            speedFactor = FINAL_SPEED_FACTOR_CONSTANT;
                        }
                    }
                }

                // No correction needed so use retrograde unchanged
                else
                {
                    Core.Thrust.TargetThrottle = 0;
                    maintainAltitude = false;
                }

                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;
            }

            public void RetroBurn(ref double desiredSpeed, ref Vector3d courseCorrection, ref Vector3d desiredThrustVector)
            {
                double controlledSpeed = VesselState.speedSurface * Math.Sign(Vector3d.Dot(VesselState.surfaceVelocity, VesselState.up)); //positive if we are ascending, negative if descending
                double desiredSpeedAfterDt = GetMaxSpeed(true, VesselState.deltaT);
                double minAccel = -VesselState.localg * Math.Abs(Vector3d.Dot(VesselState.surfaceVelocity.normalized, VesselState.up));
                double maxAccel = VesselState.limitedMaxThrustAccel * Vector3d.Dot(VesselState.forward, -VesselState.surfaceVelocity.normalized) -
                                  VesselState.localg * Math.Abs(Vector3d.Dot(VesselState.surfaceVelocity.normalized, VesselState.up));

                double speedError;
                double desiredAccel;

                speedError = desiredSpeed - controlledSpeed;
                if (VesselState.altitudeTrue < altSpeedSlow)
                {
                    desiredSpeedAfterDt *= limitedMaxThrustAccel / VesselState.limitedMaxThrustAccel;
                }
                desiredAccel = speedError / SPEED_CORRECTION_TIME_CONSTANT + (desiredSpeedAfterDt - desiredSpeed) / VesselState.deltaT;

                if ( (desiredAccel - minAccel) > 0 )
                {
                    Core.Thrust.TargetThrottle = Mathf.Clamp((float)((desiredAccel - minAccel) / (maxAccel - minAccel)), 0.0F, 1.0F);
                }
                else if ( VesselState.altitudeASL > Core.Landing.DecelerationEndAltitude() + 5 )
                {
                    // perform correction for high altitude
                    if ( VesselState.altitudeTrue > 30000 )
                    {
                        double _correctionAngle = courseCorrection.magnitude / (2 * TWR);
                        double correctionAngle = Math.Min(CORRECT_ONLY_ANGLE_LIMIT, _correctionAngle);
                        desiredThrustVector = (-VesselState.surfaceVelocity.normalized + correctionAngle * courseCorrection.normalized).normalized;
                        Core.Thrust.TargetThrottle = Mathf.Min(1, (float)(correctionAngle * 0.2));
                    }

                    // Approaching target thru atmosphere - perform small corrections since otherwise thrusters would be off.
                    else
                    {
                        double _correctionAngle = courseCorrection.magnitude / (CORRECT_ONLY_ANGLE_FACTOR * TWR);
                        double correctionAngle = Math.Min(CORRECT_ONLY_ANGLE_LIMIT, _correctionAngle);
                        desiredThrustVector = (-VesselState.surfaceVelocity.normalized + correctionAngle * courseCorrection.normalized).normalized;
                        Core.Thrust.TargetThrottle = Mathf.Min(CORRECT_ONLY_TFACTOR, (float)correctionAngle * CORRECT_ONLY_FACTOR);
                    }
                }
                else
                {
                    Core.Thrust.TargetThrottle = 0;
                }

                if ( Core.Thrust.TargetThrottle > 0.01 )
                {
                    ThrottleCount = 1000;
                }
                else if ( ThrottleCount > 0 )
                {
                    ThrottleCount--;
                }

                if ( (ThrottleCount>0) && (Core.Thrust.TargetThrottle < 0.0001 ) )
                {
                    Core.Thrust.TargetThrottle = 0.001f;
                }

                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;
            }

            //
            // Targeted Final Descent:
            //   - If Close to Target & Very Small Horizontal Velocity - Proceed to Land using MaxAllowedSpeed with little horizontal adjustment.
            //   - Else - Slow down to zero Vertical Speed (based on True Altitude) and Continue to Correct to Target.
            //
            public void FinalDescent(ref double desiredSpeed)
            {
                float thresholdConstant = MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT;

                if ( lowGravMode )
                {
                    thresholdConstant *= 2.0f;
                }

                desiredSpeed = -Mathf.Clamp01((float)VesselState.altitudeTrue / thresholdConstant) *
                    (float)Math.Sqrt((limitedMaxThrustAccel - VesselState.localg) * 2 * thresholdConstant)
                    * FINAL_SPEED_FACTOR_CONSTANT;
                desiredSpeed = (float)Math.Min(-Core.Landing.TouchdownSpeed, desiredSpeed);
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // Don't do anything until warp is complete
                if (checkWarp == true)
                {
                    return this;
                }

                // Landing Completed - exit
                if (Vessel.LandedOrSplashed || (landStabilizeCounter!=0))
                {
                    Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                    Core.Thrust.TransKillH = false;
                    Core.Thrust.TargetThrottle = 0;
                    Core.Thrust.TransSpdAct = 0;
                    if ( landStabilizeCounter < LAND_STABILIZE_COUNT)
                    {
                        landStabilizeCounter++;
                        Core.Attitude.attitudeTo(VesselState.forward, AttitudeReference.INERTIAL, Core.Landing);
                    }
                    else
                    {
                        Core.Landing.StopLanding();
                        return null;
                    }
                }

                // Consider lowering the landing gear
                if (!_deployedGears)
                {
                    if (VesselState.altitudeTrue < LANDING_GEAR_ALT_CONSTANT)
                    {
                        Vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
                        _deployedGears = true;
                    }
                }

                // Set Acceleration Limit - At approach to target limit it to 
                // a lower level to gain more control
                actualLimitedMaxThrustAccel = VesselState.limitedMaxThrustAccel;
                if (VesselState.altitudeTrue < ALT_LOWER_ACCEL_CONSTANT)
                {
                    actualLimitedMaxThrustAccel = limitedMaxThrustAccel;
                }

                // Set desired speed
                // As deorbit approaches end scale it down to lower speed based on
                // lower acceleration limit, but do this at a higher altitude
                // to take advantage of available thrust so it can reach desired speed quickly.
                double desiredSpeed = GetMaxSpeed(true);
                if ( (VesselState.altitudeTrue < altSpeedSlow) || (lowGravMode) )
                {
                    desiredSpeed *= limitedMaxThrustAccel / VesselState.limitedMaxThrustAccel;
                }

                Vector3d desiredThrustVector = -VesselState.surfaceVelocity.normalized;

                // Initial strategy to reducde horizontal speed.
                if ( maintainAltitude == true )
                {
                    RetroBurnMaintainAltitude(ref desiredSpeed,ref desiredThrustVector);
                }
                else if ( moveToTarget == true )
                {
                    if ( lowGravMode )
                    {
                        MoveToTargetLG(ref desiredSpeed, ref desiredThrustVector);
                    }
                    else
                    {
                        MoveToTarget(ref desiredSpeed, ref desiredThrustVector);
                    }
                }
                else
                {
                    Vector3d courseCorrection = Vector3d.zero;

                    // Perform Retroburn
                    if ( VesselState.altitudeTrue > altMoveToTargetThreshold )
                    {
                        courseCorrection = Core.Landing.ComputeCourseCorrection(true, 20.0);

                        // Apply correction if angle is not too steep towards the ground
                        if (Math.Abs(Vector3d.Angle(courseCorrection, VesselState.up)) < 100)
                        {
                            double _correctionAngle = courseCorrection.magnitude / (CORRECT_ANGLE_FACTOR * TWR);
                            double correctionAngle = Math.Min(MAX_CORRECTION_ANGLE/* TWR*/, _correctionAngle);

                            desiredThrustVector = (desiredThrustVector + correctionAngle * courseCorrection.normalized).normalized;
                        }
                        RetroBurn(ref desiredSpeed, ref courseCorrection, ref desiredThrustVector);
                    }
                    else
                    {
                        moveToTarget = true;
                        speedFactor = FINAL_SPEED_FACTOR_CONSTANT;
                    }
                }

                Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, Core.Landing);
                Status = Localizer.Format("#MechJeb_LandingGuidance_Status6",
                         Math.Abs(desiredSpeed) >= double.MaxValue ? "∞" : desiredSpeed.ToString("F1")); //"Braking: target speed = " +  + " m/s"
                return this;
            }
        }
    }
}
