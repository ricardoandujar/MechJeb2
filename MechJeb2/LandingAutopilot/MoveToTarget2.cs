using System;
using System.Linq;
using KSP.Localization;
using Steamworks;
using UnityEngine;
using static alglib;
using static SoftMasking.SoftMask;
using static UnityEngine.TouchScreenKeyboard;

namespace MuMech
{
    namespace Landing
    {
        public class MoveToTarget2 : AutopilotStep
        {
            enum Ascend2Target
            {
                Disabled,    // Sub Orbital targetting disabled
                InitialBurn, // Perform initial burn for sub orbital targetting
                FineTune,    // First sub orbital fine tuning to target 
                Completed    // Sub Orbital targetting done
            }

            enum DecelerateSteps
            {
                DisabledDec,    // Initial state - no modifiers employed - short hops
                IHorizontalDec, // (maxThrust) need to reduce horizontal speed based on velocity pitch (not attitude pitch)
                                // Horizontal is primary with some vertical control
                IVerticalDec,   // (maxThrust) Go here once Horizontal under control, some horizontal control.
                                // Vertical is primary with some horizontal control as long as vertical is ok
                HorizontalDec,  // Perform Horizontal Control when Vertical Control is zero thrust
                VerticalDec     // Normal Vertical Control to final landing. 
            }

            private const float MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT = 4000;// 4000(eve-die);//3000(landing hot earth);//4000; //7000.0F;
            private const float SAFE_MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT = 8000;// 4000(eve-die);//3000(landing hot earth);//4000; //7000.0F;
            private const float ALT_LOWER_ACCEL_CONSTANT = 7000;//6000; // Lower TWR Max below this altitude to gain more control.
            private const float SAFE_ALT_SPEED_SLOW_CONSTANT = 23000;//13000;//12000;//15000(works-short);
            private const float ALT_SPEED_SLOW_CONSTANT = 18000;//17000;//17000;//13000;//12000;//15000(works-short);  Under this Altitude scale down the vertical speed to landing. 
            private const float ALT_MIN_WARP_CONSTANT = 19000; // 19000 <= 50000 Do not warp below this altitude
            private const float H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT = 10000;  // When above this horizontal distance to target and below the altitude threshold, use suborbital targetting to approach target, otherwise use normal targetting. This allows more aggressive approach when far from target and more precise control when close to target.
            private const float H_SUBORBITAL_ALTITUDE_THRESHOLD_CONSTANT = 1000;   // For High gravity use suborbital when above this alt, otherwise use normal targetting

            private const float H_CORRECTION_ANGLE_CONSTANT = 0.4f; // Used to determine horizontal correction angle 
            private const float LANDING_GEAR_ALT_CONSTANT = 200.0F; // Drop landing gear below this altitude
            private const float SAFE_FINAL_SPEED_FACTOR_CONSTANT = 0.8F; // To avoid crashing in terrain in final descent
            private const double OVERSHOOT_ANGLE_FRACTION = 0.9; // Small margin to avoid losing vertical control
            public const double LIMITED_MAX_THRUST_G_RATIO = 5.0; // 10<=5.0 <=4 <= 3.125 this is multiplied with Mainbody g
            private const double LIMITED_SLOW_THRUST_G_RATIO = 1.25; // this is multiplied with Mainbody g
            private const double LOW_GRAVITY_THRUST_MAX = 10;// 10 <= (bad)100.0; // 10.0
            private const int LAND_STABILIZE_COUNT = 500; // controls how long to stabilize landing.
            private const int IGNORE_LANDING_COUNT = 100; // controls how long to ignore landing check.
            private const int IGNORE_ERR_INTEGRATE_COUNT = 40;  // Initial ignore count when ascending in a hop
            private const int DECREASE_ERR_INTEGRATE_COUNT = 6; // Integration count to determine when closest approach reached
            private const int INCREASE_ERR_INTEGRATE_COUNT = 6; // Integration count to determine when cannot improve approach
            private const float EMERGENCY_ALTITUDE   = 400.0f;  // below this altitude activate emergency ascent
            private const float EMERGENCY_H_DISTANCE = 500.0f;  // beyond this distance activeate emergency ascent
            private const double ASCENT_H_DISTANCE_DIVISOR = 2000.0; // used to calculate desired vertical speed
            private const double ASCENT_AOA_MIN_ALT = 2000.0;   // minimun altitude where desired vertical speed is set to zero.
            private const double MIN_ASCENT_ANGLE = 1.75; // minimum angle to increase horizontal speed
            private bool stopIncrease = false;  // Set to true when need to stop vertical increase
            private bool checkWarp = true;      // Only enabled for retro burn to wait for right moment to burn
            private bool warpOn = false;        // Used to track if warp is on to end it at the right time 
            private int suppressThrottle;       // Used to control throttle suppression (=1) while performing sub orbital targeting
            private bool emergency;             // Used to trigger an emergency ascent to avoid crash 
            private int _deployedGears = 1;     // 0->1->2  - deploy gears twice   2->1->0 store gears twice
            private double hFarCorrectionAngle;             // far horizontal distance overshoot angle
            private double hMidCorrectionAngle;             // mid horizontal distance overshoot angle
            private Vector3d velaccum_h = Vector3d.zero;    // Integral velocity error accumulator for Horizontal PI Controller
            private double velaccum_h2 = 0;                 // Integral velocity error accumulator for Horizontal PI Controller
            private double velaccum_v = 0;                  // Integral velocity error accumulator for Vertical PI Controller
            private double limitedMaxThrustAccel = 0;       // Limited thrust acceleration used to calculate TWR and overshoot angle + scale max speed
            private double actualLimitedMaxThrustAccel = 0; // Calculate actual limited max thrust used at this time.
            private IDescentSpeedPolicy _aggressivePolicy;  // Used to calculate max speed when not flying safe - more aggressive
            private float altSpeedSlow;                     // Under this terrain altitude the vertical velocity will be scaled down a bit.
            private float altLandThreshold;                 // Under this terrain altitude the vertical velocity will be linearly scaled down to zero ( + clamp to landing speed )
            private float speedFactor = 1.0F;               // Used to store the Landing Margin Factor applied to target speed, slows down faster the larger the margin.
            //private double vCorrectionAngle = 0.0;  // range is -1.0 to +1.0
            private int landStabilizeCounter = 0;   // Used to Control how long to stabilize landing.
            private int ignoreLandingCounter = 0;   // Used to Control how long to ignore landing check
            //private int verticalCounter = 0;   // Used to Control how long to ignore landing check
            private double verticalSign = 1;   // Used to control the sign of vertical thrust in low gravity - when upright verticalSign is positive, when pitched forward to speed up it is negative, when pitched back to slow down it is positive. This allows the PID controller to continue to operate even when pitched forward or backward by not allowing the vertical thrust to be negative when pitched forward and not allowing the vertical thrust to be positive when pitched back.
            private double horizontalSign = 1; // Used to control the sign of horizontal thrust - when facing target horizontalSign is positive, when facing away from target horizontalSign is negative. This allows the PID controller to continue to operate even when facing away from target by not allowing the horizontal thrust to be positive when facing away from target and not allowing the horizontal thrust to be negative when facing towards target.
            private int skipCounter = 0;
            private double hCurrentError = 0;
            private double prevError = 0;
            private double startTiltAlt = 0;
            private double tiltDelta = 0;
            private double subOrbitalApA; // When performing suborbital targetting this is the target ApA to aim for to approach the target - it is set based on the body and atmosphere to give good results.
            private double _subOrbitalApA = 0; // Set the Target Orbital ApA based on groundtrack result
            private double hCloseLimit = 50;   // 50 <=(baseline)20
            private double hMidLimit = 1500; // 1500<=1000(baseline)<=500<=100
            private double hFarLimit = 10000;  // (baseline)10000
            private double divClose;
            private double divMid;
            private double divFar;
            double baseGain; // Calibrated gain for horizontal angle and to correct movement that would diverge from target.

            // The steepness controls the ratio curvature - When horizontal distance to target is small we
            // dont want to modify the desired vertical velocity.
            // When horizontally far from target we want to maintain altitude, but slower allow vertical drop
            // as approach target.
            private float steepness;// Defines the steepness to final descent ratio curvature to target.
            private float maxRatio; // Desired Vertical speed is zero at this or larger ratios 
            private float minRatio; // Desired Vertical speed is unmodified at this or smaller ratios

            // Try with course correction.
            private Ascend2Target    errStep = Ascend2Target.Disabled;
            private DecelerateSteps  decStep = DecelerateSteps.DisabledDec;
            private int ignoreCount = IGNORE_ERR_INTEGRATE_COUNT;
            private int decreaseCount = DECREASE_ERR_INTEGRATE_COUNT;
            private int increaseCount = INCREASE_ERR_INTEGRATE_COUNT;

            private double throttleAngle;
            private double attitudeAngleFromThrust;
            private string trace;

            private Vector3d lastDesiredThrustVector;
            private double override_ratio = 0.75;
            private Vector3 pitchYawVector = new Vector3(1f, 0f, 1f);
            private double hFrac = 1;
            private double vFrac = 1;


            public MoveToTarget2(MechJebCore core) : base(core)
            {
                lastDesiredThrustVector = Vector3d.zero;
                double tempFactor = (Core.Landing.FlySafe == false) ? 20.0 : 10.0; // 10(baseline)<=100<=1000 Allow large angles - if angles are too small there will not be enough horizontal thrust
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

                _deployedGears = 1; // This will trigger both store and deploy gears

                if ((VesselState.speedVertical > -1) && (VesselState.altitudeTrue < 5000))
                {
                    errStep = Ascend2Target.Disabled; // disable far target fine tune - Vertical ascent can be enabled if landed or with positive speed at low altitude
                }
                else
                {
                    errStep = Ascend2Target.Completed; // For all other cases do not allow ascent guidance.
                }

                ignoreCount = IGNORE_ERR_INTEGRATE_COUNT;
                decreaseCount = DECREASE_ERR_INTEGRATE_COUNT;
                increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                startTiltAlt = 0; // Initialize to zero so it gets set when tilting towards target within the atmosphere
                if (Core.Landing.g < Core.Landing.LOW_GRAVITY)
                {
                    throttleAngle = 30.0; // Allow throttling if within 5 degrees - in final landing angle will be wider.
                }
                else
                {
                    throttleAngle = 30.0; // Allow throttling if within 30 degrees - in final landing angle will be wider.
                }
                suppressThrottle = 0; // initialize throttle suppression - start by not suppressing
                landStabilizeCounter = 0;

                if ( MainBody.atmosphere == true )
                {
                    subOrbitalApA = 1.2 * MainBody.atmosphereDepth; // 1.2 <= 1.15(still in atmosphere) <= 1.1
                }
                else
                {
                    subOrbitalApA = Math.Max(24000.0,0.08 * MainBody.Radius);
                }

                checkWarp = false; // bypassing retroburn to move to target
                Core.Landing.UseOnlyMoveToTarget = false;
                limitedMaxThrustAccel = Math.Min(VesselState.limitedMaxThrustAccel, tempFactor * LIMITED_MAX_THRUST_G_RATIO * Core.Landing.g);
                hFarCorrectionAngle = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(limitedMaxThrustAccel, 2) - Math.Pow(Core.Landing.g, 2)) / Core.Landing.g;
                hMidCorrectionAngle = OVERSHOOT_ANGLE_FRACTION * Math.Sqrt(Math.Pow(Math.Min(limitedMaxThrustAccel, tempFactor * LIMITED_SLOW_THRUST_G_RATIO * Core.Landing.g), 2) - Math.Pow(Core.Landing.g, 2)) / Core.Landing.g;

                hCloseLimit = 50;   // 50 <=(baseline)20
                hMidLimit = 1500; // 1500<=1000(baseline)<=500<=100
                hFarLimit = 10000;  // (baseline)10000

                if (Core.Landing.FlySafe)
                {
                    altSpeedSlow = SAFE_ALT_SPEED_SLOW_CONSTANT;
                    speedFactor = SAFE_FINAL_SPEED_FACTOR_CONSTANT;
                    altLandThreshold = SAFE_MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT;
                    divClose = 66.55;
                    divMid = 33.28;
                    divFar = 16.63;
                }
                else
                {
                    altSpeedSlow = ALT_SPEED_SLOW_CONSTANT;
                    speedFactor = Mathf.Max(0.5f, Mathf.Min(1.0f, 1.0f - (float)Core.Landing.BurnMarginPerc / 100.0f));
                    if ( MainBody.atmosphere == false )
                    {
                        speedFactor = Mathf.Min(0.90f, speedFactor); // Airless worlds need a minimum vertical margin to avoid crashing
                    }
                    else if (Core.Landing.g < 0.5 * Core.Landing.EARTH_GRAVITY)
                    {
                        speedFactor *= 0.95f; // assuming lower gravity worlds with thinner atmosphere 
                    }
                    altLandThreshold = MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT;
                    divClose = 30;
                    divMid = 20;
                    divFar = (MainBody.atmosphere == false) ? 1.10 : 1;  // noatmos:1.25  atmos:1 <= 2
                }
            }
            private double GetMaxSpeed()
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
                    if ((Core.Landing.FlySafe == false) || (VesselState.atmosphericDensityGrams < 1.0 ))
                    {
                        double alt = VesselState.altitudeTrue;

                        if (_aggressivePolicy == null)
                        {
                            Core.Target.GetPositionTargetPosition();
                            _aggressivePolicy = new GravityTurnDescentSpeedPolicy(Core.Target.GetPositionTargetPosition().magnitude, MainBody.GeeASL * 9.81, VesselState.limitedMaxThrustAccel); // this constant policy creation is wastefull...
                        }
                        maxSpeed = _aggressivePolicy.MaxAllowedSpeed(VesselState.CoM - MainBody.position,
                            VesselState.surfaceVelocity, Core.Target.GetPositionTargetPosition().magnitude);
                        maxSpeed = speedFactor * Math.Max(maxSpeed, Math.Sqrt((VesselState.limitedMaxThrustAccel - VesselState.localg) * 2 * alt));
                    }
                    else
                    {
                        maxSpeed = Core.Landing.MaxAllowedSpeed();
                    }
                }

                if ((VesselState.altitudeASL < MainBody.RealMaxAtmosphereAltitude()))
                {
                    maxSpeed = Math.Min(Core.Landing.atmosSafeSpeed, Math.Abs(maxSpeed));
                };

                if ((Core.Landing.FlySafe == false) && ((VesselState.altitudeTrue < altSpeedSlow) || (Core.Landing.g < Core.Landing.LOW_GRAVITY)))
                {
                    maxSpeed *= limitedMaxThrustAccel / VesselState.limitedMaxThrustAccel;
                }

                return (float)Math.Min(-Core.Landing.TouchdownSpeed, -Math.Abs(maxSpeed));
            }

            public Vector3d pidVelocity_h(Vector3d herror)
            {
                const double PID_ACCUM_MAX = 60;// 50; // 50<=30 <= 20
                // Increase PI Gains in higher gravity worlds - to track desired vertical speed
                double PID_KP = hFrac*Core.Landing.debug1h * Math.Max(20, Math.Min(50, 20 + (50 - 20) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 50(highg);// 40; // 40<=30 <= 20
                double PID_KI = hFrac*Core.Landing.debug2h * Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 0.05(lowg) 0.175(highg);// 0.35;// 0.25;// 0.15;// 0.2;// 0.1;
                double _limitedMaxThrustAccel = actualLimitedMaxThrustAccel;

                if (_limitedMaxThrustAccel < VesselState.limitedMaxThrustAccel)
                {
                    _limitedMaxThrustAccel = actualLimitedMaxThrustAccel + (VesselState.limitedMaxThrustAccel - actualLimitedMaxThrustAccel)
                        * (Core.Landing.g - 0.05) / (23 - 0.05);
                }
                herror /= _limitedMaxThrustAccel;
                velaccum_h += herror;
                velaccum_h = velaccum_h.normalized*Math.Max(0, Math.Min(PID_ACCUM_MAX, velaccum_h.magnitude));
                if (herror.magnitude > Core.Landing.debug92) velaccum_h = Vector3d.zero;
                double gain;
                double herror_ = herror.magnitude;
                if (herror_ < Core.Landing.debug3h) gain = Core.Landing.debug4h;
                else if (herror_ < Core.Landing.debug5h) gain = Core.Landing.debug4h + Core.Landing.debug6h * Mathf.Clamp01((float)((herror_ - Core.Landing.debug3h) / (Core.Landing.debug5h - Core.Landing.debug3h)));
                else gain = Core.Landing.debug4h + Core.Landing.debug6h + Core.Landing.debug8h * Mathf.Clamp01((float)((herror_ - Core.Landing.debug5h) / (Core.Landing.debug7h - Core.Landing.debug5h)));

                //Debug.Log("pidh accum:" + velaccum_h.magnitude.ToString("F4") + " gain:" + gain.ToString("F4"));

                Vector3d rc = gain*(PID_KP * ((PID_KI * velaccum_h) + herror)) / VesselState.maxThrustAccel;
                double rcM = Math.Min(_limitedMaxThrustAccel, rc.magnitude);
                return rcM * rc.normalized;
            }

            public double pidVelocity_h(double herror)
            {
                const double PID_ACCUM_MAX = 60;// 50; // 50<=30 <= 20
                // Increase PI Gains in higher gravity worlds - to track desired vertical speed
                double PID_KP = hFrac*Core.Landing.debug1h * Math.Max(20, Math.Min(50, 20 + (50 - 20) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 50(highg);// 40; // 40<=30 <= 20
                double PID_KI = hFrac*Core.Landing.debug2h * Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 0.05(lowg) 0.175(highg);// 0.35;// 0.25;// 0.15;// 0.2;// 0.1;
                double _limitedMaxThrustAccel = actualLimitedMaxThrustAccel;

                if (_limitedMaxThrustAccel < VesselState.limitedMaxThrustAccel)
                {
                    _limitedMaxThrustAccel = actualLimitedMaxThrustAccel + (VesselState.limitedMaxThrustAccel - actualLimitedMaxThrustAccel)
                        * (Core.Landing.g - 0.05) / (23 - 0.05);
                }

                herror = herror / _limitedMaxThrustAccel;
                double verrorAbs = Math.Abs(herror);
                velaccum_h2 += herror;
                velaccum_h2 = Math.Max(-PID_ACCUM_MAX, Math.Min(PID_ACCUM_MAX, velaccum_h2));
                if (Math.Abs(herror) > Core.Landing.debug92) velaccum_h2 = 0;
                double gain;
                if (verrorAbs < Core.Landing.debug3h) gain = Core.Landing.debug4h;
                else if (verrorAbs < Core.Landing.debug5h) gain = Core.Landing.debug4h + Core.Landing.debug6h * Mathf.Clamp01((float)((verrorAbs - Core.Landing.debug3h) / (Core.Landing.debug5h - Core.Landing.debug3h)));
                else gain = Core.Landing.debug4h + Core.Landing.debug6h + Core.Landing.debug8h * Mathf.Clamp01((float)((verrorAbs - Core.Landing.debug5h) / (Core.Landing.debug7h - Core.Landing.debug5h)));

                //Debug.Log("pidh accum:" + velaccum_h2.ToString("F4") + " gain:" + gain.ToString("F4"));

                return (Math.Max(-_limitedMaxThrustAccel, Math.Min(_limitedMaxThrustAccel, gain * (PID_KP * (PID_KI * velaccum_h2 + verrorAbs))) / VesselState.maxThrustAccel));
            }

            public double pidVelocity_v(double verror)
            {
                const double PID_ACCUM_MAX = 60;// 50; // 50<=30 <= 20
                // Increase PI Gains in higher gravity worlds - to track desired vertical speed
                double PID_KP = vFrac*Core.Landing.debug1 * Math.Max(20, Math.Min(50, 20 + (50 - 20) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 50(highg);// 40; // 40<=30 <= 20
                double PID_KI = vFrac*Core.Landing.debug2 * Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 0.05(lowg) 0.175(highg);// 0.35;// 0.25;// 0.15;// 0.2;// 0.1;
                double _limitedMaxThrustAccel = actualLimitedMaxThrustAccel;

                if (_limitedMaxThrustAccel < VesselState.limitedMaxThrustAccel)
                {
                    _limitedMaxThrustAccel = actualLimitedMaxThrustAccel + (VesselState.limitedMaxThrustAccel - actualLimitedMaxThrustAccel)
                        * (Core.Landing.g - 0.05) / (23 - 0.05);
                }
                double gComp = VesselState.localg;
                
                verror = verror/_limitedMaxThrustAccel;
                double verrorAbs = Math.Abs(verror);
                velaccum_v += verror;
                velaccum_v = Math.Max(-PID_ACCUM_MAX, Math.Min(PID_ACCUM_MAX, velaccum_v));
                if (Math.Abs(verror) > Core.Landing.debug91) velaccum_v = 0;
                double gain;
                if (verrorAbs < Core.Landing.debug3)      gain = Core.Landing.debug4;
                else if (verrorAbs < Core.Landing.debug5) gain = Core.Landing.debug4                     + Core.Landing.debug6*Mathf.Clamp01((float)((verrorAbs - Core.Landing.debug3)/(Core.Landing.debug5-Core.Landing.debug3)));
                else                                      gain = Core.Landing.debug4+Core.Landing.debug6 + Core.Landing.debug8*Mathf.Clamp01((float)((verrorAbs - Core.Landing.debug5)/(Core.Landing.debug7-Core.Landing.debug5)));

                //Debug.Log("pidv accum:" + velaccum_v.ToString("F4") + " gain:" + gain.ToString("F4"));

                return (Math.Max(-_limitedMaxThrustAccel, Math.Min(_limitedMaxThrustAccel, gComp + gain * (PID_KP * (PID_KI * velaccum_v + verror))) / VesselState.maxThrustAccel));
            }

            // Build the vertical and horizontal thrust vectors independently, then put it together.
            public void setThrustVector(ref double hTargetError, ref Vector3d desiredThrustVector, ref Vector3d desiredHorizontalVel, Vector3d hError, double hCorrectionAngle, ref double desiredVerticalSpeed, double vError)
            {
                if (hTargetError < hFarLimit)
                {
                    hFrac = vFrac = 1;
                }
                else
                {
                    hFrac = vFrac = Core.Landing.debug94;
                }
                bool     upright = false;
                double   vStepAngle = 0.02;
                //double   hStepAngle = Core.Landing.debug95;
                double   vMinAngle = 0;
                double   vMaxAngle = 1;
                double   vZeroThrustAngle = 0;
                double   verticalThrust = pidVelocity_v(vError);
                double   verticalThrust_;
                double   hsign = (Vector3.Angle(desiredHorizontalVel,hError) <= 90) ? 1.0 : -1.0;
                double   horizontalThrust = pidVelocity_h(hsign*hError.magnitude);
                double   horizontalThrust_ = horizontalThrust;
                //horizontalSign = System.Math.Min(Math.Abs(horizontalThrust), System.Math.Max(-Math.Abs(horizontalThrust), horizontalSign + hStepAngle * horizontalThrust));

                Vector3d horizontal = hCorrectionAngle * hError;

                // High Gravity
                if (Core.Landing.g > Core.Landing.LOW_GRAVITY)
                {
                    upright = true;
                    throttleAngle = 30;
                    if ((desiredVerticalSpeed > 10) && (VesselState.altitudeTrue >= 100))
                    {
                        upright = false;
                        //                        horizontal = horizontalSign*Math.Abs(horizontalThrust_)*hError.normalized;
//                        horizontal = (horizontalSign * desiredHorizontalVel).normalized + Core.Landing.debug96 * Math.Abs(horizontalThrust_) * hError.normalized;
                        horizontal = desiredHorizontalVel.normalized + Core.Landing.debug96 * Math.Abs(horizontalThrust_) * hError.normalized;
                        vMinAngle = -0.2*Math.Abs(verticalThrust);
                        vMaxAngle = +Math.Abs(verticalThrust);
                    }
                    else if (VesselState.altitudeTrue < 1000)
                    {
                        vMinAngle = 0.05;
                        vMaxAngle = 0.15; // 0.15 <= 2
                        vZeroThrustAngle = 1.3;
                    }
                    else
                    {
                        vMinAngle = 0.0;
                        vMaxAngle = 0.2;
                        vZeroThrustAngle = 0;
                    }
                }

                // Low Gravity
                else
                {
                    if (VesselState.altitudeTrue < 25)
                    {
                        upright = true;
                        vMinAngle = 0.0;
                        vMaxAngle = 0.2;
                        vZeroThrustAngle = 1.0;
                        throttleAngle = 5;
                    }
                    else
                    {
                        throttleAngle = 30;
                        vMinAngle = -Math.Abs(verticalThrust);
                        vMaxAngle = +Math.Abs(verticalThrust);
                    }
                }


                // Both High Gravity and Low Gravity use this at lower altitudes
                Vector3d vertical;
                if (upright == true)
                {
                    verticalThrust_ = System.Math.Max(0, verticalThrust);
                    if (verticalThrust_ == 0)
                    {
                        verticalSign = vZeroThrustAngle * horizontal.magnitude;
                    }
                    else
                    {
                        verticalSign = System.Math.Min(vMaxAngle, System.Math.Max(vMinAngle, verticalSign + vStepAngle * Math.Abs(verticalThrust)));
                        double horizontal_ = Core.Landing.debug93*Math.Min(hCorrectionAngle*verticalSign, horizontal.magnitude);
                        horizontal *= horizontal_ / horizontal.magnitude;
                        horizontalThrust_ *= horizontal_ / horizontal.magnitude;
                    }

                    if ((desiredVerticalSpeed > 0.01) && (VesselState.altitudeTrue < 100))
                    {
                        horizontal = Vector3d.zero;
                        horizontalThrust_ = 0;
                    }
                    vertical = verticalSign * Vessel.up;

                    //Debug.Log("1-sign:" + verticalSign.ToString("F4") + "," + horizontalSign.ToString("F4") +
                    //    " err:" + hError.magnitude.ToString("F4") + "," + vError.ToString("F4") + horizontalThrust_.ToString("F4") + " " + horizontalThrust.ToString("F4") + " " + " " + verticalThrust_.ToString("F4") + " " + verticalThrust.ToString("F4") +
                    //    " hcorr:" + hCorrectionAngle.ToString("F4"));
                }

                // Only Low Gravity uses this at high altitude
                else
                {
                   // verticalSign = System.Math.Min(vMaxAngle, System.Math.Max(vMinAngle, verticalSign + vStepAngle * verticalThrust));
                    verticalSign = System.Math.Min(vMaxAngle, System.Math.Max(vMinAngle, System.Math.Sign(verticalThrust) + Core.Landing.debug95 * vError));
                    vertical = System.Math.Abs(verticalThrust) *Vessel.up*verticalSign;

                    // do not allow vertical thrust to be opposite of the sign - this causes instability in low gravity when trying to maintain altitude.
                    verticalThrust_ = (verticalSign * verticalThrust < 0) ? 0 : verticalThrust;

                    //Debug.Log("2-sign:" + verticalSign.ToString("F4") + "," + horizontalSign.ToString("F4") +
                    //    " err:" + hError.magnitude.ToString("F4") + "," + vError.ToString("F4") + horizontalThrust_.ToString("F4") + " " + horizontalThrust.ToString("F4") + " " + " " + verticalThrust_.ToString("F4") + " " + verticalThrust.ToString("F4") +
                    //    " hcorr:" + hCorrectionAngle.ToString("F4"));
                }

                // Set Vertical Vector
                //Vector3d vertical = verticalSign * Vessel.up;

                // Set desired thrust vector based on horizontal and vertical errors.
                desiredThrustVector = horizontal + vertical;

                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;
                attitudeAngleFromThrust = Core.Attitude.attitudeAngleFromTarget();
                // float adjThrust = Mathf.Clamp01(1f - (float)attitudeAngleFromThrust/60f);
                float adjThrust = Mathf.Clamp01(1.75f - (float)attitudeAngleFromThrust*0.05f); // y = 1.75 - 0.05x
                Core.Thrust.RequestActiveThrottle(adjThrust*(float)Math.Sqrt(Math.Pow(verticalThrust_, 2) + Math.Pow(horizontalThrust_, 2)));
            }

            // move 
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
            public void move(ref double desiredVerticalSpeed, ref Vector3d desiredThrustVector)
            {
                // Adjust a negative desiredVerticalSpeed for Low Gravity worlds to avoid limiting it to a very small number when far away vertically.
                if ((Core.Landing.g < Core.Landing.LOW_GRAVITY) && (Core.Landing.increaseVertical == false) && desiredVerticalSpeed < 0 && VesselState.altitudeTrue >= 25)
                {
                    desiredVerticalSpeed = Math.Min(desiredVerticalSpeed, -175.0*Mathf.Clamp01((float)(VesselState.altitudeTrue-25.0)/1500.0f));
                }

                // Get the Horizontal distance to target. This will be used to calculate the horizontal speed.
                double hTargetError = Core.Landing.targetingResult.hTargetError;

                // Get Horizontal direction vector towards the target. This will be combined later with
                // the vertical vector to get the thrust direction.
                Vector3d courseCorrection = Core.Landing.targetingResult.horizontalVec.normalized;

                // Using the current horizontal velocity and desired horizontal velocity calculate the desired
                // horizontal thrust vector.  An velocity herror vector will be calculated.
                // A speed up would be an herror vector in the direction of target
                // A speed down would be an herror vector opposite in the direction of target.
                float ratio = (float)(hTargetError / VesselState.altitudeTrue);
                float maxverror = 0.1f; // hysterisis in the negative direction.
                float minverror = -3f;  // hysterisis in the positive direction.

                // Set the desired vertical speed - it can be overriden to achieve suborbital flight
                setVerticalSpeed(ref desiredVerticalSpeed, ref hTargetError, ref ratio, ref minverror, ref maxverror);
                double vError = desiredVerticalSpeed - VesselState.speedVertical;

                // Set the desired Horizontal Velocity to reach target. Error to subtract the vessels current horizontal velocity.
                double hCorrectionAngle = setHorizontalSpeed(out Vector3d desiredHorizontalVel, out Vector3d herror, ref hTargetError, ref ratio, ref courseCorrection);

                // Set horizontal vector based on positive vertical vector: can be suborbital or local vertical increase
                if (Core.Landing.increaseVertical == true)
                {
                    setSubOrbitalHorizontal(ref desiredHorizontalVel, ref herror);
                }

                // Set thrust vector based on vertical and horizontal inputs
                setThrustVector(ref hTargetError, ref desiredThrustVector, ref desiredHorizontalVel, herror, hCorrectionAngle, ref desiredVerticalSpeed, vError);

                // This is a long distance target requiring sub orbital targeting
                if ((hCurrentError > 0) && (errStep > Ascend2Target.Disabled) && (errStep < Ascend2Target.Completed))
                {
                    TargetSubOrbital(ref desiredThrustVector);
                }

                // STOP Increase Vertical
                if (stopIncrease == true)
                {
                    stopIncrease = false;
                    Core.Landing.increaseVertical = false; // max ratio is less than max ratio  - no need to increase altitude.
                    Debug.Log("stopIncrease: " + hCurrentError.ToString("F1") + ", " + ratio.ToString("F2"));

                    // If in range of suborbital target then do retro burn.
                    if (((Core.Landing.g > Core.Landing.LOW_GRAVITY) && (hTargetError > Core.Landing.POST_TARGET_THRESHOLD))
                        || (VesselState.orbitPeA >= 9000) )
                    {
                        trace += 'f';
                        errStep = Ascend2Target.FineTune;
                    }

                    // Otherwise use move to target to get there.
                    else
                    {
                        checkWarp = false;    // Exiting ascent phase at low altitude or due to large periapsis - just move to target
                        trace += 'm';          
                        suppressThrottle = 2;                   // make sure throttle is NOT disabled
                        errStep = Ascend2Target.Completed;      // Set ascent step to completed so it does not start it again.
                        decStep = DecelerateSteps.DisabledDec;  // Disable deceleration steps as this is not the normal transition
                    }
                }

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status17", // Moving to target: speed(V:<<1>> , H:<<2>>) m/s
                    desiredVerticalSpeed.ToString("F1"), //trace);
                    desiredHorizontalVel.magnitude.ToString("F1"));
                Status += "\n" + trace;
            }

            public void setVerticalSpeed(ref double desiredVerticalSpeed, ref double hTargetError, ref float ratio, ref float minverror, ref float maxverror)
            {
                // Activate Emergency vertical speed increase if far from target and altitude is less than 100 meters and far away.
                // The reasoning is that the algorithm should push for a small ratio to begin with so something
                // went wrong.
                emergency = false;
                if ((ratio > maxRatio) && (VesselState.altitudeTrue < EMERGENCY_ALTITUDE) && (hTargetError > EMERGENCY_H_DISTANCE))
                {
                    Core.Landing.increaseVertical = true; // Emergency - avoid terrain collision
                    emergency = true;
                }

                // Increase Vertical so that maxRatio is reached - this can be due to a hop or emergency crash avoidance
                if (Core.Landing.increaseVertical == true)
                {
                    if (ratio > maxRatio)
                    {
                        double vFrac = (MainBody.atmosphere == true )? 1.0:0.2;
                        desiredVerticalSpeed = Math.Min(vFrac * Core.Landing.GetCircularOrbitSpeed(VesselState.altitudeTrue, MainBody), Math.Max(2.0, (ratio - maxRatio) * (1 + hTargetError / ASCENT_H_DISTANCE_DIVISOR)));
                        if ((Math.Abs(VesselState.orbitApA - Vessel.terrainAltitude) > Math.Min(1.2 * hTargetError, subOrbitalApA)) && (VesselState.altitudeTrue > ASCENT_AOA_MIN_ALT))
                        {
                            desiredVerticalSpeed = 0;
                        }
                        ignoreLandingCounter = 0; // reset ignore landing count to allow start from a landing
                    }
                    else
                    {
                        desiredVerticalSpeed = 2.0; // give it a little boost to get going and avoid stalling at low altitude with low vertical speed.
                        if (VesselState.altitudeTrue > 10) stopIncrease = true; // Trigger STOP vertical increase
                    }
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
                double _hCorrectionUpAngle;
                double maxHThrust = Math.Min(LOW_GRAVITY_THRUST_MAX * Core.Landing.g, limitedMaxThrustAccel);
                double gainCancelHVelocity = (hTargetError <= hCloseLimit) ? baseGain : baseGain / 13.0; // gilly=4(hTargetError < 500)

                // Scale divisor based on local gravity and cap the values.
                // 2 <= 1.0 (slight flipping) <= 0.5 (lots of flipping) <= 2.0  ? Divisor is bigger for larger gravity - not sure this is right
                divisor = Math.Max(1.0, Math.Min(800, divisor * (0.4 * 20.408) * Core.Landing.g));

                // Controls Max angle to move horizontally.
                if (hTargetError < 1.25)
                {
                    _hCorrectionUpAngle = 0.025 * 50.4 ; //  (baseline)0.025 * 36<=0.025 * 18; //  0.025 * 9; // 0.025*3;
                }
                else if (hTargetError < hCloseLimit)
                {
                    _hCorrectionUpAngle = 0.025 * 25.2; //  (baseline)0.025 * 36<=0.025 * 18; //  0.025 * 9; // 0.025*3;
                }
                else if (hTargetError < hFarLimit)
                {
                    _hCorrectionUpAngle = hMidCorrectionAngle; // mid horizontal distance correction angle
                }
                else
                {
                    _hCorrectionUpAngle = hFarCorrectionAngle; // far horizontal distance correction angle
                }


                // If horizontal herror is really large set desired horizontal velocity based on reaching target
                // If target within reach then avoid changing the horizontal velocity.
                Vector3d hSurfaceVelocity = Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                if ((Core.Landing.g > Core.Landing.LOW_GRAVITY) && (VesselState.altitudeTrue > H_SUBORBITAL_ALTITUDE_THRESHOLD_CONSTANT) && 
                    (hTargetError > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT) && (ratio > 0.5*maxRatio))
                {
                    // Set horizontal velocity to reach predicted target
                    // Use max expected velocity and scale it based on  
                    double maxHorizontalSpeed;

                    hCurrentError = Core.Landing.targetingResult.distanceToTarget;
                    if (errStep == Ascend2Target.Disabled)
                    {
                        maxHorizontalSpeed = Math.Min(hCurrentError * Core.Landing.g / 70.0, Math.Sqrt(hTargetError * 2 * maxHThrust));
                        errStep++; trace += ((int)errStep).ToString(); // Will execute fine tune to far target 
                    }
                    else if (errStep == Ascend2Target.InitialBurn)
                    {
                        maxHorizontalSpeed = Core.Landing.GetCircularOrbitSpeed(VesselState.altitudeTrue, MainBody);

                        if (hCurrentError < 0.1 * hTargetError)
                        {
                            if (_subOrbitalApA == 0)
                            {
                                _subOrbitalApA = VesselState.orbitApA;
                            }
                        }
                        else
                        {
                            _subOrbitalApA = 0;
                        }
                    }
                    else
                    {
                        maxHorizontalSpeed = Math.Min(hCurrentError * Core.Landing.g / 70.0, Math.Sqrt(hTargetError * 2 * maxHThrust));
                    }


                    if (Core.Landing.increaseVertical == true)
                    {
                        _hCorrectionUpAngle = (MainBody.atmosphere == true) ? 0.005 : 0.05;
                    }
                    else
                    {
                        // If ratio is large at high altitude going fast then check to see if we need to slow down
                        if ((ratio > maxRatio) && (VesselState.altitudeASL > 0.5 * subOrbitalApA) && (VesselState.speedSurface > Core.Landing.atmosSafeSpeed))
                        {
                            double pitchAngle = 30;//90.0 - Vector3d.Angle(VesselState.surfaceVelocity, VesselState.up);
                            double stoppingDistance = Math.Pow(VesselState.speedSurfaceHorizontal, 2) / (2 * VesselState.limitedMaxThrustAccel * Math.Abs(Math.Cos(pitchAngle * UtilMath.Deg2Rad)));
                            double rangeToTarget = Core.Landing.targetingResult.hTargetError;

                            if (stoppingDistance < rangeToTarget)
                            {
                                maxHorizontalSpeed = 0.3 * VesselState.surfaceVelocity.magnitude;
                            }
                            else if (maxHorizontalSpeed > VesselState.speedSurface)
                            {
                                maxHorizontalSpeed = hSurfaceVelocity.magnitude;
                            }
                        }
                    }

                    desiredHorizontalVel = maxHorizontalSpeed * courseCorrection.normalized;
                    herror = desiredHorizontalVel - hSurfaceVelocity;
                }

                // Use default method to calculate horizontal velocity
                else
                {
                    prevError = hCurrentError = 0; // Dont allow fine tune as not needed

                    // When the target is really close then perform then amplify the horizontal error to aggressively eliminate horizontal motion.
                    if (hTargetError < 0.80)
                    {
                        desiredHorizontalVel = (Math.Sqrt((1 + Core.Landing.g) * hTargetError * maxHThrust / divisor) * 2.0 / 1.5) * courseCorrection.normalized;

                        herror = 0.25 * desiredHorizontalVel - hSurfaceVelocity;
                    }

                    // Use the correction vector and a calculated max horizontal speed. Clamp the max to the orbital speed at the current altitude to avoid overshooting the target.
                    else
                    {
                        double maxHorizontalSpeed = Math.Min((Math.Sqrt((1 + Core.Landing.g) * hTargetError * maxHThrust / divisor) * 2.0 / 1.5), 
                                                              Math.Sqrt(hTargetError * 2* maxHThrust));
                        if (Core.Landing.g > Core.Landing.LOW_GRAVITY)
                        {
                            maxHorizontalSpeed = System.Math.Min(Core.Landing.GetCircularOrbitSpeed(VesselState.altitudeASL, MainBody), maxHorizontalSpeed);
                        }

                        // If ratio is large at high altitude going fast then check to see if we need to slow down
                        if ((ratio > maxRatio) && (VesselState.altitudeASL > 0.5 * subOrbitalApA) && (VesselState.speedSurface > Core.Landing.atmosSafeSpeed))
                        {
                            double pitchAngle = 90.0 - Vector3d.Angle(VesselState.surfaceVelocity, VesselState.up);
                            double stoppingDistance = System.Math.Pow(VesselState.speedSurfaceHorizontal, 2) / (2 * VesselState.limitedMaxThrustAccel * Math.Abs(Math.Cos(pitchAngle * UtilMath.Deg2Rad)));
                            double rangeToTarget = Core.Landing.targetingResult.hTargetError;

                            // If stopping distance is less than range to target then make sure thrust is maxed out reducing
                            // the max allowed thrust, otherwise allow to operate normally.
                            if (stoppingDistance < rangeToTarget)
                            {
                                maxHorizontalSpeed = 0.3*VesselState.surfaceVelocity.magnitude;
                            }
                        }

                        desiredHorizontalVel = maxHorizontalSpeed * courseCorrection.normalized;
                        herror = desiredHorizontalVel - hSurfaceVelocity;
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
                return Math.Max(0, Math.Min(_hCorrectionUpAngle, baseGain * H_CORRECTION_ANGLE_CONSTANT * herror.magnitude ));
            }

            public void setSubOrbitalHorizontal(ref Vector3d desiredHorizontalVel, ref Vector3d herror)
            {
                Vector3d hSurfaceVelocity = Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);

                // If vertical increase is enabled then disable horizontal angle at small altitude.
                if (VesselState.altitudeTrue < 35.0 || emergency == true) 
                {
                    herror *= 0.1;
                    desiredHorizontalVel = herror + hSurfaceVelocity;
                }

                else if (((errStep == Ascend2Target.Disabled) || (hCurrentError > 0)))
                {
                    // Eve      35k alt   atmos density = 125 g/m^3   drag = 30 m/s^2   - FAIL
                    // kerbin   35k alt   atmos density = 2.2 g/m^3   drag = 1.3 m/s^2  - PASS
                    // kerbin 17.5k alt   atmos density =  66 g/m^3 - start of tilt.
                    // kerbin    0k alt   atmos density = 1.2 kg/m^3
                    // for atmospheric planets we taper starting at a specific altitude and increase tilt at higher altitudes
                    // Also qualify it with a minimum vertical velocity before we start tilting.
                    if (MainBody.atmosphere == true)
                    {
                        double orig_hvel = desiredHorizontalVel.magnitude - 2.0;
                        desiredHorizontalVel = desiredHorizontalVel.normalized * 2.0; // 2 ft/sec is the minimum horizontal speed moving towards target.

                        if (VesselState.atmosphericDensityGrams < 76.0)  // 76 <= 66
                        {
                            if (tiltDelta < 0.1) tiltDelta = MainBody.atmosphereDepth - VesselState.altitudeASL;
                            else if (VesselState.speedVertical > Core.Landing.g * 2 * 28)   // 28<=30<=20(too shallow)<=30
                            {
                                if (startTiltAlt < 0.1) startTiltAlt = VesselState.altitudeASL;
                            }

                            if ((startTiltAlt >= 0.1) && (VesselState.altitudeASL > startTiltAlt))
                            {
                                desiredHorizontalVel = desiredHorizontalVel.normalized * (2.0 + orig_hvel * Mathf.Clamp01((float)((VesselState.altitudeASL - startTiltAlt) / tiltDelta)));
                            } // else - dont add any tilt
                        }
                    }
                    else
                    {
                        double orig_hvel = desiredHorizontalVel.magnitude;
                        desiredHorizontalVel = 0.2*desiredHorizontalVel; // 20% is the minimum horizontal speed moving towards target in space - allows to get out of vertical limit cycle and move towards target.

                        if (tiltDelta < 0.1) tiltDelta = Math.Max(500, Math.Abs(Core.Landing.TargetAltitude - VesselState.altitudeASL));
                        else if (VesselState.speedVertical > Core.Landing.g * 2 * 28)   // 28<=30<=20(too shallow)<=30
                        {
                            if (startTiltAlt < 0.1) startTiltAlt = VesselState.altitudeASL;
                        }

                        if ((startTiltAlt >= 0.1) && (VesselState.altitudeASL > startTiltAlt))
                        {
                            desiredHorizontalVel = orig_hvel*desiredHorizontalVel.normalized * (0.2 + 0.8*Mathf.Clamp01((float)((VesselState.altitudeASL - startTiltAlt) / tiltDelta)));
                        } // else - dont add any tilt
                    }

                    herror = desiredHorizontalVel - hSurfaceVelocity;
                } // else - no atmosphere - leave horizontal vector unmodified.
            }

            public void TargetSubOrbital(ref Vector3d desiredThrustVector)
            {
                double _hCurrentError = hCurrentError;
                double decelerationStartTime = VesselState.orbitTimeToAp - 10;

                if (skipCounter > 0 ) skipCounter--;
                else
                {
                    skipCounter = 5* (int)Core.Landing.debug10;
                    if ((ignoreCount > 0) || (suppressThrottle == 0)) ignoreCount--;
                    else if (prevError > 0)
                    {
                        if (decreaseCount > 0)
                        {
                            if (_hCurrentError < prevError)
                            {
                                decreaseCount -= 2 * (int)Core.Landing.debug11;
                                if (decreaseCount <= 0) trace += 'd';
                            }
                            else decreaseCount = Math.Min(DECREASE_ERR_INTEGRATE_COUNT* (int)Core.Landing.debug12, decreaseCount + 1);
                        }
                        else if (increaseCount <= 0)
                        {
                            // If the initial burn was not successful then wait till reach new Apoapsis
                            // if not already reached it
                            if ((errStep == Ascend2Target.InitialBurn) && (_hCurrentError > 1050))
                            {
                                //apaFactor = 0.9;
                                if ((decelerationStartTime > 0) && (VesselState.speedVertical > 0))
                                {
                                    suppressThrottle = 1; // suppress throttle
                                    trace += 'S';
                                }
                            }

                            // go to next step and reset all counters.
                            errStep++; trace += ((int)errStep).ToString();
                            ignoreCount = IGNORE_ERR_INTEGRATE_COUNT * (int)Core.Landing.debug13;
                            increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                        }
                        else if (_hCurrentError > prevError)
                        {
                            increaseCount -= 2 * (int)Core.Landing.debug11;
                            if (increaseCount <= 0) trace += 'u';
                        }
                        else increaseCount = Math.Min(INCREASE_ERR_INTEGRATE_COUNT * (int)Core.Landing.debug14, increaseCount + 1);

                    }
                    prevError = _hCurrentError;
                }

                bool overrideAttitude = false;

                // Use correction attitude if apoapsis is high enough.
                // Determine when we can stop burning when target apoapsis has been reached
                // Once stopped a secondary burn will be performed to reach target
                if (suppressThrottle == 0)
                {
                    double checkApa = (_subOrbitalApA > 0) ? _subOrbitalApA : subOrbitalApA;
                    if (Math.Abs(VesselState.orbitApA) >= checkApa)
                    {
                        Debug.Log("checkApa: " + checkApa.ToString("F1"));
                        overrideAttitude = true; // with atmosphere
                        if ((decelerationStartTime > 0) && (VesselState.speedVertical > 0))
                        {
                            suppressThrottle = 1; // suppress throttle during initial burn
                            trace += 'S';
                        }
                    }
                }

                // While throttle is disabled monitor it until the second burn can start
                if (suppressThrottle == 1)
                {
                    // Hold on until we get to point where we can burn to target.
                    if ((decelerationStartTime > 0) && (VesselState.speedVertical > 0))
                    {
                        if (Core.Node.Autowarp && (decelerationStartTime > 0))
                        {
                            Core.Warp.WarpToUT(decelerationStartTime + VesselState.time);
                        }
                        else
                        {
                            Core.Warp.MinimumWarp();
                        }
                    }
                    else
                    {
                        suppressThrottle = 2; // latch it off - throttle will no longer be suppressed and second burn can begin
                        trace += 'U';
                        Core.Warp.MinimumWarp();
                        errStep++; trace += ((int)errStep).ToString();
                    }
                    increaseCount = INCREASE_ERR_INTEGRATE_COUNT;
                }

                // For initial burn the first burn will not override attitude.
                if (errStep == Ascend2Target.InitialBurn)
                {
                    if (suppressThrottle >= 2)
                    {
                        overrideAttitude = true; // only after throttle suppression ends - start 2nd burn cycle
                    }
                }

                //If attitude override enabled then set it
                if (overrideAttitude == true)
                {
                    if (suppressThrottle == 1)
                    {
                        // while suppressing throttle or setting target use surface velocity direction
                        desiredThrustVector = VesselState.surfaceVelocity.normalized;
                    }
                    else
                    {
                        // for second burn adjust override_ratio to stay behind the apoapsis
                        if (VesselState.orbitTimeToAp < 30)
                        {
                            override_ratio = 0.55;
                        }
                    }
                    desiredThrustVector = override_ratio * desiredThrustVector.normalized +
                        (1 - override_ratio) * Core.Landing.ComputeCourseCorrection().normalized;
                }
                else
                {
                    override_ratio = 0.75;
                }
            }
            
            // If there is any reasone why we should not allow disabling thrust due to attitude divergence
            // do it here.
            public bool allowDisableThrust(double desiredSpeed)
            {
                bool allow = true;

                // If in atmosphere with negative velocity with a large positive error then dont allow
                // thrust to be disabled.
                // The reason is vehicles with insufficient attitude control cannot overcome atmospheric forces
                // exerted on the vehicle preventing it from reach its target attitude - this has been
                // observed causing the vehicle to crash on the surface even though the thrusters were pointing straight down.
                // it would fire them at lower altitudes but vehicles with lower available thrust cannot regain vertical speed
                // control before crashing.
                if (Core.Landing.g > Core.Landing.LOW_GRAVITY)
                {
                    if (VesselState.speedVertical < 0 && (decStep != DecelerateSteps.IHorizontalDec))
                    {
                        if ((desiredSpeed - VesselState.speedVertical) > 3) allow = false;
                    }
                }

                return allow;
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
                    Core.Thrust.ThrustOff();
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
                if (hCurrentError >= 10 || Core.Landing.increaseVertical == true)
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
                double desiredSpeed;
                try
                {
                    desiredSpeed = GetMaxSpeed();
                }
                catch (Exception ex)
                {
                    // Handle the error gracefully, for example by logging it and setting a default desired speed
                    Debug.Log("Error: " + ex.Message);
                    desiredSpeed = Core.Landing.MaxAllowedSpeed();
                }

                Vector3d desiredThrustVector = -VesselState.surfaceVelocity.normalized;

                // Move to target strategy
                move(ref desiredSpeed, ref desiredThrustVector);
                if ( (errStep >= Ascend2Target.FineTune) && (errStep < Ascend2Target.Completed))
                {
                    Core.Thrust.ThrustOff();
                    if (Core.Landing.LandingType == 1)
                    {
                        return new TargetSubOrbit(Core);
                    }
                    else
                    {
                        return new CourseCorrection(Core);
                    }
                }

                Quaternion targetRot = QuaternionD.LookRotation(desiredThrustVector.normalized, -Vessel.GetTransform().forward);
                Core.Attitude.attitudeTo(targetRot, AttitudeReference.INERTIAL, this);

                lastDesiredThrustVector = desiredThrustVector;

                // Set Debug Vectors as horizontal and vertical components of desired thrust vector
                MechJebModuleDebugArrows.debugVector = Vector3d.Dot(lastDesiredThrustVector, Vessel.upAxis) * Vessel.upAxis;
                MechJebModuleDebugArrows.debugVector2 = lastDesiredThrustVector - MechJebModuleDebugArrows.debugVector;
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
                if ((suppressThrottle == 1) || (checkWarp == true)) 
                {
                    Core.Thrust.ThrustOff();
                }
                else
                {
                    if ( (attitudeAngleFromThrust > throttleAngle) && (allowDisableThrust(desiredSpeed) == true) )
                    {
                        Core.Thrust.RequestActiveThrottle(0);
                    }
                }

                if (double.IsNaN(desiredSpeed)) return new MoveToTarget2(Core);
                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                Core.Landing.targetingResult.update(); // Update used in OnFixedUpdate and Drive so that it is updated at the right time for both.

                double pitchAngle = 90.0 - Vector3d.Angle(VesselState.surfaceVelocity, VesselState.up);
                double hStoppingDistance = Math.Pow(VesselState.speedSurfaceHorizontal, 2) / (2 * VesselState.limitedMaxThrustAccel*Math.Abs(Math.Cos(pitchAngle * UtilMath.Deg2Rad)));
                double vStoppingDistance = (VesselState.speedVertical<0) ? Math.Pow(VesselState.speedVertical, 2) / (2 * (VesselState.limitedMaxThrustAccel * Math.Abs(Math.Sin(pitchAngle * UtilMath.Deg2Rad)) - VesselState.localg) ):0;
                double hTargetError = Core.Landing.targetingResult.hTargetError;
                float ratio = (float)(hTargetError / VesselState.altitudeTrue);

                if ((hTargetError<(1.5*hStoppingDistance) ) ||
                    (VesselState.altitudeTrue < (1.5 * vStoppingDistance)) ||
                    (ratio < 1.5*Core.Landing.maxRatio) || 
                    ((VesselState.speedVertical < 0) && ((VesselState.altitudeTrue < ALT_MIN_WARP_CONSTANT) ||
                    ((VesselState.altitudeASL < MainBody.RealMaxAtmosphereAltitude()) && (VesselState.speedSurface > Core.Landing.atmosSafeSpeed)))) )
                {
                    checkWarp = false; // Can't warp under these circumstances
                }

                //Warp at a rate no higher than the rate that would have us impacting the ground 10 seconds from now:
                if (checkWarp && Core.Node.Autowarp)
                {
                    // Make sure if we're hovering that we don't go straight into too fast of a warp
                    // (g * 5 is average velocity falling for 10 seconds from a hover)
                    double velocityGuess = Math.Max(Math.Abs(VesselState.speedVertical), VesselState.localg * 5);
                    Core.Warp.WarpRegularAtRate((float)Math.Min(VesselState.altitudeASL / (6 * velocityGuess), (Orbit.period / 6)));
                    warpOn = true;
                }
                else if ((warpOn == true) /*|| !MuUtils.PhysicsRunning()*/)
                {
                    Core.Warp.MinimumWarp();
                    warpOn = false;
                }

                return this;
            }
        }
    }
}
