using System;
using KSP.Localization;
using UnityEngine;


namespace MuMech
{
    namespace Landing
    {
        public class MoveToTarget : AutopilotStep
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
            private const float SAFE_ALT_SPEED_SLOW_CONSTANT = 23000;//13000;//12000;//15000(works-short);
            private const float ALT_SPEED_SLOW_CONSTANT = 18000;//17000;//17000;//13000;//12000;//15000(works-short);  Under this Altitude scale down the vertical speed to landing. 
            private const float ALT_MIN_WARP_CONSTANT = 19000; // 19000 <= 50000 Do not warp below this altitude
            private const float H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT = 10000;  // When above this horizontal distance to target and below the altitude threshold, use suborbital targetting to approach target, otherwise use normal targetting. This allows more aggressive approach when far from target and more precise control when close to target.
            private const float H_CORRECTION_ANGLE_CONSTANT = 0.4f; // Used to determine horizontal correction angle 
            private const float LANDING_GEAR_ALT_CONSTANT = 200.0F; // Drop landing gear below this altitude
            private const float SAFE_FINAL_SPEED_FACTOR_CONSTANT = 0.8F; // To avoid crashing in terrain in final descent
            private const double OVERSHOOT_ANGLE_FRACTION = 0.9; // Small margin to avoid losing vertical control
            public const double LIMITED_MAX_THRUST_G_RATIO = 5.0; // 10<=5.0 <=4 <= 3.125 this is multiplied with Mainbody g
            private const double LIMITED_SLOW_THRUST_G_RATIO = 1.25; // this is multiplied with Mainbody g
            private const double LOW_GRAVITY_THRUST_MAX = 10;// 10 <= (bad)100.0; // 10.0
            private const int LAND_STABILIZE_COUNT = 500; // controls how long to stabilize landing.
            private const int IGNORE_LANDING_COUNT = 100; // controls how long to ignore landing check.
            private const float EMERGENCY_ALTITUDE   = 400.0f;  // below this altitude activate emergency ascent
            private const float EMERGENCY_H_DISTANCE = 500.0f;  // beyond this distance activeate emergency ascent
            private const double ASCENT_H_DISTANCE_DIVISOR = 4000.0; // <=2000 used to calculate desired vertical speed
            private const double ASCENT_AOA_MIN_ALT = 2000.0;   // minimun altitude where desired vertical speed is set to zero.
            private bool stopIncrease = false;  // Set to true when need to stop vertical increase
            private bool checkWarp = true;      // Only enabled for retro burn to wait for right moment to burn
            private bool warpOn = false;        // Used to track if warp is on to end it at the right time 
            private int suppressThrottle;       // Used to control throttle suppression (=1) while performing sub orbital targeting
            private bool emergency;             // Used to trigger an emergency ascent to avoid crash 
            private int _deployedGears = 1;     // 0->1->2  - deploy gears twice   2->1->0 store gears twice
            private double hFarCorrectionAngle;             // far horizontal distance overshoot angle
            private double hMidCorrectionAngle;             // mid horizontal distance overshoot angle
            private double velaccum_h = 0;                 // Integral velocity error accumulator for Horizontal PI Controller
            private double velaccum_v = 0;                  // Integral velocity error accumulator for Vertical PI Controller
            private double limitedMaxThrustAccel = 0;       // Limited thrust acceleration used to calculate TWR and overshoot angle + scale max speed
            private double actualLimitedMaxThrustAccel = 0; // Calculate actual limited max thrust used at this time.
            private IDescentSpeedPolicy _aggressivePolicy;  // Used to calculate max speed when not flying safe - more aggressive
            private float altSpeedSlow;                     // Under this terrain altitude the vertical velocity will be scaled down a bit.
            private float altLandThreshold;                 // Under this terrain altitude the vertical velocity will be linearly scaled down to zero ( + clamp to landing speed )
            private float speedFactor = 1.0F;               // Used to store the Landing Margin Factor applied to target speed, slows down faster the larger the margin.
            private int landStabilizeCounter = 0;   // Used to Control how long to stabilize landing.
            private int ignoreLandingCounter = 0;   // Used to Control how long to ignore landing check
            private double verticalSign = 1;   // Used to control the sign of vertical thrust in low gravity - when upright verticalSign is positive, when pitched forward to speed up it is negative, when pitched back to slow down it is positive. This allows the PID controller to continue to operate even when pitched forward or backward by not allowing the vertical thrust to be negative when pitched forward and not allowing the vertical thrust to be positive when pitched back.
            private Vector3d horizontalAvg;
            private double hCurrentError = 0;
            private double startTiltAlt = 0;
            private double startTiltHVel = 0;
            private double tiltDelta = 0;
            private double subOrbitalApA; // When performing suborbital targetting this is the target ApA to aim for to approach the target - it is set based on the body and atmosphere to give good results.
            private double _subOrbitalApA = 0; // Set the Target Orbital ApA based on groundtrack result
            private double hCloseLimit = 25;   // 25 <= 50 <=(baseline)20
            private double hMidLimit = 700;    // 700 <= 1500 <=1000(baseline)<=500<=100
            private double hFarLimit = 5000;   // 5000 <= (baseline)10000
            private double divClose;
            private double divMid;
            private double divFar;
            private double baseGain; // Calibrated gain for horizontal angle and to correct movement that would diverge from target.

            // The steepness controls the ratio curvature - When horizontal distance to target is small we
            // dont want to modify the desired vertical velocity.
            // When horizontally far from target we want to maintain altitude, but slower allow vertical drop
            // as approach target.
            private float steepness;// Defines the steepness to final descent ratio curvature to target.
            private float maxRatio; // Desired Vertical speed is zero at this or larger ratios 
            private float minRatio; // Desired Vertical speed is unmodified at this or smaller ratios

            // Try with course correction.
            private Ascend2Target    errStep = Ascend2Target.Disabled;
            private double throttleAngle;
            private double attitudeAngleFromThrust;
            private string trace;
            private Vector3d lastDesiredThrustVector;
            private double hFrac = 1;
            private double vFrac = 1;
            private double desiredVerticalSpeed;
            private double previousDesiredVerticalSpeed = 0;
            private bool subOrbitalTileLatched = false;
            private bool hHysteresis = false; // Used to freeze the horizontal vector while hysteresis is active - may not be used when landing
            private Vector3d hHysteresisVector;
            private int vCollisionCounter = 0;
            private const int COLLISION_TAIL = 30*50;
            private int vIgnoreCollision = COLLISION_TAIL;
            private double vCollisionDeltaV = 0;

            public MoveToTarget(MechJebCore core) : base(core)
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
                    divFar = Core.Landing.debug14;// (MainBody.atmosphere == false) ? 1.10 : 1;  // noatmos:1.25  atmos:1 <= 2
                }
            }
            private double GetMaxSpeed()
            {
                double maxSpeed;

                // Set Acceleration Limit  
                actualLimitedMaxThrustAccel = limitedMaxThrustAccel;

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

            // Persistent state
            double v_prev;           // Previous measured velocity
            double dv_filt;          // Filtered derivative of velocity
            const double alpha = 0.75; // Filter coefficient (tune as needed)

            double ComputeDerivativeContribution(double v_meas, double dt, double K_d)
            {
                double raw_dv = (v_meas - v_prev) / dt;          // Derivative of PV
                dv_filt = alpha * dv_filt + (1 - alpha) * raw_dv; // Low-pass filter
                v_prev = v_meas;
                return -K_d * dv_filt;                             // Contribution to u
            }

            public double pidVelocity_h(ref Vector3d desiredHorizontalVel, ref Vector3d hError, ref double hCorrectionAngle)
            {
                double   hsign = (hError.magnitude==0) ? 0 : (Vector3.Angle(desiredHorizontalVel,hError) <= 90) ? 1.0 : -1.0;
                double   herror = hsign*hError.magnitude;
                const double PID_ACCUM_MAX = 500;// 60<=50<=30 <= 20
                // Increase PI Gains in higher gravity worlds - to track desired vertical speed
                double PID_KP = hFrac*Core.Landing.debug1h * Math.Max(15, Math.Min(50, 10 + (50 - 10) * (Core.Landing.g - 0.05) / (2 - 0.05)));// KP range [15(lowg)-50(highg)];// 40; // 40<=30 <= 20
                double PID_KI = hFrac*Core.Landing.debug2h * Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 0.05(lowg) 0.175(highg);// 0.35;// 0.25;// 0.15;// 0.2;// 0.1;
                double _limitedMaxThrustAccel = actualLimitedMaxThrustAccel;

                velaccum_h = Math.Max(-PID_ACCUM_MAX, Math.Min(PID_ACCUM_MAX, velaccum_h + herror*Time.fixedDeltaTime));
                if (Math.Abs(herror) > Core.Landing.debug92) velaccum_h = 0;
                double gain;

                herror = Math.Abs(herror);
                gain = Core.Landing.debug4h;
                if (herror < Core.Landing.debug3h)
                {
                    if (hHysteresis==false)
                    {
                        hHysteresis = true;
                        hHysteresisVector = hError;
                    }
                    hError = hHysteresisVector;
                }
                else
                {
                    hHysteresis = false;
                    if (herror < Core.Landing.debug5h)
                    {
                        gain += Core.Landing.debug6h * Mathf.Clamp01((float)((herror - Core.Landing.debug3h) / (Core.Landing.debug5h - Core.Landing.debug3h)));
                    }
                    else
                    {
                        gain += Core.Landing.debug6h + Core.Landing.debug8h * Mathf.Clamp01((float)((herror - Core.Landing.debug5h) / (Core.Landing.debug7h - Core.Landing.debug5h)));
                    }
                }

                double d = ComputeDerivativeContribution(Vessel.horizontalSrfSpeed, Time.fixedDeltaTime, Core.Landing.debug13);
                double t = Math.Max(-VesselState.limitedMaxThrustAccel, Math.Min(VesselState.limitedMaxThrustAccel,  gain * (PID_KP*herror + PID_KI*velaccum_h + d))) / VesselState.maxThrustAccel;
                Debug.Log("pidh  t:" + t.ToString("F4") + "  herror:" + (hsign*herror).ToString("F2") + "  accum:" + velaccum_h.ToString("F4") + "  gain:" + gain.ToString("F4") + "  d:" + d.ToString("F4"));
                return t;
            }

            public double pidVelocity_v(double verror)
            {
                const double PID_ACCUM_MAX = 500; // 60<=50<=30<=20
                // Increase PI Gains in higher gravity worlds - to track desired vertical speed
                double PID_KP = vFrac*Core.Landing.debug1 * Math.Max(20, Math.Min(50, 20 + (50 - 20) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 50(highg);// 40; // 40<=30 <= 20
                double PID_KI = vFrac*Core.Landing.debug2 * Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));// 0.05(lowg) 0.175(highg);// 0.35;// 0.25;// 0.15;// 0.2;// 0.1;
                double _limitedMaxThrustAccel = actualLimitedMaxThrustAccel;

                double gComp = VesselState.localg*Math.Abs(Vector3d.Dot(Vessel.up,VesselState.forward));
                
                if ( Core.Landing.g > Core.Landing.LOW_GRAVITY)
                {
                    if (verror < 0)
                    {
                        verror *= 0.005; // reduce gain when ascending and trying to slow down - this avoids overshooting and oscillation.
                    }
                }
                double verrorAbs = Math.Abs(verror);
                velaccum_v += verror;
                velaccum_v = Math.Max(-PID_ACCUM_MAX, Math.Min(PID_ACCUM_MAX, velaccum_v));
                if (verrorAbs > Core.Landing.debug91) velaccum_v = 0;
                double gain;

                if (verrorAbs < Core.Landing.debug3) gain = Core.Landing.debug4;
                else if (verrorAbs < Core.Landing.debug5) gain = Core.Landing.debug4 + Core.Landing.debug6 * Mathf.Clamp01((float)((verrorAbs - Core.Landing.debug3) / (Core.Landing.debug5 - Core.Landing.debug3)));
                else gain = Core.Landing.debug4 + Core.Landing.debug6 + Core.Landing.debug8 * Mathf.Clamp01((float)((verrorAbs - Core.Landing.debug5) / (Core.Landing.debug7 - Core.Landing.debug5)));

                double t = Math.Max(-VesselState.limitedMaxThrustAccel, Math.Min(VesselState.limitedMaxThrustAccel, gComp + gain * (PID_KP*verror + PID_KI*velaccum_v    ))) / VesselState.maxThrustAccel;
                Debug.Log("pidv  t:" + t.ToString("F4") + "  verr:" + verror.ToString("F2") + "  accum:" + velaccum_v.ToString("F4") + "  gain:" + gain.ToString("F4"));
                return t;
            }

            // Build the vertical and horizontal thrust vectors independently, then put it together.
            public void setThrustVector(ref double hTargetError, ref Vector3d desiredThrustVector, ref Vector3d desiredHorizontalVel, Vector3d hError, double hCorrectionAngle, ref double desiredVerticalSpeed, double vError)
            {
                hFrac = vFrac = (hTargetError < hFarLimit) ? 1 : Core.Landing.debug94; // Reduce gains when farther away from target
                bool     upright = false;
                double   vMinAngle;
                double   vMaxAngle;
                double   verticalThrust = pidVelocity_v(vError);
                double   verticalThrust_;
                double   horizontalThrust = pidVelocity_h(ref desiredHorizontalVel, ref hError, ref hCorrectionAngle);
                double   horizontalThrust_ = horizontalThrust;
                Vector3d horizontal = Vector3d.zero;

                // High Gravity
                if (Core.Landing.g > Core.Landing.LOW_GRAVITY)
                {
                    throttleAngle = 30;

                    // settings when ascending
                    if (desiredVerticalSpeed > 0)
                    {
                        if (VesselState.altitudeTrue >= 100)
                        {
                            horizontal = horizontalAvg = horizontalAvg * (1 - Core.Landing.debug96) + Core.Landing.debug96 * horizontalThrust_ * hError.normalized;
                            vMinAngle = 0.2 * Math.Abs(verticalThrust);
                            vMaxAngle = +Math.Abs(verticalThrust);
                            hCorrectionAngle = 0; // not used for this case
                        }
                        else
                        {
                            vMinAngle = 0.2;
                            vMaxAngle = 1.0;
                        }
                        Debug.Log("ascending " + vMinAngle.ToString("F2") + " " + vMaxAngle.ToString("F2"));
                    }

                    // settings when descending
                    else
                    {
                        upright = true;
                        if (VesselState.altitudeTrue < 5000)
                        {
                            vMinAngle = 0.05;
                            vMaxAngle = 0.15; // 0.15 <= 2
                        }
                        else
                        {
                            vMinAngle = 0;
                            vMaxAngle = 1.3;
                        }
                        Debug.Log("descending " + vMinAngle.ToString("F2") + " " + vMaxAngle.ToString("F2"));
                    }

                    // Set horizontal vector if not already set
                    if (horizontal == Vector3d.zero)
                    {
                        horizontal = horizontalAvg = horizontalAvg * (1 - Core.Landing.debug96) + Core.Landing.debug96 * hCorrectionAngle * hError;
                        horizontalThrust_ = Math.Min(horizontal.magnitude, horizontalThrust_);
                    }
                }

                // Low Gravity
                else
                {
                    horizontal = horizontalAvg = horizontalAvg * (1 - Core.Landing.debug96) + Core.Landing.debug96 * horizontalThrust_ * hError.normalized;
                    // settings when ascending
                    if (desiredVerticalSpeed >= 0)
                    {
                        if (VesselState.altitudeTrue >= 100)
                        {
                            vMinAngle = 0.2 * Math.Abs(verticalThrust);
                            vMaxAngle = +Math.Abs(verticalThrust);
                            hCorrectionAngle = 0; // not used for this case
                        }
                        else
                        {
                            vMinAngle = 0.15;
                            vMaxAngle = 1.0;
                        }
                        Debug.Log("lg ascending " + vMinAngle.ToString("F2") + " " + vMaxAngle.ToString("F2"));
                    }
                    else
                    {
                        if (VesselState.altitudeTrue < 30)
                        {
                            upright = true;
                            vMinAngle = 0.05;
                            vMaxAngle = 0.15;
                            throttleAngle = 10;
                        }
                        else
                        {
                            vMinAngle = -Math.Abs(verticalThrust);
                            vMaxAngle = +Math.Abs(verticalThrust);
                            throttleAngle = 30;
                        }
                        Debug.Log("lg descending " + vMinAngle.ToString("F2") + " " + vMaxAngle.ToString("F2"));
                    }

                    // Set horizontal vector if not already set
                    if (horizontal == Vector3d.zero)
                    {
                        Vector3d hthrust = Math.Min(horizontalThrust_, hCorrectionAngle * hError.magnitude) * hError.normalized;
                        horizontal = horizontalAvg = horizontalAvg * (1 - Core.Landing.debug96) + Core.Landing.debug96 * hthrust;
                        horizontalThrust_ = Math.Min(horizontal.magnitude, horizontalThrust_);
                    }
                }

                // Set Vertical Vector
                verticalSign = System.Math.Min(vMaxAngle, System.Math.Max(vMinAngle, verticalSign*(1-Core.Landing.debug95) + Core.Landing.debug95 * vError*Math.Abs(verticalThrust)));
                Vector3d vertical = Vessel.up * verticalSign;

                // do not allow vertical thrust to be opposite of the sign - this causes instability in low gravity when trying to maintain altitude.
                verticalThrust_ = (verticalSign * vError <= 0) ? 0 : verticalThrust;

                // When upright and have vertical thrust, limit horizontal thrust to avoid instability - this is especially important in low gravity when vertical thrust can be very low and easily overcome by horizontal thrust causing the vessel to tip over.
                if ((upright == true) && (vertical.magnitude > 0.0))
                {
                    double horizontal_ = Math.Min(Core.Landing.debug93*vertical.magnitude, horizontal.magnitude);
                    if (horizontal.magnitude > 0)
                    {
                        double hratio = horizontal_ / horizontal.magnitude;
                        horizontal *= hratio;
                        horizontalThrust_ *= hratio;
                    }
                    else
                    {
                        horizontal = Vector3d.zero;
                        horizontalThrust_ = 0;
                    }
                    Debug.Log("hadjust " + horizontal.magnitude.ToString("F3") + "," + horizontalThrust_.ToString("F3"));
                }

                // Initial ascension only vertical thrust is needed until we reach a certain altitude, then we can start to use horizontal thrust to speed up or slow down horizontal movement towards target.
                if (((desiredVerticalSpeed > 0) && (VesselState.altitudeTrue <= 100)))
                {
                    horizontal = horizontalAvg = Vector3d.zero;
                    horizontalThrust_ = 0;
                    Debug.Log("initial ascension");
                }

                // Zero out horizontal thrust if angle to correct thrust is too large to avoid instability
                else if (((horizontal.magnitude > 0.005) && Vector3.Angle(horizontal, VesselState.forward) > 50))   // 50<=(35-bad)<=45
                {
                    horizontalThrust_ = 0;
                }

                // Set desired thrust vector based on horizontal and vertical errors.
                desiredThrustVector = horizontal + vertical;
                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;
                attitudeAngleFromThrust = Core.Attitude.attitudeAngleFromTarget();
                Core.Thrust.RequestActiveThrottle((float)Math.Sqrt(Math.Pow(verticalThrust_, 2) + Math.Pow(horizontalThrust_, 2)));
                Debug.Log("vector  v:" + vertical.magnitude.ToString("F3") + "  h:" + horizontal.magnitude.ToString("F3") + "  hCorrectionAngle:" + hCorrectionAngle.ToString("F3"));
                Debug.Log("thrust  v:" + verticalThrust_.ToString("F3") + "  h:" + horizontalThrust_.ToString("F3") + "  hCorrectionAngle:" + hCorrectionAngle.ToString("F3"));
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

                // For now check collision avoidance while increasing vertical as it may not be enough
                if(vIgnoreCollision > 0)
                {
                    vIgnoreCollision--;
                }
                else if ((hTargetError > 5000) )
                {
                    (double deltaV, bool willCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, 1500);
                    if (deltaV > 0)
                    {
                        vCollisionDeltaV = Math.Max(vCollisionDeltaV, deltaV);
                        desiredVerticalSpeed += vCollisionDeltaV;
                        vError = desiredVerticalSpeed - VesselState.speedVertical;
                        vCollisionCounter = COLLISION_TAIL;
                        Debug.Log("collision1: " + vError.ToString("F2"));
                    }
                    else if (willCrash == true)
                    {
                        vCollisionDeltaV = Math.Max(vCollisionDeltaV, 500);
                        desiredVerticalSpeed += vCollisionDeltaV;
                        vError = desiredVerticalSpeed - VesselState.speedVertical;
                        desiredHorizontalVel = Vector3d.zero;
                        herror = desiredHorizontalVel - VesselState.horizontalSurface;
                        vCollisionCounter = COLLISION_TAIL;
                        Debug.Log("collision2: " + vError.ToString("F2") + "  " + herror.magnitude.ToString("F2"));
                    }
                    else if (vCollisionCounter > 0)
                    {
                        desiredVerticalSpeed += vCollisionDeltaV*(vCollisionCounter/COLLISION_TAIL);
                        vError = desiredVerticalSpeed - VesselState.speedVertical;
                        vCollisionCounter--;
                        Debug.Log("collision3: " + vError.ToString("F2") + "  " + vCollisionCounter.ToString("F0"));
                    }
                    else
                    {
                        vCollisionDeltaV = 0;
                    }
                }

                // Set thrust vector based on vertical and horizontal inputs
                setThrustVector(ref hTargetError, ref desiredThrustVector, ref desiredHorizontalVel, herror, hCorrectionAngle, ref desiredVerticalSpeed, vError);

                // This is a long distance target requiring sub orbital targeting
                if (((hCurrentError > 0) || (Core.Landing.increaseVertical == true)) && (errStep > Ascend2Target.Disabled) && (errStep < Ascend2Target.Completed) )
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
                    }
                }

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status17", // Moving to target: speed(V:<<1>> , H:<<2>>) m/s
                    desiredVerticalSpeed.ToString("F1"), //trace);
                    desiredHorizontalVel.magnitude.ToString("F1"));
                Status += "\n" + trace;
            }

            public void setVerticalSpeed(ref double desiredVerticalSpeed, ref double hTargetError, ref float ratio, ref float minverror, ref float maxverror)
            {
                // Activate Emergency vertical speed increase if far from target and altitude is less than EMERGENCY_ALTITUDE meters and far away than EMERGENCY_H_DISTANCE.
                // The reasoning is that the algorithm should push for a small ratio to begin with so something went wrong.
                if (Core.Landing.increaseVertical == false)
                {
                    emergency = false;
                    if ((ratio > maxRatio) && (VesselState.altitudeTrue < EMERGENCY_ALTITUDE) && (hTargetError > EMERGENCY_H_DISTANCE))
                    {
                        Core.Landing.increaseVertical = true; // Emergency - avoid terrain collision
                        emergency = true;
                    }
                }

                // Increase Vertical so that maxRatio is reached - this can be due to a hop or emergency crash avoidance
                if (Core.Landing.increaseVertical == true)
                {
                    if (ratio > 0.8*maxRatio)
                    {
                        double vFrac = (MainBody.atmosphere == true )? 0.8:0.2;
                        desiredVerticalSpeed = Math.Min(vFrac * Core.Landing.GetCircularOrbitSpeed(VesselState.altitudeTrue, MainBody), Math.Max(3.0, (ratio - maxRatio) * (1 + hTargetError / ASCENT_H_DISTANCE_DIVISOR)));
                        if ((Math.Abs(VesselState.orbitApA - Vessel.terrainAltitude) > Math.Min(1.2 * hTargetError, subOrbitalApA)) && (VesselState.altitudeTrue > ASCENT_AOA_MIN_ALT))
                        {
                            desiredVerticalSpeed = 0;
                        }
                        ignoreLandingCounter = 0; // reset ignore landing count to allow start from a landing
                    }
                    else
                    {
                        desiredVerticalSpeed = 3.0; // give it a little boost to get going and avoid stalling at low altitude with low vertical speed.
                        if (VesselState.altitudeTrue > 15) stopIncrease = true; // Trigger STOP vertical increase
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
                if ((Core.Landing.g > Core.Landing.LOW_GRAVITY) && (Core.Landing.increaseVertical == true) &&
                    (hTargetError > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT) && (ratio > 0.5*maxRatio))
                {
                    // Set horizontal velocity to reach predicted target
                    // Use max expected velocity and scale it based on  
                    double maxHorizontalSpeed;

                    hCurrentError = (Core.Landing.targetingResult.distanceToTarget>0) ? Core.Landing.targetingResult.distanceToTarget : hTargetError;
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

                    _hCorrectionUpAngle = (MainBody.atmosphere == true) ? 0.005 : 0.05;
                    desiredHorizontalVel = maxHorizontalSpeed * courseCorrection.normalized;
                    herror = desiredHorizontalVel - hSurfaceVelocity;
                    Debug.Log("Core.Landing.increaseVertical");
                }

                // Use default method to calculate horizontal velocity
                else
                {
                    if (Core.Landing.increaseVertical == false)
                    {
                        hCurrentError = 0; // Dont allow fine tune when landing.
                    }

                    // When the target is really close then perform then amplify the horizontal error to aggressively eliminate horizontal motion.
                    if (hTargetError < 0.80)
                    {
                        desiredHorizontalVel = (Math.Sqrt((1 + Core.Landing.g) * hTargetError * maxHThrust / divisor) * 2.0 / 1.5) * courseCorrection.normalized;

                        herror = 0.25 * desiredHorizontalVel - hSurfaceVelocity;
                    }

                    // Use the correction vector and a calculated max horizontal speed. Clamp the max to the orbital speed at the current altitude to avoid overshooting the target.
                    else
                    {
                        double maxHorizontalSpeed = Math.Min((Math.Sqrt(Math.Max(3,1 + Core.Landing.g) * hTargetError * maxHThrust / divisor) * 2.0 / 1.5), 
                                                              Math.Sqrt(hTargetError * 2* maxHThrust));
                        if (Core.Landing.g > Core.Landing.LOW_GRAVITY)
                        {
                            maxHorizontalSpeed = System.Math.Min(Core.Landing.GetCircularOrbitSpeed(VesselState.altitudeASL, MainBody), maxHorizontalSpeed);
                            if ((Core.Landing.increaseVertical == false) && (startTiltHVel > 0.1)) maxHorizontalSpeed = Math.Min(maxHorizontalSpeed, startTiltHVel);
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
                    Debug.Log("setHorizontalSpeed: default horizontal");
                }

                // Zero out the surface lateral horizontal velocity relative to the desired horizontal velocity - avoids a limit cycle.
                Vector3d lateralHorizontal;
                if (Core.Landing.g <= Core.Landing.LOW_GRAVITY)
                {
                    lateralHorizontal = Core.Landing.debug12 * gainCancelHVelocity * Vector3d.Exclude(desiredHorizontalVel, VesselState.horizontalSurface);
                }
                else
                {
                    lateralHorizontal = Core.Landing.debug12 * gainCancelHVelocity * Vector3d.Exclude(desiredHorizontalVel, VesselState.horizontalSurface);
                }
                lateralHorizontal = Math.Min(Core.Landing.debug10*herror.magnitude,lateralHorizontal.magnitude) *lateralHorizontal.normalized; // limit lateral to 20% of the horizontal error to avoid instability.
                herror -= lateralHorizontal;

                // Calculate horizontal correction angle using horizontal herror.
                return Math.Max(0, Math.Min(_hCorrectionUpAngle, baseGain * H_CORRECTION_ANGLE_CONSTANT * herror.magnitude));
            }

            public void setSubOrbitalHorizontal(ref Vector3d desiredHorizontalVel, ref Vector3d herror)
            {
                Vector3d hSurfaceVelocity = Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);

                // If vertical increase is enabled then disable horizontal angle at small altitude.
                if (VesselState.altitudeTrue < 100.0 || emergency == true) 
                {
                    herror *= 0.1;
                    desiredHorizontalVel = herror;
                    Debug.Log("setSubOrbitalHorizontal: lowAlt or emergency:");
                }

                else if (((errStep == Ascend2Target.Disabled) || (hCurrentError > 0) || (subOrbitalTileLatched == true)))
                {
                     if (MainBody.atmosphere == true)
                    {
                        // If the desired vertical speed is decreasing then we are in a deceleration phase and should slowly increase the horizontal speed
                        if ((VesselState.atmosphericDensityGrams >= 76) && (subOrbitalTileLatched || (previousDesiredVerticalSpeed > desiredVerticalSpeed)))
                        {
                            if (subOrbitalTileLatched == false)
                            {
                                startTiltHVel = VesselState.speedSurfaceHorizontal;
                            }
                            subOrbitalTileLatched = true;
                            double desiredHVel = desiredHorizontalVel.magnitude;
                            desiredHVel = 150 + (desiredHVel - 150) * Mathf.Clamp01((float)(500 - VesselState.atmosphericDensityGrams) / (1000));
                            startTiltHVel = Math.Min(desiredHVel, startTiltHVel + Math.Max(0.25 / 50.0, 0.001 * startTiltHVel));
                            desiredHorizontalVel = desiredHorizontalVel.normalized * startTiltHVel;
                            Debug.Log("setSubOrbitalHorizontal: slowdown:" + desiredHorizontalVel.magnitude.ToString("F1"));
                        }

                        // Eve      35k alt   atmos density = 125 g/m^3   drag = 30 m/s^2   - FAIL
                        // kerbin   35k alt   atmos density = 2.2 g/m^3   drag = 1.3 m/s^2  - PASS
                        // kerbin 17.5k alt   atmos density =  66 g/m^3 - start of tilt.
                        // kerbin    0k alt   atmos density = 1.2 kg/m^3
                        // for atmospheric planets we taper starting at a specific altitude and increase tilt at higher altitudes
                        // Also qualify it with a minimum vertical velocity before we start tilting.
                        else
                        {
                            if (VesselState.atmosphericDensityGrams < 76.0)  // 76 <= 66
                            {
                                if ((startTiltHVel < 0.1) || (subOrbitalTileLatched = true)) startTiltHVel = VesselState.speedSurfaceHorizontal;
                                double orig_hvel = desiredHorizontalVel.magnitude - startTiltHVel;
                                desiredHorizontalVel = desiredHorizontalVel.normalized * startTiltHVel;
                                if (tiltDelta < 0.1) tiltDelta = MainBody.atmosphereDepth - VesselState.altitudeASL;
                                else if (VesselState.speedVertical > Core.Landing.g * 2 * 28)   // 28<=30<=20(too shallow)<=30
                                {
                                    if (startTiltAlt < 0.1) startTiltAlt = VesselState.altitudeASL;
                                }

                                if ((startTiltAlt >= 0.1) && (VesselState.altitudeASL > startTiltAlt))
                                {
                                    desiredHorizontalVel = desiredHorizontalVel.normalized * (startTiltHVel + orig_hvel * Mathf.Clamp01((float)((VesselState.altitudeASL - startTiltAlt) / tiltDelta)));
                                } // else - dont add any tilt
                            }
                            else
                            {
                                startTiltHVel = 100.0 * Mathf.Clamp01((1024.0f - (float)VesselState.atmosphericDensityGrams) / 1024.0f);
                                desiredHorizontalVel = desiredHorizontalVel.normalized * startTiltHVel;
                            }
                            Debug.Log("setSubOrbitalHorizontal: density:" + VesselState.atmosphericDensityGrams.ToString("F1") + " hDesiredVel:" + desiredHorizontalVel.magnitude.ToString("F1"));
                        }
                    }

                    // else - no atmosphere
                    else
                    {
                        // If the desired vertical speed is decreasing then we are in a deceleration phase and should slowly increase the horizontal speed
                        if (subOrbitalTileLatched || (previousDesiredVerticalSpeed > desiredVerticalSpeed))
                        {
                            if (subOrbitalTileLatched == false)
                            {
                                startTiltHVel = VesselState.speedSurfaceHorizontal;
                            }
                            subOrbitalTileLatched = true;
                            double desiredHVel = desiredHorizontalVel.magnitude;
                            startTiltHVel = Math.Min(desiredHVel, startTiltHVel + Math.Max(2 / 50.0, 0.001 * startTiltHVel));
                            desiredHorizontalVel = desiredHorizontalVel.normalized * startTiltHVel;
                            Debug.Log("setSubOrbitalHorizontal: noatmos slowdown:" + desiredHorizontalVel.magnitude.ToString("F1"));
                        }
                        else
                        {
                            if (startTiltHVel < 0.1) startTiltHVel = VesselState.speedSurfaceHorizontal;
                            double orig_hvel = desiredHorizontalVel.magnitude - startTiltHVel;
                            desiredHorizontalVel = desiredHorizontalVel.normalized * startTiltHVel;

                            if (tiltDelta < 0.1) tiltDelta = Math.Max(500, Math.Abs(Core.Landing.TargetAltitude - VesselState.altitudeASL));
                            else if (VesselState.speedVertical > Core.Landing.g * 2 * 28)   // 28<=30<=20(too shallow)<=30
                            {
                                if (startTiltAlt < 0.1) startTiltAlt = VesselState.altitudeASL;
                            }

                            if ((startTiltAlt >= 0.1) && (VesselState.altitudeASL > startTiltAlt))
                            {
                                desiredHorizontalVel = desiredHorizontalVel.normalized * (startTiltHVel + orig_hvel * Mathf.Clamp01((float)((VesselState.altitudeASL - startTiltAlt) / tiltDelta)));
                            } // else - dont add any tilt
                            Debug.Log("setSubOrbitalHorizontal: no atmos:" + startTiltAlt.ToString("F0") + " hDesiredVel:" + desiredHorizontalVel.magnitude.ToString("F1") +
                                " tiltDelta:" + tiltDelta.ToString("F0"));
                        }
                    }

                    herror = desiredHorizontalVel - hSurfaceVelocity;
                }
                else
                {
                    Debug.Log("setSubOrbitalHorizontal: do nothing");
                }

                previousDesiredVerticalSpeed = desiredVerticalSpeed; // save previous desired vertical speed to determine if we are in a deceleration phase.
            }

            public void TargetSubOrbital(ref Vector3d desiredThrustVector)
            {
                double _hCurrentError = hCurrentError;
                double decelerationStartTime = VesselState.orbitTimeToAp - 4;

                // Use correction attitude if apoapsis is high enough.
                // Determine when we can stop burning when target apoapsis has been reached
                // Once stopped a secondary burn will be performed to reach target
                if (suppressThrottle == 0)
                {
                    double checkApa = (_subOrbitalApA > 0) ? _subOrbitalApA : subOrbitalApA;
                    if (Math.Abs(VesselState.orbitApA) >= checkApa)
                    {
                        if ((decelerationStartTime > 0) && (VesselState.speedVertical > 0))
                        {
                            suppressThrottle++; // suppress throttle during initial burn
                            trace += 'S' + 'a';
                        }
                    }
                }

                // While throttle is disabled monitor it until the second burn can start
                if (suppressThrottle == 1)
                {
                    // Hold on until we get to point where we can burn to target.
                    if ((decelerationStartTime > 0) && (VesselState.speedVertical > 0))
                    {
                        Debug.Log("warp");
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
                        stopIncrease = true;
                        suppressThrottle++;
                        trace += 'U';
                        Core.Warp.MinimumWarp();
                        errStep++; trace += ((int)errStep).ToString();
                    }
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
                    //if (VesselState.speedVertical < 0 && (decStep != DecelerateSteps.IHorizontalDec))
                    //{
                    //    if ((desiredSpeed - VesselState.speedVertical) > 3) allow = false;
                   // }
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
                try
                {
                    desiredVerticalSpeed = GetMaxSpeed();
                }
                catch (Exception ex)
                {
                    // Handle the error gracefully, for example by logging it and setting a default desired speed
                    Debug.Log("Error: " + ex.Message);
                    desiredVerticalSpeed = Core.Landing.MaxAllowedSpeed();
                }

                Vector3d desiredThrustVector = -VesselState.surfaceVelocity.normalized;

                // Move to target strategy
                move(ref desiredVerticalSpeed, ref desiredThrustVector);
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
                if ((vIgnoreCollision==0) && ((suppressThrottle == 1) || (checkWarp == true))) 
                {
                    Core.Thrust.ThrustOff();
                }
                else
                {
                    if ( (attitudeAngleFromThrust > throttleAngle) && (allowDisableThrust(desiredVerticalSpeed) == true) )
                    {
                        Core.Thrust.RequestActiveThrottle(0);
                    }
                }

                if (double.IsNaN(desiredVerticalSpeed)) return new MoveToTarget(Core);
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
