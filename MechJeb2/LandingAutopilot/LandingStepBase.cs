using System;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Common base for the landing-autopilot steps that share trip-speed, brake-budget, geometry
    /// and logging logic: TargetSubOrbit, MoveToTarget and LandingBurn.
    /// Why: these steps used to carry their own copies of the same constants and formulas, which
    /// drifted apart (e.g. 8g/0.7 vs 16g/0.85). Anything used by more than one step lives here so
    /// it is defined once. How: a step derives from LandingStepBase instead of AutopilotStep and
    /// uses the members below directly. Nothing here commands the vessel.
    /// </summary>
    public abstract class LandingStepBase : AutopilotStep
    {
        // ---------------------------------------------------------------------------------
        // Shared tuning constants
        // ---------------------------------------------------------------------------------

        /// <summary>Max g-multiple for the planned brake / trip-cruise acceleration
        /// (Ricardo Nov 18 19:42, was 8.0). Used by PlanBrakeAccel and the TSO trip-cruise caps.</summary>
        protected const double TRIP_CRUISE_MAX_G_MULT = 16.0;

        /// <summary>Fraction of max thrust acceleration the brake / trip-cruise plan may use
        /// (Ricardo Nov 18 19:42, was 0.7). Used by PlanBrakeAccel, TSO caps and LB gain floor.</summary>
        protected const double TRIP_CRUISE_MAX_MARGIN = 0.85;

        /// <summary>Gain from horizontal error to horizontal correction angle (TSO and MTT).</summary>
        protected const float H_CORRECTION_ANGLE_CONSTANT = 0.4f;

        /// <summary>Fraction of the max overshoot angle actually used, a small margin so vertical
        /// control is never lost (TSO and MTT hFarCorrectionAngle).</summary>
        protected const double OVERSHOOT_ANGLE_FRACTION = 0.9;

        /// <summary>Thrust-acceleration cap as a multiple of body g (MTT limitedMaxThrustAccel and the
        /// LB mover-handoff check). TSO uses a scaled value, see TargetSubOrbit. Revisit later.</summary>
        public const double LIMITED_MAX_THRUST_G_RATIO = 5.0;

        /// <summary>Floor (m/s^2) for any acceleration used as a thrust budget or as a divisor,
        /// so a zero-thrust or degenerate state never divides by zero.</summary>
        protected const double MIN_ACCEL = 0.1;

        /// <summary>Floor (m/s^2) for local gravity used in brake planning (very small bodies).</summary>
        protected const double MIN_LOCAL_G = 0.05;

        /// <summary>The planned brake acceleration is always at least local g plus this (m/s^2),
        /// so a low-TWR plan can still arrest the vertical speed.</summary>
        protected const double PLAN_ACCEL_G_MARGIN = 0.05;

        /// <summary>Squared-magnitude threshold below which a vector is treated as zero
        /// before normalising it.</summary>
        protected const double VEC_SQR_EPS = 1e-12;

        /// <summary>Squared-magnitude threshold for direction vectors (orbit normal, velocity)
        /// below which the direction is considered undefined.</summary>
        protected const double DIR_SQR_EPS = 1e-6;

        /// <summary>Small positive floor for a divisor or square-root argument (time step, throttle
        /// divisor, thrust accel, distance) so a zero value cannot produce Inf/NaN.</summary>
        protected const double TINY_DIVISOR = 1e-6;

        /// <summary>Span below which an interpolation range is treated as zero (avoids 0/0).</summary>
        protected const double SPAN_EPS = 1e-9;

        /// <summary>Floor (m/s^2) on limited thrust accel in g-based throttle caps.</summary>
        protected const double THRUST_ACCEL_TINY = 1e-3;

        /// <summary>A g-based throttle cap never goes below this throttle, so the step can still steer.</summary>
        protected const double MIN_THROTTLE_CAP = 0.03;

        /// <summary>Look-ahead (s) for Core.Landing.PredictAvoidCollisionDeltaV collision-avoidance checks.</summary>
        protected const double CA_LOOKAHEAD_S = 15;

        /// <summary>Default throttle-permission cone (deg): throttle only when attitude is within this of target.</summary>
        protected const double THROTTLE_ANGLE_DEG = 30.0;

        /// <summary>Minimum steepness of the final-descent ratio curve (TSO and MTT).</summary>
        protected const float MIN_STEEPNESS = 10.0f;

        /// <summary>Thrust-cap scale (x LIMITED_MAX_THRUST_G_RATIO x g) when flying aggressively
        /// (FlySafe off; TSO always).</summary>
        protected const double TEMP_FACTOR_AGGRESSIVE = 20.0;

        /// <summary>Thrust-cap scale when FlySafe is on (MTT).</summary>
        protected const double TEMP_FACTOR_SAFE = 10.0;

        /// <summary>Gravity range (m/s^2) over which gains are interpolated (GravityScheduled):
        /// G_SCHED_MIN maps to the low-g value, G_SCHED_MAX to the high-g value.</summary>
        protected const double G_SCHED_MIN = 0.05;
        protected const double G_SCHED_MAX = 2.0;

        /// <summary>baseGain bounds: numerator / g at low and high gravity (TSO and MTT horizontal gain).</summary>
        protected const double BASE_GAIN_LOW_NUM = 0.03136;
        protected const double BASE_GAIN_HIGH_NUM = 0.4704;

        /// <summary>Gravity-scheduled velocity-PID gains shared by TSO pidVelocity and the MTT PIDs:
        /// Kp from PID_KP_LOW_G to PID_KP_HIGH_G, Ki from PID_KI_LOW_G to PID_KI_HIGH_G.</summary>
        protected const double PID_KP_LOW_G = 20;
        protected const double PID_KP_HIGH_G = 50;
        protected const double PID_KI_LOW_G = 0.05;
        protected const double PID_KI_HIGH_G = 0.17;

        /// <summary>Ratio-curve fraction below which the descent vz is exactly 0 (RatioCurveVz).</summary>
        protected const double RATIO_CURVE_MIN_FRACTION = 0.0001;

        /// <summary>Altitude-hold vertical-speed loop (TSO trip hold, MTT trip climb hold):
        /// vz = clamp(error / ALT_HOLD_TAU_S, +/-ALT_HOLD_VZ_MAX).</summary>
        protected const double ALT_HOLD_VZ_MAX = 5.0;
        protected const double ALT_HOLD_TAU_S = 30.0;

        /// <summary>Coast warp-rate guard (LB and MTT): assumed fall speed is at least
        /// localg * WARP_FALL_GUESS_S (the speed after falling this long from a hover), and the rate
        /// never exceeds the orbit period / WARP_PERIOD_DIVISOR.</summary>
        protected const double WARP_FALL_GUESS_S = 5;
        protected const double WARP_PERIOD_DIVISOR = 6;

        /// <summary>Converts BurnMarginPerc (percent) to a fraction.</summary>
        protected const double PERCENT = 100.0;

        /// <summary>Base stop-distance multiplier for the horizontal (downrange) brake distance,
        /// LandingBurn d_horiz. BurnMarginPerc is added on top.</summary>
        protected const double BRAKE_STOP_MARGIN_H = 1.00;

        /// <summary>Base stop-distance multiplier for vertical brake heights (LandingBurn h_vert /
        /// h_suicide). BurnMarginPerc is added on top.</summary>
        protected const double BRAKE_STOP_MARGIN_V = 1.05;

        /// <summary>Default interval (s) between repeated diagnostic log lines.</summary>
        protected const double DEFAULT_LOG_INTERVAL_S = 1.0;

        /// <summary>Interval (s) of the [TSO-1Hz] / [LB-1Hz] guidance snapshot lines.</summary>
        protected const double GUIDANCE_LOG_INTERVAL_S = 1.0;

        /// <summary>Minimum squared horizontal pad distance (m^2) for the pad direction to be
        /// considered defined (closer than 1 m the direction is noise).</summary>
        protected const double MIN_PAD_DIR_SQR_M2 = 1.0;

        /// <summary>Minimum radius (m) used as a divisor guard for body-centred positions.</summary>
        protected const double MIN_RADIUS_M = 1.0;

        protected LandingStepBase(MechJebCore core) : base(core) { }

        // ---------------------------------------------------------------------------------
        // Gain schedules and caps
        // ---------------------------------------------------------------------------------

        /// <summary>Where g lies in [G_SCHED_MIN, G_SCHED_MAX] (0..1, not clamped).</summary>
        protected static double GravityFraction(double g) => (g - G_SCHED_MIN) / (G_SCHED_MAX - G_SCHED_MIN);

        /// <summary>
        /// Value interpolated from atMinG to atMaxG by GravityFraction(g), clamped to [floor, ceil].
        /// Used for the gravity-scheduled PID gains so TSO and MTT share one schedule.
        /// </summary>
        protected static double GravityScheduled(double atMinG, double atMaxG, double floor, double ceil, double g) =>
            Math.Max(floor, Math.Min(ceil, atMinG + (atMaxG - atMinG) * GravityFraction(g)));

        /// <summary>Horizontal base gain for body gravity g (between BASE_GAIN_LOW_NUM/g and
        /// BASE_GAIN_HIGH_NUM/g). Set in TSO and MTT OnStart-style init.</summary>
        protected static double ComputeBaseGain(double g)
        {
            double lo = BASE_GAIN_LOW_NUM / g;
            double hi = BASE_GAIN_HIGH_NUM / g;
            return GravityScheduled(lo, hi, lo, hi, g);
        }

        /// <summary>Velocity-PID Kp for gravity g (shared schedule).</summary>
        protected static double ScheduledKp(double g) => GravityScheduled(PID_KP_LOW_G, PID_KP_HIGH_G, PID_KP_LOW_G, PID_KP_HIGH_G, g);

        /// <summary>Velocity-PID Ki for gravity g (shared schedule).</summary>
        protected static double ScheduledKi(double g) => GravityScheduled(PID_KI_LOW_G, PID_KI_HIGH_G, PID_KI_LOW_G, PID_KI_HIGH_G, g);

        /// <summary>
        /// Throttle cap that limits thrust acceleration to gMult * g (g already floored by the caller):
        /// clamp(gMult*g / aMax, MIN_THROTTLE_CAP, 1). Used by the TSO trip/lateral/prograde burns and
        /// the MTT vacuum Ap-strip so low-g bodies are not driven at full thrust.
        /// </summary>
        protected double GMultThrottleCap(double gMult, double g) =>
            Math.Max(MIN_THROTTLE_CAP, Math.Min(1.0, gMult * g / Math.Max(THRUST_ACCEL_TINY, VesselState.limitedMaxThrustAccel)));

        /// <summary>
        /// Final-descent vertical speed from the downrange/altitude ratio curve (TSO and MTT
        /// setVerticalSpeed): 0 at or above maxRatio, the full -|desiredVz| at minRatio, shaped by an
        /// exponential of the given steepness in between.
        /// </summary>
        protected static float RatioCurveVz(double desiredVz, float ratio, float maxRatio, float minRatio, float steepness)
        {
            float fraction = (ratio >= maxRatio) ? 0.0f : Mathf.Max(0.0f, Mathf.Min(1.0f, (maxRatio - ratio) / (maxRatio - minRatio)));
            return (fraction < RATIO_CURVE_MIN_FRACTION) ? 0 : -(float)Math.Abs(desiredVz) * ((Mathf.Pow(steepness, fraction) - 1.0f) / (steepness - 1.0f));
        }

        /// <summary>Altitude-hold vertical speed (m/s) for an altitude error (m): error / ALT_HOLD_TAU_S
        /// clamped to +/-ALT_HOLD_VZ_MAX.</summary>
        protected static double AltitudeHoldVz(double errorM) =>
            Math.Max(-ALT_HOLD_VZ_MAX, Math.Min(ALT_HOLD_VZ_MAX, errorM / ALT_HOLD_TAU_S));

        /// <summary>
        /// Safe regular-warp rate for a coast at altitude alt: alt / (altDivisor * fall speed), with the
        /// fall speed at least localg * WARP_FALL_GUESS_S, capped at period / WARP_PERIOD_DIVISOR.
        /// LB and MTT pass their own altDivisor.
        /// </summary>
        protected double CoastWarpRate(double alt, double altDivisor)
        {
            double velocityGuess = Math.Max(Math.Abs(VesselState.speedVertical), VesselState.localg * WARP_FALL_GUESS_S);
            return Math.Min(alt / (altDivisor * velocityGuess), Orbit.period / WARP_PERIOD_DIVISOR);
        }

        // ---------------------------------------------------------------------------------
        // Trip-speed lever (Ricardo t231u)
        // ---------------------------------------------------------------------------------

        /// <summary>Trip speed factor k (TripSpeedFactor setting). 1.0 = lever off.</summary>
        protected double TripK => Core.Landing.TripSpeedK;

        /// <summary>True when the trip lever is active (k &gt; 1). Every trip branch is guarded by this.</summary>
        protected bool TripOn => Core.Landing.TripSpeedOn;

        // ---------------------------------------------------------------------------------
        // Brake budget
        // ---------------------------------------------------------------------------------

        /// <summary>
        /// Planned brake acceleration (m/s^2) shared by LandingBurn ignition/guidance and the TSO
        /// trip planner: min(TRIP_CRUISE_MAX_G_MULT*g, TRIP_CRUISE_MAX_MARGIN*aMax), at least
        /// g + PLAN_ACCEL_G_MARGIN, never above aMax. Why one function: LB's ignition distance and
        /// TSO's prediction of it must use the same budget or TSO hands off at the wrong point.
        /// </summary>
        /// <param name="aMax">Limited max thrust acceleration used (floored at MIN_ACCEL).</param>
        protected double PlanBrakeAccel(out double aMax)
        {
            aMax = Math.Max(VesselState.limitedMaxThrustAccel, MIN_ACCEL);
            double g = Math.Max(VesselState.localg, MIN_LOCAL_G);
            double aPlan = Math.Min(TRIP_CRUISE_MAX_G_MULT * g, TRIP_CRUISE_MAX_MARGIN * aMax);
            aPlan = Math.Max(aPlan, g + PLAN_ACCEL_G_MARGIN);
            return Math.Min(aPlan, aMax);
        }

        /// <summary>PlanBrakeAccel without the aMax output.</summary>
        protected double PlanBrakeAccel() => PlanBrakeAccel(out _);

        /// <summary>
        /// Stop distance (m) for speed v at deceleration a: (marginBase + BurnMarginPerc/100) * v^2 / (2a),
        /// with a floored at MIN_ACCEL. This is the LandingBurn formula; pass BRAKE_STOP_MARGIN_H for
        /// downrange distance and BRAKE_STOP_MARGIN_V for vertical heights.
        /// </summary>
        protected double BrakeStopDistance(double speed, double accel, double marginBase)
        {
            double margin = marginBase + Core.Landing.BurnMarginPerc / PERCENT;
            return margin * speed * speed / (2.0 * Math.Max(accel, MIN_ACCEL));
        }

        /// <summary>
        /// Splits a retrograde brake along velocity vel into vertical (sinTheta) and horizontal
        /// (cosTheta) shares relative to up. Used by LandingBurn ignition and the TSO ignition
        /// prediction so both scale aPlan the same way. A zero velocity counts as horizontal.
        /// </summary>
        protected static void RetroBrakeShares(Vector3d vel, Vector3d up, out double sinTheta, out double cosTheta)
        {
            if (vel.sqrMagnitude < DIR_SQR_EPS)
            {
                sinTheta = 0.0;
                cosTheta = 1.0;
                return;
            }
            Vector3d retro = -vel.normalized;
            sinTheta = Math.Abs(Vector3d.Dot(retro, -up));
            cosTheta = Math.Sqrt(1.0 - sinTheta * sinTheta);
        }

        // ---------------------------------------------------------------------------------
        // Geometry
        // ---------------------------------------------------------------------------------

        /// <summary>Vessel CoM relative to the body centre (the frame of GetPositionTargetPosition).</summary>
        protected Vector3d VesselPosBody => VesselState.CoM - MainBody.position;

        /// <summary>Surface velocity with the vertical component removed.</summary>
        protected Vector3d HorizontalSurfaceVelocity => Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);

        /// <summary>Horizontal (local-up excluded) vector from the vessel to the landing target.</summary>
        protected Vector3d HorizontalToPad()
        {
            Vector3d tgtBody = Core.Target.GetPositionTargetPosition(); // body-relative
            return Vector3d.Exclude(VesselState.up, tgtBody - VesselPosBody);
        }

        /// <summary>
        /// Component (m/s) of the body's surface rotation velocity at the vessel along toPad
        /// (orbital - surface velocity, horizontal). 0 when toPad is shorter than 1 m.
        /// Used to convert between orbital and surface-relative trip speeds (TSO and MTT).
        /// </summary>
        protected double SurfaceRotationAlong(Vector3d toPad)
        {
            Vector3d vRot = Vector3d.Exclude(VesselState.up, VesselState.orbitalVelocity - VesselState.surfaceVelocity);
            return (toPad.sqrMagnitude > MIN_PAD_DIR_SQR_M2) ? Vector3d.Dot(vRot, toPad.normalized) : 0.0;
        }

        /// <summary>
        /// Unit thrust direction and its components for burn-axis log labels: upDot = dot with up,
        /// drDot = dot of its horizontal part with downrange. Returns false for a ~zero direction.
        /// Each step maps the components to its own labels and thresholds (DescribeBurnAxis).
        /// </summary>
        protected static bool ThrustAxisComponents(Vector3d dir, Vector3d up, Vector3d downrange,
            out Vector3d unitDir, out double upDot, out double drDot)
        {
            unitDir = Vector3d.zero;
            upDot = 0.0;
            drDot = 0.0;
            if (dir.sqrMagnitude <= VEC_SQR_EPS)
                return false;
            unitDir = dir.normalized;
            upDot = Vector3d.Dot(unitDir, up);
            Vector3d horiz = Vector3d.Exclude(up, unitDir);
            if (downrange.sqrMagnitude > VEC_SQR_EPS && horiz.sqrMagnitude > VEC_SQR_EPS)
                drDot = Vector3d.Dot(horiz.normalized, downrange.normalized);
            return true;
        }

        // ---------------------------------------------------------------------------------
        // Logging helpers
        // ---------------------------------------------------------------------------------

        /// <summary>
        /// Rate limiter for repeated log lines and periodic evaluations: returns true (and stamps
        /// lastUt = now) when at least intervalS has passed since lastUt, else false.
        /// </summary>
        protected static bool RateLimit(ref double lastUt, double now, double intervalS)
        {
            if (now - lastUt < intervalS)
                return false;
            lastUt = now;
            return true;
        }

        private double _lastTripLogUt = double.NegativeInfinity;

        /// <summary>Interval (s) between non-forced [TripSpeed] lines; a step may override it.</summary>
        protected virtual double TripLogIntervalS => DEFAULT_LOG_INTERVAL_S;

        /// <summary>
        /// Writes a "[TripSpeed]" line to KSP.log, rate-limited to TripLogIntervalS. force=true always
        /// logs and does not reset the timer, so one-off gate lines do not starve the periodic line.
        /// </summary>
        protected void LogTrip(string msg, bool force = false)
        {
            if (!force && !RateLimit(ref _lastTripLogUt, Planetarium.GetUniversalTime(), TripLogIntervalS))
                return;
            Debug.Log("[TripSpeed] " + msg);
        }

        private double _lastGuidanceLogUt = double.NegativeInfinity;

        /// <summary>True once per GUIDANCE_LOG_INTERVAL_S: gate for the 1 Hz guidance snapshot lines.</summary>
        protected bool GuidanceLogDue(double now) => RateLimit(ref _lastGuidanceLogUt, now, GUIDANCE_LOG_INTERVAL_S);

        /// <summary>Horizontal distance to target for logging; returns fallback if it cannot be computed.</summary>
        protected double HDistanceToTargetOr(double fallback)
        {
            try { return Core.Landing.getHDistanceToTarget(); }
            catch { return fallback; }
        }
    }
}
