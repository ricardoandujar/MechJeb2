using System;
using KSP.Localization;
using UnityEngine;


namespace MuMech
{
    namespace Landing
    {
        public class MoveToTarget : AutopilotStep
        {
            // Live landing mover: unified 3D PID in the landing frame (H/V/L).
            // Axis-independent reference: MoveToTargetAxisIndependent.
            // Vector PID helper: LandingFramePID in MoveToTargetV.cs.

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
            private LandingFramePID velocityPID;            // Unified 3D velocity PID (along-track, vertical, lateral)
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
            // Landing-frame H/L smoothing (world-frame horizontalAvg re-coupled axes when hDir rotated).
            private double aHAvg;
            private double aLAvg;
            private Vector3d prevHDir;
            private Vector3d prevLatDir;
            private bool hasPrevLandingFrame;
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
            // hopBlend 1 = full H/L strip (vertical-first). Decays with remaining
            // V boost / HOP_STRIP_BOOST so the strip drop does not slam aH.
            private bool collisionHopActive = false;
            private double hopBlend = 0;
            // Set when FineTune reached but terrain/hop blocked TargetSubOrbit handoff.
            // Retried each Drive until CoastClearForSubOrbitHandOff, then hand off.
            private bool pendingSubOrbitHandOff = false;
            private bool _coastClearThisDrive;
            private bool _coastClearResult;
            // Vacuum far hop: do not arm TargetSubOrbital coast/warp until H toward pad
            // is established (avoids Ap over the launch site / frac~0).
            private const double MIN_DOWNRANGE_H_TOWARD_TARGET = 50.0; // m/s along-track toward pad
            private const double DOWNRANGE_H_LOG_INTERVAL = 2.0; // seconds between wait/hold spam logs
            private double _lastDownrangeHLogUT = double.NegativeInfinity;
            private const bool LOG_LANDING_TICK = false; // Unity Debug.Log on the physics thread freezes the mouse
            private const double HOP_STRIP_BOOST = 20; // m/s remaining V: full strip above this
            private const double HOP_REARM_RATIO = 1.5; // re-arm in-progress hop only if new dV is well above remaining and the armed peak
            // PredictAvoid clearance for departure climb-out (not pad).
            // Was 1500: with long localRimLook it held vertical-first past the rim.
            private const double AVOID_MIN_ALTITUDE = 900.0;
            // Soft leftover-V dump band (desired V ~ 0 cruise). Hard cut at +2 slammed
            // vertical command on/off as V oscillated around that threshold.
            private const double DUMP_V_FULL_EXCESS = 5.0; // m/s above desired: full pitch-over dump
            private const double DUMP_V_START_EXCESS = 0.25; // begin fading dump above desired

            // True when MoveToTarget may safely hand off to TargetSubOrbit: no armed collision
            // hop and the hop predictor (trajectory + heading rims) reports clear. Keeps hop
            // in charge over irregular terrain (Vesta) until the coast is actually clear.
            // nearLandingSite (hTargetError <= 5 km): pad path only -- do not require
            // terrain clear for suborbit handoff near the pad. Away from the pad,
            // clear means LOCAL rim/terrain PredictAvoid is clear (not "far from pad").
            private bool CoastClearForSubOrbitHandOff(double hTargetError)
            {
                if (_coastClearThisDrive)
                    return _coastClearResult;
                _coastClearThisDrive = true;
                bool nearLandingSite = hTargetError <= 5000;
                if ((vCollisionCounter > 0) || (hopBlend > 0))
                    _coastClearResult = false;
                else if (nearLandingSite)
                    _coastClearResult = true;
                else
                {
                    (double deltaV, bool willCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, AVOID_MIN_ALTITUDE);
                    _coastClearResult = !(willCrash || deltaV > 0);
                }
                return _coastClearResult;
            }

            /// <summary>
            /// Vacuum far hop: horizontal velocity toward the pad must exist before arming
            /// TargetSubOrbital suppressThrottle/warp. Pure vertical Ap leaves Ap over start.
            /// </summary>
            /// <summary>
            /// Design cruise Ap for vacuum far hops. Prefer latched _subOrbitalApA but never above
            /// 2x baseline (a runaway latch must not become the coast/H-need target).
            /// </summary>
            private double CruiseApaTarget()
            {
                double baseline = Math.Max(1000.0, subOrbitalApA);
                double cap = 2.0 * baseline;
                if (_subOrbitalApA > 0)
                    return Math.Min(_subOrbitalApA, cap);
                return baseline;
            }

            // ---- Trip-speed lever (Ricardo t231u). Every behaviour branch is guarded by TripOn
            // (k > 1 + 1e-9); at k == 1.0 none of the code below changes a command. ----
            private double TripK => Core.Landing.TripSpeedK;
            private bool TripOn => Core.Landing.TripSpeedOn;
            // CA owns the climb while a hop is armed or its tail is decaying; the lever waits.
            private bool TripCaQuiet => (vCollisionCounter <= 0) && (hopBlend <= 0);
            private double _lastTripLogUT = double.NegativeInfinity;

            private void LogTrip(string msg)
            {
                double ut = VesselState.time;
                if ((ut - _lastTripLogUT) < DOWNRANGE_H_LOG_INTERVAL)
                    return;
                _lastTripLogUT = ut;
                Debug.Log("[TripSpeed] " + msg);
            }

            // 1.15x vacuum Ap strip / dump threshold. Relaxed to 1.15*k only while the lever is on
            // AND CA is quiet (a powered climb braking at k*g peaks its Kepler Ap at ~h + k*(cruise-h)).
            // k == 1.0 (or CA active): exactly 1.15, the pre-lever constant.
            private double ApStripFactor()
            {
                if (TripOn && TripCaQuiet)
                    return 1.15 * TripK;
                return 1.15;
            }

            // Same preconditions as setVerticalSpeed's lowGSubOrbitalClimb (vacuum low-g far hop).
            private bool TripLowGFarHop(double hTargetError)
            {
                return (MainBody.atmosphere == false)
                    && (Core.Landing.LandingType == 1)
                    && (Core.Landing.g <= Core.Landing.LOW_GRAVITY)
                    && (hTargetError > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT)
                    && (errStep < Ascend2Target.FineTune)
                    && (suppressThrottle == 0);
            }

            /// <summary>
            /// Step 3 (k > 1 only, CA quiet): powered climb to the cruise ASL with active vertical-speed
            /// cancel there instead of today's ballistic lob + coast-up. Below the braking curve
            /// vz = sqrt(2 k g (cruise - h)) climb at up to (k-1)*200 m/s; on/above it command the curve
            /// (vz -> 0 at cruise) - the existing dump/pitch-over path supplies the (k-1) g of down-thrust.
            /// At/above cruise hold ASL (+/-5 m/s). Cruise = CruiseApaTarget() (terrain Ap floor, unchanged).
            /// </summary>
            private void TripPoweredClimb(ref double desiredVerticalSpeed)
            {
                double cruise = CruiseApaTarget();
                double cruiseHold = cruise + 100.0; // small bias so Ap ends >= cruise (TargetSubOrbital gate)
                double asl = VesselState.altitudeASL;
                double vz = VesselState.speedVertical;
                double gk = TripK * Math.Max(VesselState.localg, 0.05);
                double toGo = cruiseHold - asl;
                double vzArrive = (toGo > 0) ? Math.Sqrt(2.0 * gk * toGo) : 0.0;
                double vzCap = Math.Max(3.0, (TripK - 1.0) * 200.0);
                string phase;
                if ((toGo > 0) && (vz < vzArrive))
                {
                    desiredVerticalSpeed = Math.Max(3.0, Math.Min(vzCap, vzArrive));
                    phase = "climb";
                }
                else if (toGo > 0)
                {
                    desiredVerticalSpeed = vzArrive; // on the k*g braking curve: cancel vz under power
                    phase = "brake";
                }
                else
                {
                    desiredVerticalSpeed = Math.Max(-5.0, Math.Min(5.0, toGo / 30.0)); // hold cruise ASL
                    phase = "hold";
                }
                LogTrip("MTT " + phase + " k=" + TripK.ToString("F2")
                    + " asl=" + asl.ToString("F0") + " cruise=" + cruise.ToString("F0")
                    + " vz=" + vz.ToString("F1") + " desV=" + desiredVerticalSpeed.ToString("F1")
                    + " H=" + VesselState.speedSurfaceHorizontal.ToString("F1")
                    + " ApA=" + VesselState.orbitApA.ToString("F0"));
            }

            // k-scaled downrange H gate (same formula as HasMeaningfulDownrangeTowardTarget).
            private double TripHNeed()
            {
                double need = MIN_DOWNRANGE_H_TOWARD_TARGET;
                double circ = Core.Landing.GetCircularOrbitSpeed(Math.Max(1000.0, subOrbitalApA), MainBody);
                if (!double.IsNaN(circ) && !double.IsInfinity(circ) && circ > 0)
                    need = Math.Max(need, 0.25 * circ);
                return need * TripK;
            }

            /// <summary>
            /// k > 1 only: the powered climb reaches cruise in ~1/3 of today's time, so the apex is near the
            /// start. MTT has no coast-floor check, so cap the climb H command at 1.2x the k-scaled H gate;
            /// the real speed-up happens in TSO at cruise height (terrain-checked there).
            /// </summary>
            private void TripCapClimbH(ref Vector3d desiredHorizontalVel, ref Vector3d herror)
            {
                double cap = 1.2 * TripHNeed();
                if (desiredHorizontalVel.magnitude > cap && desiredHorizontalVel.sqrMagnitude > 1e-9)
                {
                    desiredHorizontalVel = desiredHorizontalVel.normalized * cap;
                    herror = desiredHorizontalVel - Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                }
            }

            // Body-general H cap during the Ap hold: keep commanded H near the gate so the
            // vertical bleed can finish at the cruise floor. No-brake if current H is already above cap.
            private void CapHDuringApHold(ref Vector3d desiredHorizontalVel, ref Vector3d herror)
            {
                double cap = 1.05 * TripHNeed();
                double currentH = VesselState.speedSurfaceHorizontal;
                if (currentH > cap)
                    cap = currentH;
                if (desiredHorizontalVel.magnitude > cap && desiredHorizontalVel.sqrMagnitude > 1e-9)
                {
                    desiredHorizontalVel = desiredHorizontalVel.normalized * cap;
                    herror = desiredHorizontalVel - Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                    LogDownrangeH("CapHDuringApHold: cap=" + cap.ToString("F1")
                        + " need=" + TripHNeed().ToString("F1")
                        + " H=" + currentH.ToString("F1")
                        + " ApA=" + VesselState.orbitApA.ToString("F0"));
                }
            }

            /// <summary>
            /// Step 1: read-only headroom line at the MTT -> TSO handoff (no command changes).
            /// </summary>
            private void LogTripSpeedHeadroom()
            {
                try
                {
                    Vector3d posBody = VesselState.CoM - MainBody.position;
                    double r = posBody.magnitude;
                    double vCirc = Math.Sqrt(MainBody.gravParameter / Math.Max(r, 1.0));
                    Vector3d tgtBody = Core.Target.GetPositionTargetPosition(); // body-relative
                    Vector3d toTgt = Vector3d.Exclude(VesselState.up, tgtBody - posBody);
                    // Surface rotation velocity at the vessel = orbital - surface velocity (exact).
                    Vector3d vRot = Vector3d.Exclude(VesselState.up, VesselState.orbitalVelocity - VesselState.surfaceVelocity);
                    double vRotAlong = (toTgt.sqrMagnitude > 1.0) ? Vector3d.Dot(vRot, toTgt.normalized) : 0.0;
                    double g = Math.Max(VesselState.localg, 0.05);
                    // Same args as move()/CoastClearForSubOrbitHandOff this tick -> per-UT cache.
                    (double caDv, bool caCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, AVOID_MIN_ALTITUDE);
                    Debug.Log("[TripSpeed] k=" + TripK.ToString("F2")
                        + " ut=" + VesselState.time.ToString("F1")
                        + " vH=" + VesselState.speedSurfaceHorizontal.ToString("F1")
                        + " vCircSurf=" + (vCirc - vRotAlong).ToString("F1")
                        + " vCirc=" + vCirc.ToString("F1")
                        + " vRotAlong=" + vRotAlong.ToString("F1")
                        + " vz=" + VesselState.speedVertical.ToString("F1")
                        + " coastUp=" + (Math.Max(0, VesselState.speedVertical) / g).ToString("F0")
                        + " cruiseApA=" + CruiseApaTarget().ToString("F0")
                        + " ApA=" + VesselState.orbitApA.ToString("F0")
                        + " aMax=" + VesselState.limitedMaxThrustAccel.ToString("F2")
                        + " aMaxFull=" + VesselState.maxThrustAccel.ToString("F2")
                        + " g=" + g.ToString("F3")
                        + " caDv=" + caDv.ToString("F0")
                        + " caCrash=" + caCrash);
                }
                catch (Exception e)
                {
                    Debug.Log("[TripSpeed] headroom log failed: " + e.Message);
                }
            }

            private void LogDownrangeH(string msg)
            {
                double ut = VesselState.time;
                if ((ut - _lastDownrangeHLogUT) < DOWNRANGE_H_LOG_INTERVAL)
                    return;
                _lastDownrangeHLogUT = ut;
                Debug.Log(msg);
            }

            private bool HasMeaningfulDownrangeTowardTarget()
            {
                Vector3d targetPos = Core.Target.GetPositionTargetPosition();
                if (!MechJebModuleLandingAutopilot.IsFiniteVec(targetPos))
                    return false;

                Vector3d toTargetFlat = Vector3d.Exclude(VesselState.up, targetPos - VesselState.CoM);
                if (toTargetFlat.sqrMagnitude < 1.0)
                    return true; // already over pad horizontally

                Vector3d hVel = Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                double toward = Vector3d.Dot(hVel, toTargetFlat.normalized);
                double need = MIN_DOWNRANGE_H_TOWARD_TARGET;
                // Always size H need from design cruise (baseline subOrbitalApA), not a runaway
                // latched Ap. Midnight log: latched multi-Mm Ap inflated need so H=60 still waited.
                double cruiseAlt = Math.Max(1000.0, subOrbitalApA);
                double circ = Core.Landing.GetCircularOrbitSpeed(cruiseAlt, MainBody);
                if (!double.IsNaN(circ) && !double.IsInfinity(circ) && circ > 0)
                    need = Math.Max(need, 0.25 * circ);
                if (TripOn)
                    need *= TripK; // Step 2: H gate scales with the trip-speed factor (k > 1 only)

                return toward >= need;
            }

            /// <summary>
            /// Vacuum far hop: once ApA is at/above the suborbital cruise target, stop raising Ap.
            /// Keep building H via tilt/horizontal only. Soft-dump V if Ap already way overshot
            /// (never climb toward escape while waiting for downrange H).
            /// </summary>
            private void HoldApWhileBuildingDownrangeH(ref double desiredVerticalSpeed, double checkApa)
            {
                // Midnight KSP.log (52b5d313): wait started at ApA~24 km H~3, then Ap ran
                // away to hyperbolic while H stayed ~2-3. desiredVerticalSpeed=0 kept the
                // low-g ascending envelope + allowPitchOver off, so thrust stayed vertical.
                // Keep desired V slightly negative so pitch-over stays armed and H can build;
                // dump harder when Ap is already overshot or hyperbolic.
                double apaSigned = VesselState.orbitApA;
                double apa = Math.Abs(apaSigned);
                double targetApa = (checkApa > 0) ? checkApa : CruiseApaTarget();
                bool hyperbolic = (apaSigned < 0) || double.IsInfinity(apa) || (apa > 1e8);
                // High-TWR/low-g: hard-dump once Ap > ~1.15x cruise (was 2x; soft-hold let Ap runaway).
                if (hyperbolic || (targetApa > 0 && apa > ApStripFactor() * targetApa))
                {
                    // 203350: mountain ASL/Ap already > cruise at AGL~2; dumping V
                    // into the pad with hop H-strip left thr=0 forever. Clear AGL
                    // first (gravity-scaled loft), then dump while building H.
                    if (VesselState.altitudeTrue < ASCENT_AOA_MIN_ALT)
                    {
                        // Need V > g*2*28 for vacuum pitch-over latch; 8*g alone is ~2 on Vesta.
                        double gClear = Math.Max(Core.Landing.g, 0.05);
                        double tiltGate = gClear * 2.0 * 28.0 + 1.0;
                        double clearCap = Math.Max(3.0, Math.Min(25.0, Math.Max(8.0 * gClear, tiltGate)));
                        desiredVerticalSpeed = clearCap;
                        LogDownrangeH("setVerticalSpeed: vacuum Ap high but AGL low, clear pad before dump ApA="
                            + apaSigned.ToString("F0") + " target=" + targetApa.ToString("F0")
                            + " AGL=" + VesselState.altitudeTrue.ToString("F1")
                            + " desV=" + desiredVerticalSpeed.ToString("F1"));
                    }
                    else
                    {
                        double overshoot = hyperbolic ? 15.0 : Math.Min(40.0, apa / Math.Max(targetApa, 1.0));
                        desiredVerticalSpeed = -Math.Max(5.0, Math.Min(25.0, 0.35 * Math.Abs(VesselState.speedVertical) + overshoot));
                        LogDownrangeH("setVerticalSpeed: vacuum Ap overshot/hyperbolic, dump V while building H ApA="
                            + apaSigned.ToString("F0") + " target=" + targetApa.ToString("F0")
                            + " H=" + VesselState.speedSurfaceHorizontal.ToString("F1")
                            + " desV=" + desiredVerticalSpeed.ToString("F1"));
                    }
                }
                else
                {
                    // F1 (Ricardo Nov 10, impact_ap_dump_spread.txt): proportional Ap hold.
                    // Fixed -2 m/s with vz ~40 m/s was a saturated full-throttle dump that
                    // bang-banged against the 50.7 m/s climb branch at ApA == cruise (handoff ApA
                    // quantized 45225..45483). Command the vz that coasts exactly to cruise + bias:
                    // ApA - h = vz^2/(2 g_eff)  ->  vzHold = vz * sqrt((apHold - h)/(ApA - h)) (g_eff cancels).
                    // Fallback -2 (previous value) when vz < 3 m/s or inputs are degenerate.
                    const double AP_HOLD_BIAS_M = 250.0;
                    double apHold = targetApa + AP_HOLD_BIAS_M;
                    double hNow = VesselState.altitudeASL;
                    double vzNow = VesselState.speedVertical;
                    double holdV = -2.0;
                    if ((vzNow > 3.0) && ((apa - hNow) > 50.0) && (apHold > hNow))
                    {
                        double v = vzNow * Math.Sqrt((apHold - hNow) / (apa - hNow));
                        if (!double.IsNaN(v) && !double.IsInfinity(v))
                            holdV = v;
                    }
                    desiredVerticalSpeed = holdV;
                    LogDownrangeH("setVerticalSpeed: vacuum Ap ready, pitch for H toward pad ApA="
                        + apaSigned.ToString("F0") + " target=" + targetApa.ToString("F0")
                        + " H=" + VesselState.speedSurfaceHorizontal.ToString("F1")
                        + " apHold=" + apHold.ToString("F0") + " vz=" + vzNow.ToString("F1")
                        + " desV=" + desiredVerticalSpeed.ToString("F1"));
                }
            }

            /// <summary>
            /// Vacuum far hop may coast/FineTune only with finite Ap near the design cruise.
            /// Hyperbolic / multi-Mm Ap after a vertical runaway must keep dumping + building H.
            /// </summary>
            private bool ApaSaneForVacuumCoast(double checkApa)
            {
                double apa = VesselState.orbitApA;
                if (apa < 0 || double.IsNaN(apa) || double.IsInfinity(apa))
                    return false;
                double targetApa = (checkApa > 0) ? checkApa : CruiseApaTarget();
                if (targetApa <= 0)
                    return apa >= 0;
                return apa <= 3.0 * targetApa;
            }

            public MoveToTarget(MechJebCore core) : base(core)
            {
                lastDesiredThrustVector = Vector3d.zero;
                velocityPID = new LandingFramePID();
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
                    // Ricardo (Nov 9): if the hoop must rise for terrain, the only better option
                    // is a higher initial climb so TSO starts shaping above the ridge. Only raises.
                    try
                    {
                        // k > 1 (trip cruise): climb to max(ridge ahead + 2.5 km, pad + 4.2 km) instead
                        // of the k == 1.0 coast floor (ridge + 900 + gate). k == 1.0: unchanged call.
                        double terrainApa = TripOn ? Core.Landing.TripCruiseFloorAsl() : Core.Landing.AscentApaFloorForTerrain();
                        if (terrainApa > subOrbitalApA)
                        {
                            Debug.Log("MoveToTarget: " + (TripOn ? "trip cruise floor " : "terrain Ap floor ") + terrainApa.ToString("F0")
                                + " > base " + subOrbitalApA.ToString("F0"));
                            subOrbitalApA = terrainApa;
                        }
                    }
                    catch (System.Exception e)
                    {
                        Debug.Log("MoveToTarget: terrain Ap floor failed: " + e.Message);
                    }
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

            // Unified 3D velocity PID (along-track, vertical, lateral). Replaces the two
            // independent pidVelocity_h / pidVelocity_v loops that mixed at setThrustVector
            // and fought each other. Old axis-independent controller: MoveToTargetAxisIndependent.
            private const double DERIV_ALPHA = 0.75; // same filter as the working horizontal D term

            /// <summary>
            /// Piecewise error-magnitude gain used by the working per-axis PIDs.
            /// Inner deadband (debug3/4), mid ramp (debug5/6), outer ramp (debug7/8).
            /// </summary>
            private static double ScheduledGain(double absError, double d3, double d4, double d5, double d6, double d7, double d8)
            {
                if (absError < d3) return d4;
                if (absError < d5) return d4 + d6 * Mathf.Clamp01((float)((absError - d3) / Math.Max(1e-9, d5 - d3)));
                return d4 + d6 + d8 * Mathf.Clamp01((float)((absError - d5) / Math.Max(1e-9, d7 - d5)));
            }

            private double VerticalKp()
            {
                return vFrac * Core.Landing.debug1 * Math.Max(20, Math.Min(50, 20 + (50 - 20) * (Core.Landing.g - 0.05) / (2 - 0.05)));
            }

            private double VerticalKi()
            {
                return vFrac * Core.Landing.debug2 * Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));
            }

            private double HorizontalKp()
            {
                return hFrac * Core.Landing.debug1h * Math.Max(15, Math.Min(50, 10 + (50 - 10) * (Core.Landing.g - 0.05) / (2 - 0.05)));
            }

            private double HorizontalKi()
            {
                return hFrac * Core.Landing.debug2h * Math.Max(0.05, Math.Min(0.17, 0.05 + (0.17 - 0.05) * (Core.Landing.g - 0.05) / (2 - 0.05)));
            }

            /// <summary>
            /// Orthonormal landing frame: X along-track (horizontal toward target), Y up, Z lateral.
            /// </summary>
            private void BuildLandingFrame(Vector3d desiredHorizontalVel, Vector3d hError, out Vector3d hDir, out Vector3d up, out Vector3d latDir)
            {
                up = Vessel.up;
                // Near the pad desiredHorizontalVel is tiny and noisy. Prefer the geometric
                // target direction so the H/L frame does not chatter and re-fight itself.
                Vector3d towardTarget = Vector3d.Exclude(up, Core.Landing.targetingResult.horizontalVec);
                if (desiredHorizontalVel.sqrMagnitude >= 0.25)
                    hDir = Vector3d.Exclude(up, desiredHorizontalVel);
                else if (towardTarget.sqrMagnitude > 1e-12)
                    hDir = towardTarget;
                else
                    hDir = Vector3d.Exclude(up, desiredHorizontalVel);
                if (hDir.sqrMagnitude < 1e-12)
                    hDir = Vector3d.Exclude(up, hError);
                if (hDir.sqrMagnitude < 1e-12)
                    hDir = towardTarget;
                if (hDir.sqrMagnitude < 1e-12)
                    hDir = Vector3d.Exclude(up, VesselState.forward);
                if (hDir.sqrMagnitude < 1e-12)
                    hDir = Vector3d.Exclude(up, new Vector3d(1, 0, 0));
                if (hDir.sqrMagnitude < 1e-12)
                    hDir = new Vector3d(1, 0, 0);
                hDir = hDir.normalized;

                latDir = Vector3d.Cross(up, hDir);
                if (latDir.sqrMagnitude < 1e-12)
                    latDir = Vector3d.Cross(up, VesselState.forward);
                if (latDir.sqrMagnitude < 1e-12)
                    latDir = Vector3d.Cross(up, new Vector3d(1, 0, 0));
                if (latDir.sqrMagnitude < 1e-12)
                    latDir = new Vector3d(0, 0, 1);
                latDir = latDir.normalized;

                hDir = Vector3d.Cross(latDir, up);
                if (hDir.sqrMagnitude < 1e-12)
                    hDir = Vector3d.Exclude(up, new Vector3d(1, 0, 0)).normalized;
                else
                    hDir = hDir.normalized;
            }

            /// <summary>
            /// Unified 3D PID: one acceleration vector, then attitude + throttle from that vector.
            /// Per-axis gains/schedules are the working v* and h* knobs; lateral uses debug10/debug12.
            /// Coupling: gravity is a vertical feedforward (tilt raises throttle automatically),
            /// upright cone (debug93) is a projection of the 3D command, saturation scales the whole
            /// vector, and anti-windup bleeds all three integrals together.
            /// </summary>
            public void setThrustVector(ref double hTargetError, ref Vector3d desiredThrustVector, ref Vector3d desiredHorizontalVel, Vector3d hError, double hCorrectionAngle, ref double desiredVerticalSpeed, double vError)
            {
                hFrac = vFrac = (hTargetError < hFarLimit) ? 1 : Core.Landing.debug94;

                Vector3d up, hDir, latDir;
                BuildLandingFrame(desiredHorizontalVel, hError, out hDir, out up, out latDir);

                Vector3d hSurface = Vector3d.Exclude(up, VesselState.surfaceVelocity);

                // Along-track hysteresis: freeze the frame when |eH| is inside the innermost H band
                // so attitude does not chatter around the target (same debug3h deadband as before).
                // Near the pad BuildLandingFrame already picked targetingResult (desired H vel
                // is tiny). Freezing a heading captured on a noisy desired-vel sample would
                // override that toward-target frame and re-fight H/L. Keep hysteresis for the
                // far / noisy-eH chatter case it was meant for.
                Vector3d towardTarget = Vector3d.Exclude(up, Core.Landing.targetingResult.horizontalVec);
                bool usedTowardTarget = (desiredHorizontalVel.sqrMagnitude < 0.25) && (towardTarget.sqrMagnitude > 1e-12);
                double eHProbe = Vector3d.Dot(desiredHorizontalVel - hSurface, hDir);
                if (Math.Abs(eHProbe) < Core.Landing.debug3h)
                {
                    if (usedTowardTarget)
                    {
                        if (hHysteresis && towardTarget.sqrMagnitude > 1e-12 && hHysteresisVector.sqrMagnitude > 1e-12)
                        {
                            Vector3d frozenAlign = Vector3d.Exclude(up, hHysteresisVector);
                            if (frozenAlign.sqrMagnitude > 1e-12 && Vector3d.Dot(frozenAlign.normalized, towardTarget.normalized) < 0.5)
                                hHysteresis = false;
                        }
                    }
                    else
                    {
                        if (hHysteresis == false)
                        {
                            hHysteresis = true;
                            hHysteresisVector = hDir;
                        }
                        if (hHysteresisVector.sqrMagnitude > 1e-12)
                        {
                            Vector3d frozen = Vector3d.Exclude(up, hHysteresisVector);
                            if (frozen.sqrMagnitude > 1e-12)
                            {
                                hDir = frozen.normalized;
                                latDir = Vector3d.Cross(up, hDir);
                                if (latDir.sqrMagnitude > 1e-12)
                                    latDir = latDir.normalized;
                            }
                        }
                    }
                }
                else
                {
                    hHysteresis = false;
                }

                // If the landing frame swung hard (overshoot / pad chatter), old H/L integrals and
                // the world-frame horizontal average point the wrong way and fight the new axes.
                // InvalidateHorizontal also drops the D sample history so the next tick does not
                // invent a derivative from the pre-swing frame axes.
                if (hasPrevLandingFrame)
                {
                    double alignH = Vector3d.Dot(hDir, prevHDir);
                    double alignL = Vector3d.Dot(latDir, prevLatDir);
                    if (alignH < 0.2 || alignL < 0.2)
                    {
                        velocityPID.InvalidateHorizontal();
                        horizontalAvg = Vector3d.zero;
                        aHAvg = 0;
                        aLAvg = 0;
                    }
                }
                prevHDir = hDir;
                prevLatDir = latDir;
                hasPrevLandingFrame = true;

                double eH = Vector3d.Dot(desiredHorizontalVel - hSurface, hDir);
                double eV = vError;
                double eL = -Vector3d.Dot(hSurface, latDir);
                // Collision hop is vertical-first. Scale H/L errors with (1-hopBlend)
                // so I/D do not wind a full kill-H command while V is climbing, and
                // so the strip drop fades instead of a boolean cliff.
                if (hopBlend > 0)
                {
                    double hopLive = 1.0 - hopBlend;
                    eH *= hopLive;
                    eL *= hopLive;
                }

                // High-g: soften only when the speed profile itself is climbing and we are
                // faster-up than commanded. During descent (desiredVerticalSpeed <= 0) this
                // 0.005 scale left aV stuck near +g, so the vessel climbed through a -0.5 m/s
                // target even with a working v* loop.
                if (Core.Landing.g > Core.Landing.LOW_GRAVITY && desiredVerticalSpeed > 0 && eV < 0)
                    eV *= 0.005;

                Vector3d error = new Vector3d(eH, eV, eL);
                double dt = Time.fixedDeltaTime;
                if (dt < 1e-6) dt = 0.02;

                // Pass world H surface + current frame axes so D is re-projected each tick
                // instead of treating a rotating landing frame as a velocity jump.
                velocityPID.Accumulate(error, hSurface, VesselState.speedVertical, hDir, latDir, dt, DERIV_ALPHA, Core.Landing.debug92, Core.Landing.debug91);

                double gainV = ScheduledGain(Math.Abs(eV),
                    Core.Landing.debug3, Core.Landing.debug4, Core.Landing.debug5, Core.Landing.debug6, Core.Landing.debug7, Core.Landing.debug8);
                double gainH = ScheduledGain(Math.Abs(eH),
                    Core.Landing.debug3h, Core.Landing.debug4h, Core.Landing.debug5h, Core.Landing.debug6h, Core.Landing.debug7h, Core.Landing.debug8h);
                // Schedule L on |eL|, not |eH|. Reusing gainH zeroed lateral in the H
                // inner deadband, which is exactly when we still need to kill cross-track.
                double gainL = ScheduledGain(Math.Abs(eL),
                    Core.Landing.debug3h, Core.Landing.debug4h, Core.Landing.debug5h, Core.Landing.debug6h, Core.Landing.debug7h, Core.Landing.debug8h);

                double kpV = VerticalKp();
                double kiV = VerticalKi();
                double kpH = HorizontalKp();
                double kiH = HorizontalKi();

                // D-on-measurement, along-track and lateral. Vertical has no D (working pidVelocity_v).
                double dH = -Core.Landing.debug13 * velocityPID.FilteredDeriv.x;
                double dL = -Core.Landing.debug13 * velocityPID.FilteredDeriv.z;

                // Hover feedforward on +up so a tilt raises |a| and the vertical component stays ~g.
                // Arm it only while the vertical loop is live: zero v* knobs must not keep thrusting +g
                // (that was the upward bias with debug=0).
                bool verticalLoopOn = Math.Abs(gainV) > 1e-18 && (Math.Abs(kpV) > 1e-18 || Math.Abs(kiV) > 1e-18);
                double gComp = verticalLoopOn ? VesselState.localg : 0.0;

                double aV = gComp + gainV * (kpV * eV + kiV * velocityPID.Integral.y);
                double aH = gainH * (kpH * eH + kiH * velocityPID.Integral.x + dH);

                // Lateral is a first-class axis instead of being subtracted into the H error.
                // P scale matches the old injection: debug12 * gainCancelHVelocity * (H KP).
                double gainCancelHVelocity = (hTargetError <= hCloseLimit) ? baseGain : baseGain / 13.0;
                double kpL = kpH * Core.Landing.debug12 * gainCancelHVelocity;
                double kiL = kiH * 0.25 * Core.Landing.debug12;
                double aL = gainL * (kpL * eL + kiL * velocityPID.Integral.z + dL);

                double aLimit = Math.Max(1e-6, actualLimitedMaxThrustAccel);
                // debug10 still limits L vs H, but never below 20% of available accel.
                // Capping against max(|aH|, 1e-6) froze L whenever H was settled and left
                // the pad meander the old mix was supposed to kill.
                double aHAbs = Math.Abs(aH);
                double aLMax = Math.Max(Core.Landing.debug10 * aHAbs, 0.2 * aLimit);
                if (Math.Abs(aL) > aLMax)
                    aL = Math.Sign(aL) * aLMax;
                aV = Math.Max(-aLimit, Math.Min(aLimit, aV));
                aH = Math.Max(-aLimit, Math.Min(aLimit, aH));
                aL = Math.Max(-aLimit, Math.Min(aLimit, aL));

                // A 200-1500 m terrain hop puts desiredVerticalSpeed > 0 (ASCENDING), so
                // the debug93 upright cone is off and the 100 m initial-ascent H-zero
                // does not apply. hopBlend 1 zeros H/L; as remaining V drops through
                // HOP_STRIP_BOOST it fades H/L back in instead of slamming aH.
                // Do not leave aHAvg/aLAvg stuck at 0 after the fade (debug96 near
                // zero already means no smoothing).
                if (hopBlend >= 1)
                {
                    aH = 0;
                    aL = 0;
                    aHAvg = 0;
                    aLAvg = 0;
                    if (LOG_LANDING_TICK)
                        Debug.Log("collision hop: zero H/L");
                }
                else if (hopBlend > 0)
                {
                    double hopLive = 1.0 - hopBlend;
                    aH *= hopLive;
                    aL *= hopLive;
                    if (LOG_LANDING_TICK)
                        Debug.Log("collision hop: blend=" + hopBlend.ToString("F2"));
                }

                // Final settle (<=~200 mm): fade H/L accel authority so attitude stays near
                // vertical. Blends with the setHorizontalSpeed settle (0.2..0.80 m). Tiny
                // residual authority allows pad micro-adjustments without lean overcompensation.
                const double finalSettleM = 0.2;
                const double nearPadAggressiveM = 0.80;
                double settleAuthority = 1.0;
                if (hTargetError <= finalSettleM)
                    settleAuthority = 0.05;
                else if (hTargetError < nearPadAggressiveM)
                    settleAuthority = 0.05 + 0.95 * (hTargetError - finalSettleM) / (nearPadAggressiveM - finalSettleM);
                if (settleAuthority < 1.0)
                {
                    aH *= settleAuthority;
                    aL *= settleAuthority;
                }

                // Smooth H/L in the landing frame, then rebuild world accel. Averaging the world
                // horizontal vector (old path) smeared yesterday's H into today's L whenever hDir
                // rotated ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â leftover axis fighting after the PID itself was decoupled.
                // debug96 == 0 must not freeze aHAvg/aLAvg at 0 after a frame reset (that looked
                // like dead H/L knobs). Treat a near-zero blend as "no smoothing".
                double blend = Core.Landing.debug96;
                if (blend < 1e-9)
                {
                    aHAvg = aH;
                    aLAvg = aL;
                }
                else
                {
                    aHAvg = aHAvg * (1 - blend) + blend * aH;
                    aLAvg = aLAvg * (1 - blend) + blend * aL;
                }
                aH = aHAvg;
                aL = aLAvg;
                Vector3d accel = aH * hDir + aV * up + aL * latDir;
                horizontalAvg = aH * hDir + aL * latDir;

                bool upright = false;
                double vMinAngle;
                double vMaxAngle;

                if (Core.Landing.g > Core.Landing.LOW_GRAVITY)
                {
                    throttleAngle = 30;

                    if (desiredVerticalSpeed > 0)
                    {
                        if (VesselState.altitudeTrue >= 100)
                        {
                            vMinAngle = 0.2 * Math.Abs(aV);
                            vMaxAngle = +Math.Abs(aV);
                            hCorrectionAngle = 0;
                        }
                        else
                        {
                            vMinAngle = 0.2;
                            vMaxAngle = 1.0;
                        }
                        if (LOG_LANDING_TICK)
                            Debug.Log("ascending " + vMinAngle.ToString("F2") + " " + vMaxAngle.ToString("F2"));
                    }
                    else
                    {
                        upright = true;
                        if (VesselState.altitudeTrue < 5000)
                        {
                            vMinAngle = 0.05;
                            vMaxAngle = 0.15;
                        }
                        else
                        {
                            vMinAngle = 0;
                            vMaxAngle = 1.3;
                        }
                        if (LOG_LANDING_TICK)
                            Debug.Log("descending " + vMinAngle.ToString("F2") + " " + vMaxAngle.ToString("F2"));
                    }
                }
                else
                {
                    if (desiredVerticalSpeed >= 0)
                    {
                        if (VesselState.altitudeTrue >= 100)
                        {
                            vMinAngle = 0.2 * Math.Abs(aV);
                            vMaxAngle = +Math.Abs(aV);
                            hCorrectionAngle = 0;
                        }
                        else
                        {
                            vMinAngle = 0.15;
                            vMaxAngle = 1.0;
                        }
                        if (LOG_LANDING_TICK)
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
                            vMinAngle = -Math.Abs(aV);
                            vMaxAngle = +Math.Abs(aV);
                            throttleAngle = 30;
                        }
                        if (LOG_LANDING_TICK)
                            Debug.Log("lg descending " + vMinAngle.ToString("F2") + " " + vMaxAngle.ToString("F2"));
                    }
                }

                // Keep the old verticalSign filter for logging / envelope, but do NOT replace
                // vertical acceleration with it. In the axis-independent controller verticalSign
                // was a direction mix weight (0.05..0.15), not m/s^2 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â using it as accel would
                // drop lift below hover. Tilt is constrained below via the debug93 cone.
                double vCmd = Vector3d.Dot(accel, up);
                verticalSign = System.Math.Min(vMaxAngle, System.Math.Max(vMinAngle, verticalSign * (1 - Core.Landing.debug95) + Core.Landing.debug95 * vError * Math.Abs(vCmd)));

                Vector3d horizNow = accel - up * vCmd;

                // Upright cone: limit horizontal accel to debug93 * |vertical|. Projection, not a
                // second loop ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â this is the knob that stopped the Moon 45-degree tip-over.
                if ((upright == true) && (Math.Abs(vCmd) > 0.0))
                {
                    double maxHoriz = Core.Landing.debug93 * Math.Abs(vCmd);
                    // Tighten cone toward vertical in the same settle band as setHorizontalSpeed.
                    // At <=200 mm maxHoriz -> 0 (no lean); blend out by 0.80 m.
                    if (hTargetError < nearPadAggressiveM)
                    {
                        double coneScale = (hTargetError <= finalSettleM) ? 0.0
                            : (hTargetError - finalSettleM) / (nearPadAggressiveM - finalSettleM);
                        maxHoriz *= coneScale;
                    }
                    if (horizNow.magnitude > maxHoriz)
                    {
                        double hratio = (horizNow.magnitude > 1e-12) ? maxHoriz / horizNow.magnitude : 0;
                        horizNow *= hratio;
                    }
                    accel = horizNow + up * vCmd;
                    if (LOG_LANDING_TICK)
                        Debug.Log("hadjust " + horizNow.magnitude.ToString("F3") + "," + vCmd.ToString("F3"));
                }

                // Do not cap |aH| with hCorrectionAngle. That leftover mix weight is baseGain*0.4*|herror|,
                // not tan(tilt) in m/s^2. Near the target it crushed the H PID to ~0 and left the
                // ZEM/ZEV residual ~0.7 m/s meander. Tilt limit is the debug93 cone above.

                // Initial ascent: vertical only until 100 m.
                if ((desiredVerticalSpeed > 0) && (VesselState.altitudeTrue <= 100))
                {
                    accel = up * Math.Max(0, Vector3d.Dot(accel, up));
                    horizontalAvg = Vector3d.zero;
                    aHAvg = 0;
                    aLAvg = 0;
                    if (LOG_LANDING_TICK)
                        Debug.Log("initial ascension");
                }
                // Old mixer zeroed H *throttle* when >50 deg off, but kept H in the attitude vector so
                // the craft would still slew toward the target. Stripping H from accel here
                // pointed attitude at +up forever after a vertical ZEM/ZEV handoff (forward ~ up,
                // H command ~ 90 deg) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â no steering. Drive() already cuts throttle beyond
                // throttleAngle (30 deg).

                // Rockets cannot pull. Allow a downward world component only for low-g pitch-over.
                // Far cruise commands desired V = 0 (steepness ratio). On Phobos-class g, that used
                // to strip downward accel while leftover +V was already above escape, so H tracked
                // but V never dumped and intercept math saw Inf apoapsis. Hyperbolic transit is
                // allowed; pitch over whenever we still need to reduce V, not only when desired < 0.
                // Collision hop wins while armed: remainingBoost decays under actual +V and the
                // dump was pitching over into the hop. Between hops, dump may still bleed leftover.
                bool hopArmed = (hopBlend > 0) || (vCollisionCounter > 0);
                // Soft dump: fade pitch-over authority over DUMP_V_* instead of a hard
                // on/off at desired+2 (that pulsed the vertical vector around +2 m/s).
                // Hop armed still suppresses dump into terrain.
                double dumpBlend = 0.0;
                if (!hopArmed && (desiredVerticalSpeed >= 0))
                {
                    double excessV = VesselState.speedVertical - desiredVerticalSpeed;
                    if (excessV > DUMP_V_START_EXCESS)
                    {
                        double span = DUMP_V_FULL_EXCESS - DUMP_V_START_EXCESS;
                        dumpBlend = (span > 1e-6) ? Math.Min(1.0, (excessV - DUMP_V_START_EXCESS) / span) : 1.0;
                    }
                }
                bool vacuumWaitingH = (MainBody.atmosphere == false)
                    && (Core.Landing.increaseVertical == true)
                    && !HasMeaningfulDownrangeTowardTarget()
                    && ((VesselState.orbitApA < 0) || (Math.Abs(VesselState.orbitApA) >= CruiseApaTarget()));
                bool allowPitchOver = (Core.Landing.g <= Core.Landing.LOW_GRAVITY) && (VesselState.altitudeTrue >= 30)
                    && !hopArmed
                    && ((desiredVerticalSpeed < 0) || (dumpBlend > 0) || vacuumWaitingH);
                double avFinal = Vector3d.Dot(accel, up);
                if (avFinal < 0)
                {
                    if (allowPitchOver == false)
                        accel -= up * avFinal;
                    else if ((desiredVerticalSpeed >= 0) && (dumpBlend < 1.0))
                        accel -= up * avFinal * (1.0 - dumpBlend);
                }

                Vector3d rawAccel = accel;
                double maxA = Math.Max(1e-6, VesselState.limitedMaxThrustAccel);
                if (accel.magnitude > maxA)
                    accel *= maxA / accel.magnitude;

velocityPID.AntiWindup(rawAccel, accel);

                // 192439 re-boost: HoldAp desV=-25 then PID chased undershoot with thr~0.85
                // pitch+89, pumping Ap 5->26 Mm. While vacuum Ap > 1.15x cruise, strip upward
                // accel (terrain hop still uses collision path before this). Cap thr by g/a_max.
                // Only during loft/ascent (increaseVertical or pre-FineTune). Near-pad
                // suicide must keep full thr even if Ap still >1.15x cruise (195311 crash
                // thr~0.07 into AGL with V=-84 after TSO->MTT).
                bool loftPhaseAp = Core.Landing.increaseVertical
                    || ((errStep > Ascend2Target.Disabled) && (errStep < Ascend2Target.FineTune));
                bool vacuumApOver = loftPhaseAp && (MainBody.atmosphere == false)
                    && ((VesselState.orbitApA < 0)
                        || (Math.Abs(VesselState.orbitApA) > ApStripFactor() * Math.Max(1000.0, CruiseApaTarget())));
                // 195311: departure-mountain hop set hopArmed/collisionHopActive and skipped
                // this strip while HoldAp desV=-25 -> thr=0.85 pitch90 lofted Ap 33k->774k.
                // Ap already >1.15x cruise: always strip upward / thrCap (hop must not mint Mm Ap).
                // Genuine low-AGL hop still runs in move() only when Ap not already high.
                if (vacuumApOver)
                {
                    // 201838: full up-strip at AGL~2 left pitch~0 thrCap-only H slide on
                    // mountain -> false landed at 22s. Allow ~3*g up until AGL clears
                    // ASCENT_AOA_MIN_ALT so liftoff works; then strip excess loft.
                    double upDotAp = Vector3d.Dot(accel, up);
                    double upAllow = (VesselState.altitudeTrue < ASCENT_AOA_MIN_ALT)
                        ? (3.0 * Math.Max(Core.Landing.g, 0.05))
                        : 0.0;
                    if (upDotAp > upAllow)
                        accel -= up * (upDotAp - upAllow);
                }

                desiredThrustVector = (accel.sqrMagnitude > 1e-12) ? accel : up;

                Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH = false;
                attitudeAngleFromThrust = Core.Attitude.attitudeAngleFromTarget();
                float throttle = (float)(accel.magnitude / Math.Max(VesselState.maxThrustAccel, 1e-6));
                if (vacuumApOver)
                {
                    double thrCapAp = Math.Max(0.03, Math.Min(1.0,
                        8.0 * Math.Max(0.05, Core.Landing.g) / Math.Max(1e-3, VesselState.limitedMaxThrustAccel)));
                    if (throttle > thrCapAp)
                        throttle = (float)thrCapAp;
                }
                Core.Thrust.RequestActiveThrottle(throttle);

                Vector3d vPart = Vector3d.Dot(desiredThrustVector, up) * up;
                Vector3d hPart = desiredThrustVector - vPart;
                if (LOG_LANDING_TICK)
                {
                    Debug.Log("pid3d  eH:" + eH.ToString("F2") + " eV:" + eV.ToString("F2") + " eL:" + eL.ToString("F2")
                        + "  aH:" + aH.ToString("F3") + " aV:" + aV.ToString("F3") + " aL:" + aL.ToString("F3")
                        + "  t:" + throttle.ToString("F4"));
                    Debug.Log("vector  v:" + vPart.magnitude.ToString("F3") + "  h:" + hPart.magnitude.ToString("F3") + "  hCorrectionAngle:" + hCorrectionAngle.ToString("F3"));
                    Debug.Log("thrust  v:" + vPart.magnitude.ToString("F3") + "  h:" + hPart.magnitude.ToString("F3") + "  hCorrectionAngle:" + hCorrectionAngle.ToString("F3"));
                }
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
                    if (TripOn && TripCaQuiet && TripLowGFarHop(hTargetError))
                        TripCapClimbH(ref desiredHorizontalVel, ref herror); // k>1 climb, 1.2x, unchanged
                    if (TripCaQuiet && TripLowGFarHop(hTargetError)
                        && ((Math.Abs(VesselState.orbitApA) >= CruiseApaTarget())
                            || (VesselState.altitudeASL >= CruiseApaTarget())))
                        CapHDuringApHold(ref desiredHorizontalVel, ref herror);
                }

                // Hop-phase collision: LOCAL rim/terrain vs our ASL.
                // Arm/hold/re-arm from PredictAvoid only -- never from distance to the
                // landing site. hTargetError <= 5 km is pad false-fire protection only
                // (predictor AVOID_MIN_ALTITUDE on the pad; near-pad hop is suppressed). Departure-crater climb-out
                // with a far target must still hop on local rim clearance.
                // An in-progress tail keeps decaying after we later cross 5 km; cutting
                // it off restored H mid-hop. The 30s ignore is load-bearing for hops
                // off the ground near the pad only.
                collisionHopActive = false;
                hopBlend = 0;
                bool nearLandingSite = hTargetError <= 5000;
                if (vIgnoreCollision > 0 && nearLandingSite)
                {
                    vIgnoreCollision--;
                }
                else
                {
                    if (vIgnoreCollision > 0)
                        vIgnoreCollision = 0;
                    double deltaV = 0;
                    bool willCrash = false;
                    // Pad suppress only. Away from the pad always sample local terrain
                    // so a far landing site cannot skip departure-crater climb-out.
                    if (nearLandingSite == false)
                    {
                        (deltaV, willCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, AVOID_MIN_ALTITUDE);
                    }

                    // 195311: on mountain ASL~33km already >= cruise; PredictAvoid armed hop
                    // and boosted desV after HoldAp dump -> vertical loft. Skip hop V-boost
                    // when vacuum Ap/ASL already at design cruise (terrain avoid kept when
                    // Ap not yet high).
                    double cruiseHop = CruiseApaTarget();
                    // 203350: do NOT require AGL>ASCENT_AOA_MIN_ALT. Mountain pad
                    // has ASL/Ap already >= cruise at AGL~2; gating on AGL re-armed
                    // hop H-zero under Ap-dump and stuck thr=0. Skip hop loft whenever
                    // Ap/ASL already at design cruise (rim already below cruise ASL).
                    bool apAlreadyHighForHop = (MainBody.atmosphere == false)
                        && ((Math.Abs(VesselState.orbitApA) > 1.15 * Math.Max(1000.0, cruiseHop))
                            || (VesselState.altitudeASL >= cruiseHop));
                    double newRequired = (deltaV > 0) ? deltaV : ((willCrash == true) ? 500.0 : 0.0);
                    if (apAlreadyHighForHop)
                        newRequired = 0.0;
                    double remainingBoost = (vCollisionCounter > 0)
                        ? vCollisionDeltaV * ((double)vCollisionCounter / COLLISION_TAIL)
                        : 0.0;
                    bool hopInProgress = vCollisionCounter > 0;
                    // Same hot prediction must not reset COLLISION_TAIL every tick
                    // (collision3 never ran, H stayed stripped). Re-arm only for a
                    // fresh hop or a real new obstacle: new dV well above remaining
                    // boost AND above the already-armed peak.
                    bool substantialUpgrade = hopInProgress
                        && newRequired > remainingBoost * HOP_REARM_RATIO
                        && newRequired > vCollisionDeltaV;

                    if (newRequired > 0 && (hopInProgress == false || substantialUpgrade))
                    {
                        // Fresh hop or real upgrade: invalidate H/L I/D. Do not
                        // InvalidateHorizontal on a skipped re-fire.
                        velocityPID.InvalidateHorizontal();
                        vCollisionDeltaV = Math.Max(vCollisionDeltaV, newRequired);
                        desiredVerticalSpeed += vCollisionDeltaV;
                        vError = desiredVerticalSpeed - VesselState.speedVertical;
                        // Hop is vertical-first. Zeroing only desiredHorizontalVel still
                        // leaves a large eH that the ASCENDING mixer will fight V with.
                        // hopBlend + aH/aL scale in setThrustVector is the 3D coupling.
                        desiredHorizontalVel = Vector3d.zero;
                        herror = desiredHorizontalVel - VesselState.horizontalSurface;
                        vCollisionCounter = COLLISION_TAIL;
                        hopBlend = 1;
                        collisionHopActive = true;
                        if (LOG_LANDING_TICK)
                        {
                            if (deltaV > 0)
                                Debug.Log("collision1: " + vError.ToString("F2"));
                            else
                                Debug.Log("collision2: " + vError.ToString("F2") + "  " + herror.magnitude.ToString("F2"));
                        }
                    }
                    else if (vCollisionCounter > 0)
                    {
                        remainingBoost = vCollisionDeltaV * ((double)vCollisionCounter / COLLISION_TAIL);
                        // Live remaining climb must win over a decaying tail. Otherwise
                        // hopBlend fades, leftover-V dump pitches into the rim, and the
                        // next hop never fully recovers (Vesta crater pulse-down).
                        double vBoost = remainingBoost;
                        if (newRequired > vBoost) vBoost = newRequired;
                        desiredVerticalSpeed += vBoost;
                        vError = desiredVerticalSpeed - VesselState.speedVertical;
                        // PID still wants to kill H (desired H = 0). Keep the hard H/L
                        // strip while remaining V boost is large, not merely the first
                        // half of the clock (collision2's 500 m/s floor left 250 m/s of
                        // V at half-tail). Fade H back in as remaining drops through
                        // HOP_STRIP_BOOST (~20 m/s) - but not while a rim is still in
                        // the climb-out path.
                        desiredHorizontalVel = Vector3d.zero;
                        herror = desiredHorizontalVel - VesselState.horizontalSurface;
                        if (newRequired > 0)
                            hopBlend = 1;
                        else
                            hopBlend = Math.Max(0.0, Math.Min(1.0, remainingBoost / HOP_STRIP_BOOST));
                        collisionHopActive = hopBlend >= 1;
                        vCollisionCounter--;
                        if (vCollisionCounter <= 0 && newRequired > 0)
                        {
                            // Re-arm without a dump-V gap. Do not InvalidateHorizontal
                            // (that is for a fresh obstacle); this is the same rim.
                            vCollisionCounter = COLLISION_TAIL;
                            vCollisionDeltaV = Math.Max(vCollisionDeltaV, newRequired);
                            hopBlend = 1;
                            collisionHopActive = true;
                        }
                        if (LOG_LANDING_TICK)
                            Debug.Log("collision3: " + vError.ToString("F2") + "  " + vCollisionCounter.ToString("F0") + "  hop=" + collisionHopActive + " blend=" + hopBlend.ToString("F2"));
                    }
                    else
                    {
                        vCollisionDeltaV = 0;
                    }
                }

                // Re-assert Ap dump after hop may have added +desV (195311). Loft phase only.
                // 203350: skip while AGL still below clear - dump-through-pad + hop
                // H-strip produced thr=0 stuck on mountain.
                if (Core.Landing.increaseVertical
                    && (MainBody.atmosphere == false)
                    && (VesselState.altitudeTrue >= ASCENT_AOA_MIN_ALT)
                    && ((VesselState.orbitApA < 0)
                        || (Math.Abs(VesselState.orbitApA) > ApStripFactor() * Math.Max(1000.0, CruiseApaTarget()))))
                {
                    if (desiredVerticalSpeed > 0)
                        desiredVerticalSpeed = -Math.Max(5.0, Math.Min(25.0, 12.0 * Math.Max(Core.Landing.g, 0.05)));
                    else if (desiredVerticalSpeed > -5.0)
                        desiredVerticalSpeed = -Math.Max(5.0, Math.Min(25.0, 12.0 * Math.Max(Core.Landing.g, 0.05)));
                    vError = desiredVerticalSpeed - VesselState.speedVertical;
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

                    // TargetSubOrbital may have just advanced errStep to FineTune at Ap.
                    // Never downgrade that to Completed (low-g PeA rarely hits 9000).
                    if ((errStep >= Ascend2Target.FineTune) && (errStep < Ascend2Target.Completed))
                    {
                        trace += 'f';
                        Debug.Log("stopIncrease: keep FineTune from TargetSubOrbital Ap handoff");
                    }
                    // High-g far / high PeA: baseline FineTune (TargetSubOrbital may not have fired yet).
                    else if (((Core.Landing.g > Core.Landing.LOW_GRAVITY) && (hTargetError > Core.Landing.POST_TARGET_THRESHOLD))
                        || (VesselState.orbitPeA >= 9000) )
                    {
                        trace += 'f';
                        errStep = Ascend2Target.FineTune;
                    }
                    // Low-g Ground Track long hop: keep InitialBurn so TargetSubOrbital can
                    // suppressThrottle + warp-to-Ap, then errStep++ into FineTune. Do not Completed.
                    else if ((Core.Landing.LandingType == 1)
                        && (Core.Landing.g <= Core.Landing.LOW_GRAVITY)
                        && (hTargetError > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT))
                    {
                        if (errStep <= Ascend2Target.Disabled)
                            errStep = Ascend2Target.InitialBurn;
                        trace += 'A';
                        Debug.Log("stopIncrease: keep InitialBurn for low-g TargetSubOrbital Ap coast");
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
                        // Once Ap is high enough AND (atmosphere OR vacuum downrange H ready),
                        // stop commanding climb so TargetSubOrbital can suppressThrottle + warp-to-Ap.
                        // 192439 root: HoldAp waited on altitudeTrue>2000 AGL while mountain
                        // ASL/Ap already > cruise -> 14s vertical thr~0.85 lofted Ap to 4 Mm.
                        {
                            double checkApaHold = CruiseApaTarget();
                            bool apAlreadyAtCruise = (Math.Abs(VesselState.orbitApA) >= checkApaHold)
                                || (VesselState.altitudeASL >= checkApaHold);
                            bool apGate = (Math.Abs(VesselState.orbitApA - Vessel.terrainAltitude)
                                > Math.Min(1.2 * hTargetError, subOrbitalApA))
                                && (VesselState.altitudeTrue > ASCENT_AOA_MIN_ALT);
                            if (MainBody.atmosphere == false)
                            {
                                if (TripOn && TripCaQuiet && TripLowGFarHop(hTargetError))
                                {
                                    TripPoweredClimb(ref desiredVerticalSpeed); // Step 3 (k > 1 only)
                                }
                                else if (apAlreadyAtCruise || apGate)
                                {
                                    if (!HasMeaningfulDownrangeTowardTarget() || !ApaSaneForVacuumCoast(checkApaHold))
                                        HoldApWhileBuildingDownrangeH(ref desiredVerticalSpeed, checkApaHold);
                                    else
                                        desiredVerticalSpeed = 0;
                                }
                            }
                            else if (apGate)
                            {
                                desiredVerticalSpeed = 0;
                            }
                        }
                        ignoreLandingCounter = 0; // reset ignore landing count to allow start from a landing
                    }
                    else
                    {
                        // Low-g Ground Track long hop still in InitialBurn: ratio can improve
                        // before Ap is high enough. stopIncrease would clear increaseVertical and
                        // strand TargetSubOrbital (morning log: stopIncrease hCurrentError=0, never
                        // TargetSubOrbit). Keep climbing until Ap is ready for suppressThrottle.
                        bool lowGSubOrbitalClimb = (Core.Landing.LandingType == 1)
                            && (Core.Landing.g <= Core.Landing.LOW_GRAVITY)
                            && (hTargetError > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT)
                            && (errStep > Ascend2Target.Disabled)
                            && (errStep < Ascend2Target.FineTune)
                            && (suppressThrottle == 0);
                        if (lowGSubOrbitalClimb && TripOn && TripCaQuiet && (MainBody.atmosphere == false))
                        {
                            TripPoweredClimb(ref desiredVerticalSpeed); // Step 3 (k > 1 only)
                        }
                        else if (lowGSubOrbitalClimb)
                        {
                            double checkApaClimb = CruiseApaTarget();
                            // Same mountain-start fix: ASL/Ap at cruise beats altitudeTrue gate.
                            bool apClimbReady = (Math.Abs(VesselState.orbitApA) >= checkApaClimb)
                                || (VesselState.altitudeASL >= checkApaClimb)
                                || (VesselState.altitudeTrue > ASCENT_AOA_MIN_ALT
                                    && Math.Abs(VesselState.orbitApA) >= checkApaClimb);
                            if (apClimbReady)
                            {
                                if ((MainBody.atmosphere == false) && (!HasMeaningfulDownrangeTowardTarget() || !ApaSaneForVacuumCoast(checkApaClimb)))
                                {
                                    // Ap ready: hold/dump until H ready and Ap near cruise.
                                    HoldApWhileBuildingDownrangeH(ref desiredVerticalSpeed, checkApaClimb);
                                }
                                else
                                {
                                    desiredVerticalSpeed = 0; // Ap+H ready - TargetSubOrbital arms coast/warp
                                    Debug.Log("setVerticalSpeed: low-g Ap ready, hold climb for TargetSubOrbital coast ApA="
                                        + VesselState.orbitApA.ToString("F0") + " check=" + checkApaClimb.ToString("F0")
                                        + " H=" + VesselState.speedSurfaceHorizontal.ToString("F1"));
                                }
                            }
                            else
                            {
                                // Gravity-scale loft: cap climb desV (~12*g) so TWR cannot mint Mm Ap.
                                double climbCap = Math.Max(3.0, Math.Min(40.0, 12.0 * Math.Max(Core.Landing.g, 0.05)));
                                double climbCmd = (desiredVerticalSpeed > 0) ? desiredVerticalSpeed : climbCap;
                                desiredVerticalSpeed = Math.Max(3.0, Math.Min(climbCap, climbCmd));
                                // do NOT stopIncrease - TargetSubOrbital owns Ap handoff
                            }
                        }
                        else
                        {
                            desiredVerticalSpeed = 3.0; // give it a little boost to get going and avoid stalling at low altitude with low vertical speed.
                            if (VesselState.altitudeTrue > 15) stopIncrease = true; // Trigger STOP vertical increase
                        }
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
                // High-g far climb OR low-g Ground Track far climb: start TargetSubOrbital (InitialBurn).
                bool subOrbitalAscend = (Core.Landing.increaseVertical == true)
                    && (hTargetError > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT) && (ratio > 0.5 * maxRatio)
                    && ((Core.Landing.g > Core.Landing.LOW_GRAVITY)
                        || (Core.Landing.LandingType == 1)); // low-g: enable TargetSubOrbital Ap sequence
                if (subOrbitalAscend)
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
                        if (TripOn)
                            maxHorizontalSpeed *= TripK; // Step 2: H command scales with k (k > 1 only)

                        if (hCurrentError < 0.1 * hTargetError)
                        {
                            if (_subOrbitalApA == 0)
                            {
                                // Cap latch to ~2x baseline subOrbitalApA so a prior runaway Ap
                                // cannot become the cruise/coast target (vacuum far hops).
                                double apaLatch = Math.Abs(VesselState.orbitApA);
                                double apaCap = Math.Max(subOrbitalApA, 2.0 * subOrbitalApA);
                                _subOrbitalApA = Math.Min(apaLatch, apaCap);
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
                        // Keep hCurrentError while low-g TargetSubOrbital is still coasting to Ap
                        // (InitialBurn). Zeroing it would skip TargetSubOrbital and strand the hop.
                        if ((Core.Landing.LandingType == 1)
                            && (Core.Landing.g <= Core.Landing.LOW_GRAVITY)
                            && (errStep > Ascend2Target.Disabled)
                            && (errStep < Ascend2Target.FineTune)
                            && (hTargetError > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT))
                        {
                            hCurrentError = (Core.Landing.targetingResult.distanceToTarget > 0)
                                ? Core.Landing.targetingResult.distanceToTarget
                                : hTargetError;
                        }
                        else
                        {
                            hCurrentError = 0; // Dont allow fine tune when landing.
                        }
                    }

                    // Near-pad (<0.80 m): aggressive chase still useful down to ~200 mm, then
                    // blend into final settle. Under ~200 mm the sqrt pad-chase desired H vel
                    // overcompensates through the unified 3D PID + debug93 cone (lean angles
                    // when H should already be ~0). Settle: no pad-chase velocity; only damp
                    // residual horizontal surface speed. Blend so it does not cliff at 0.2 m.
                    const double finalSettleM = 0.2;
                    const double nearPadAggressiveM = 0.80;
                    if (hTargetError < nearPadAggressiveM)
                    {
                        double hForSqrt = Math.Max(hTargetError, 1e-6);
                        Vector3d aggressiveDesired = (Math.Sqrt((1 + Core.Landing.g) * hForSqrt * maxHThrust / divisor) * 2.0 / 1.5) * courseCorrection.normalized;
                        Vector3d aggressiveHerror = 0.25 * aggressiveDesired - hSurfaceVelocity;

                        Vector3d settleDesired = Vector3d.zero;
                        Vector3d settleHerror = -hSurfaceVelocity;

                        double settleFactor = Math.Max(0.0, Math.Min(1.0,
                            (nearPadAggressiveM - hTargetError) / (nearPadAggressiveM - finalSettleM)));
                        desiredHorizontalVel = Vector3d.Lerp(aggressiveDesired, settleDesired, settleFactor);
                        herror = Vector3d.Lerp(aggressiveHerror, settleHerror, settleFactor);
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

                // Lateral is a first-class PID axis in setThrustVector (eL / aL).
                // Do not fold it into herror: that was the old axis-independent mix and it
                // skews the landing frame near the pad (BuildLandingFrame falls back to
                // hError when desiredHorizontalVel is tiny).

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
                        // Slowdown latch is for post-climb coast. While vacuum H toward the pad
                        // is still tiny, stay on tilt buildup (desired-V drop during climb must
                        // not clamp H to ~9 m/s and warp with Ap over the launch site).
                        bool downrangeReady = HasMeaningfulDownrangeTowardTarget();
                        if (downrangeReady && (subOrbitalTileLatched || (previousDesiredVerticalSpeed > desiredVerticalSpeed)))
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
                            if (!downrangeReady) subOrbitalTileLatched = false; // force tilt until H toward pad
                            if (startTiltHVel < 0.1) startTiltHVel = VesselState.speedSurfaceHorizontal;
                            double orig_hvel = desiredHorizontalVel.magnitude - startTiltHVel;
                            desiredHorizontalVel = desiredHorizontalVel.normalized * startTiltHVel;

                            if (tiltDelta < 0.1)
                            {
                                // Vacuum mountain start: |TgtAlt-altASL| can be tens of km when
                                // already above the pad, so pitch-over never ramps (H stays ~1).
                                // Cap the pitch window by g/a_max so high-TWR low-g tilts in ~1 km.
                                double absDelta = Math.Abs(Core.Landing.TargetAltitude - VesselState.altitudeASL);
                                double gLoc = Math.Max(Core.Landing.g, 0.05);
                                double aMax = Math.Max(VesselState.limitedMaxThrustAccel, gLoc);
                                double tiltCap = Math.Max(300.0, Math.Min(1500.0, 8.0 * aMax / gLoc));
                                if (VesselState.altitudeASL > Core.Landing.TargetAltitude)
                                    tiltDelta = Math.Max(300.0, Math.Min(absDelta, tiltCap));
                                else
                                    tiltDelta = Math.Max(500.0, absDelta);
                            }
                            else if ((VesselState.speedVertical > Core.Landing.g * 2 * 28)   // 28<=30<=20(too shallow)<=30
                                || ((MainBody.atmosphere == false)
                                    && (VesselState.altitudeTrue >= 100.0)
                                    && ((Math.Abs(VesselState.orbitApA) >= CruiseApaTarget())
                                        || (VesselState.altitudeASL >= CruiseApaTarget()))))
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
                    double checkApa = CruiseApaTarget();
                    if (Math.Abs(VesselState.orbitApA) >= checkApa)
                    {
                        // Vacuum: do not arm coast/warp on ApA height alone - need H toward pad
                        // AND a sane finite Ap near cruise (midnight handoff at ApA~2 Mm / InfAp dump).
                        if ((MainBody.atmosphere == false) && !HasMeaningfulDownrangeTowardTarget())
                        {
                            LogDownrangeH("TargetSubOrbital: wait downrange H before coast/warp H="
                                + VesselState.speedSurfaceHorizontal.ToString("F1")
                                + " ApA=" + VesselState.orbitApA.ToString("F0")
                                + " cruise=" + checkApa.ToString("F0"));
                        }
                        else if ((MainBody.atmosphere == false) && !ApaSaneForVacuumCoast(checkApa))
                        {
                            LogDownrangeH("TargetSubOrbital: wait Ap dump before coast/warp H="
                                + VesselState.speedSurfaceHorizontal.ToString("F1")
                                + " ApA=" + VesselState.orbitApA.ToString("F0")
                                + " cruise=" + checkApa.ToString("F0"));
                        }
                        else if (TripOn && TripCaQuiet && (MainBody.atmosphere == false)
                                 && (VesselState.speedVertical > 1.0))
                        {
                            // Step 3 (k > 1 only): keep the powered vz cancel; no ballistic coast/warp.
                            // FineTune (else below) once vz <= 1 m/s at cruise.
                            LogTrip("MTT wait vz cancel before TSO handoff vz="
                                + VesselState.speedVertical.ToString("F1")
                                + " ApA=" + VesselState.orbitApA.ToString("F0")
                                + " cruise=" + checkApa.ToString("F0"));
                        }
                        else if ((decelerationStartTime > 0) && (VesselState.speedVertical > 0))
                        {
                            suppressThrottle++; // suppress throttle during initial burn
                            trace += 'S' + 'a';
                        }
                        else
                        {
                            // Ap already high and at/past Ap: skip warp, hand FineTune now.
                            // Without this, suppressThrottle stays 0 forever when climb ends first.
                            stopIncrease = true;
                            suppressThrottle = 2;
                            trace += 'U';
                            if (errStep < Ascend2Target.FineTune)
                                errStep = Ascend2Target.FineTune;
                            Debug.Log("TargetSubOrbital: Ap ready, already at/past Ap -> FineTune H="
                                + VesselState.speedSurfaceHorizontal.ToString("F1"));
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
                        Debug.Log("TargetSubOrbital: coast/warp reached Ap -> FineTune ApA="
                            + VesselState.orbitApA.ToString("F0")
                            + " tAp=" + VesselState.orbitTimeToAp.ToString("F1")
                            + " alt=" + VesselState.altitudeTrue.ToString("F0")
                            + " errStep=" + ((int)errStep).ToString());
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
                _coastClearThisDrive = false;
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
                // Retry deferred handoff once hop/terrain look-ahead clears (Vesta).
                double handoffH = Core.Landing.targetingResult.hTargetError;
                if (pendingSubOrbitHandOff
                    && (handoffH > H_SUBORBITAL_DISTANCE_THRESHOLD_CONSTANT)
                    && CoastClearForSubOrbitHandOff(handoffH))
                {
                    errStep = Ascend2Target.FineTune;
                    pendingSubOrbitHandOff = false;
                    trace += 'r';
                }

                // TargetSubOrbital advances errStep to FineTune at Ap after coast/warp.
                // Require coast-clear before leaving MoveToTarget (collision hop stays in charge).
                if ( (errStep >= Ascend2Target.FineTune) && (errStep < Ascend2Target.Completed))
                {
                    if (!CoastClearForSubOrbitHandOff(handoffH))
                    {
                        errStep = Ascend2Target.Completed;
                        pendingSubOrbitHandOff = true;
                        trace += 't';
                        Debug.Log("MoveToTarget: defer TargetSubOrbit handoff - coast not clear hErr="
                            + handoffH.ToString("F0")
                            + " hop=" + collisionHopActive
                            + " vCol=" + vCollisionCounter);
                    }
                    else
                    {
                        pendingSubOrbitHandOff = false;
                        Core.Thrust.ThrustOff();
                        if (Core.Landing.LandingType == 1)
                        {
                            Debug.Log("MoveToTarget: Ap handoff -> TargetSubOrbit ApA="
                                + VesselState.orbitApA.ToString("F0")
                                + " tAp=" + VesselState.orbitTimeToAp.ToString("F1")
                                + " alt=" + VesselState.altitudeTrue.ToString("F0")
                                + " TgtAlt=" + Core.Landing.TgtAlt.ToString("F0")
                                + " hErr=" + hCurrentError.ToString("F0")
                                + " H=" + VesselState.speedSurfaceHorizontal.ToString("F1"));
                            LogTripSpeedHeadroom(); // Step 1: read-only
                            return new TargetSubOrbit(Core);
                        }
                        else
                        {
                            return new CourseCorrection(Core);
                        }
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
