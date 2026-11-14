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
            enum Step { STATE_INIT, STATE_LATERAL, STATE_PROGRADE, STATE_LATERAL_FINE, STATE_RETROGRADE, STATE_CORRECT, STATE_COAST_AP, STATE_TRIP_HOOP, STATE_TRIP_DESCEND };
            Step step = Step.STATE_INIT;
            private const float H_CORRECTION_ANGLE_CONSTANT = 0.4f; // Used to determine horizontal correction angle 
            private const double OVERSHOOT_ANGLE_FRACTION = 0.9; // Small margin to avoid losing vertical control
            private const double LIMITED_MAX_THRUST_G_RATIO = 3.125; // this is multiplied with Mainbody g
            private const double TARGET_FAR_DISTANCE_THRESHOLD = 2.5*50000; // Distance to target at which to switch from prograde/retrograde burn to fine lateral control to target before final landing burn. This is used to avoid overshooting the target when doing a prograde or retrograde burn and allow for more precise targeting of the final landing burn.
            private const double TARGET_DISTANCE_THRESHOLD = 300; // Final Distance to target at which to switch to final landing burn.   
            private const double DELTA_TARGET_ALT_THRESHOLD = 600; // Threshold for switching between prograde and retrograde burn when close to target - if the target altitude is above the current altitude by this amount then we will do a prograde burn, if it's below then we will do a retrograde burn, if it's close then we will do a lateral burn to fine tune the approach to target before the final burn.
            // Ricardo: MoveToTarget owns initial coast-to-Ap handoff only. Once in TargetSubOrbit,
            // stay here and shape pad overflight to TgtAlt. Ap past => RETROGRADE dump in place (no handback).
            private const double AP_HALFWAY_TARGET = 0.50;
            private const double AP_HALFWAY_LO = 0.35;
            private const double AP_HALFWAY_HI = 0.65;
            // Once Ap past / frac>~0.5 seen: stay RETROGRADE until frac stably in positive
            // halfway band [LO, TARGET]. Never clear on negative frac (04:13 signed-angle noise).
            private const int AP_PULL_CLEAR_TICKS = 50; // ~1s @50Hz; was 5 (~0.1s) -> 230322 RETRO thrash
            private const double TERRAIN_RAISE_STEP_M = 800.0;
            private const double TERRAIN_RAISE_INTERVAL_S = 2.0;
            private const double TERRAIN_RAISE_MAX_TGTALT_M = 25000.0;
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
            private double deltaDistance;
            private double tUT;

            private bool throttleOn = false;
            private int throttleCount = 0;
            private int LateralCount = 0;
            private int ProgradeCount = 0;
            private int RetrogradeCount = 0;
            private int LateralFineCount = 0;
            private double _lastRetroExitLogUt = double.NegativeInfinity;
            private double _lastApPastBounceLogUt = double.NegativeInfinity;
            private double _lastApSeqLogUt = double.NegativeInfinity;
            private double _lastTerrainRaiseUt = double.NegativeInfinity;
            private double _lastLbHandoffLogUt = double.NegativeInfinity;
            private bool _establishingAp = false;
            private bool _apPullLatch = false;
            private int _apPullClearStreak = 0;
            private double _apPullLatchEvalUt = double.NegativeInfinity;
            private double _lastApPullLatchLogUt = double.NegativeInfinity;
            // Shape once entered from MoveToTarget Ap handoff (or after in-place Ap dump clears).
            private bool _apReachedForShape = false;
            // Ricardo (Nov 9): terrain decides PRO vs RETRO before the first shaping burn.
            // INIT owns the decision (first sample on INIT entry); LATERAL re-samples until a
            // valid sample (predDt > 1 s) reads blocked, then it latches. Consumed at the LATERAL exit.
            // Per-entry: TSO is constructed fresh on every entry, so initializers reset these.
            private bool _terrainBlockedAtInit = false;
            private bool _terrainInitSampleValid = false;
            private double _terrainInitLastPredDt = double.NaN;
            private double _lastApPhaseLogUt = double.NegativeInfinity;
            private string _lastApPhaseLogged = "";
            private double minVerticalAngle;

            // ---- Trip-speed lever, steps 3+4 (k > 1 only; never set when k == 1.0) ----
            // Powered level cruise: started where PROGRADE would exit to LATERAL_FINE once the path
            // to the pad is clear; holds ASL (down-thrust when faster than circular) at k x the
            // path-clear speed, then hands LandingBurn the ignition point on LB's own formula.
            private bool _tripCruise = false;
            private double _tripVClear = double.NaN;
            private double _tripVTarget = double.NaN;
            private double _tripHoldAsl = double.NaN;
            private double _lastTripLogUt = double.NegativeInfinity;
            private const double TRIP_PREDESCENT_M = 5000.0;  // start the -3 m/s pre-descent this far before dIgn
            private const double TRIP_HOLD_MAX_RAISE_M = 3000.0;
            private const double TRIP_DESCEND_TERRAIN_MARGIN_M = 2000.0; // Ricardo t294u: descent target floor above remaining ridge
            private const double TRIP_HOOP_CLEARANCE_M = TRIP_DESCEND_TERRAIN_MARGIN_M; // remaining-path clearance at targetAsl (holds by construction)
            private const double TRIP_DESCEND_VZ_MAX = 15.0;     // cap on P-arrival vzCmd (Ricardo t304u: 12, Nov 14 t314u: 15 m/s)
            // Ricardo t304u: widened trip hoop near the pad (powered state, trip vH kept).
            // floor = max(padAsl + gate, max terrain within dIgn of pad + TRIP_DESCEND_TERRAIN_MARGIN_M),
            // ceiling = floor + TRIP_HOOP_CEIL_ABOVE_M; active within TRIP_HOOP_LIMIT_DIGN * dIgn.
            private const double TRIP_HOOP_LIMIT_DIGN = 3.0;     // Ricardo Nov 14 t314u (was 2.25)
            private const double TRIP_HOOP_CEIL_ABOVE_M = 2000.0;
            private const double TRIP_HOOP_VY_IGN_MIN = -20.0;   // predicted vy (m/s, + down) at LB ignition: climb <= 20 m/s (Nov 14 t314u; LB distance-ignites while climbing)
            private const double TRIP_HOOP_COAST_MAX_S = 600.0;
            private double _lastTripWideEvalUt = double.NegativeInfinity;
            private double _tripTargetAsl = double.NaN;
            private double _tripRidgeAsl = double.NaN;
            private double _lastTripHoopEvalUt = double.NegativeInfinity;
            private bool TripCruiseActive => _tripCruise && Core.Landing.TripSpeedOn;
            private double _tripFloorUt = double.NegativeInfinity;
            private double _tripFloorAsl = 0.0;
            private double _tripFloorTryUt = double.NegativeInfinity;

            // Autotest / KSP.log 1 Hz guidance snapshot (no behavior change).
            private double _lastGuidanceLogUt = double.NegativeInfinity;
            private Step _prevLoggedStep = Step.STATE_INIT;
            private Vector3d _lastHerror = Vector3d.zero;
            private Vector3d _lastDownrange = Vector3d.zero;
            public string AutotestStepName = "";
            public double AutotestPadDh = double.NaN;
            public double AutotestPadDt = double.NaN;
            public double AutotestApFrac = double.NaN;
            public bool AutotestApPast = false;
            public string AutotestBurnAxis = "";
            public string AutotestHandoff = "";
            public double AutotestHoopCenter = double.NaN;
            public double AutotestPredH = double.NaN;
            public double AutotestHoopMiss = double.NaN;

            public TargetSubOrbit(MechJebCore core) : base(core)
            {
                Core.Landing.NoteWarp("TSO exit:init");
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

                double timeToAp = VesselState.orbitTimeToAp;
                if (MechJebModuleLandingAutopilot.ApoapsisIsFinite(Vessel.orbit)
                    && !double.IsNaN(timeToAp) && !double.IsInfinity(timeToAp) && timeToAp < 120)
                {
                    minVerticalAngle = 0;
                }
                else
                {
                    minVerticalAngle = -0.25;
                }

                // If the current altitude is low, then adjust this to 1/3 of the current altitude to avoid overshooting.
                // Never crush below the gravity-scaled ComputeOptimal gate (094807: mountain start
                // AGL~2 km set TgtAlt=665; +/-600 band then accepted ~50-600 m over pad - too tight for LB).
                if (VesselState.altitudeTrue < 3 * Core.Landing.TgtAlt)
                {
                    Core.Landing.TgtAlt = VesselState.altitudeTrue / 3.0;
                }
                Core.Landing.EnsureTgtAltMeetsLbOverheadFloor();
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // ratio retained for steepness curve only - no MoveToTarget near-pad handback.
                // TSO owns suborbital shape to pad@TgtAlt then hands DIRECTLY to LandingBurn.
                // Enforce gravity-scaled LB overhead floor every tick (crush / bump-cap cannot undercut).
                Core.Landing.EnsureTgtAltMeetsLbOverheadFloor();
                if (_apReachedForShape)
                    Core.Landing.SyncTgtAltToLandingHoop();
                if (ratio == 0)
                    ratio = Core.Landing.getHDistanceToTarget() / Math.Max(1.0, VesselState.altitudeTrue);

                // Ricardo: stay in TargetSubOrbit. Gate dump-in-place / shape only (no Ap handback).
                Vector3d tgtPosSeq = Core.Target.GetPositionTargetPosition();
                if (LateralFirstPending())
                    SampleInitTerrainDecision();
                AutopilotStep apGate = ApplyApOwnershipGate(tgtPosSeq);
                if (apGate != null)
                    return apGate;

                // Terrain clearance: local suicide always (incl. RETRO dump); TgtAlt bump only while shaping.
                // MTT only on proven infeasibility inside ApplyShapingTerrainClearance — not after success.
                if (_apReachedForShape || step == Step.STATE_RETROGRADE || _apPullLatch)
                {
                    AutopilotStep terrainFix = ApplyShapingTerrainClearance(tgtPosSeq);
                    if (terrainFix != null)
                        return terrainFix;
                }

                // Completed suborbit (pad height, Ap before pad, path clear) -> LandingBurn.
                // Skips the far-range brake gate: TSO already finished its job.
                if (TryDirectLandingBurnHandoff(tgtPosSeq, out string lbReason))
                {
                    Core.Thrust.ThrustOff();
                    Debug.Log("[TSO->LB] direct handoff: " + lbReason
                        + " step=" + step
                        + " dist=" + Core.Landing.getHDistanceToTarget().ToString("F0")
                        + " hoopCenter=" + AutotestHoopCenter.ToString("F0")
                        + " predH=" + AutotestPredH.ToString("F0")
                        + " miss=" + AutotestHoopMiss.ToString("F0"));
                    Core.Landing.LogCoastFloorDiag("handoff");
                    return new LandingBurn(Core);
                }
                else if (lbReason == "pad-behind")
                {
                    // Proven infeasible mid-shape: pad already behind — MTT recovery.
                    Debug.Log("[TSO->MTT] infeasible: " + lbReason + " step=" + step);
                    Core.Thrust.ThrustOff();
                    return new MoveToTarget(Core);
                }
                else if (_apReachedForShape && !string.IsNullOrEmpty(lbReason)
                         && Planetarium.GetUniversalTime() - _lastLbHandoffLogUt >= 1.0)
                {
                    // 1 Hz threshold reasoning while shaping toward the handoff.
                    _lastLbHandoffLogUt = Planetarium.GetUniversalTime();
                    if (lbReason.StartsWith("hoop-fail")
                        || lbReason.StartsWith("hoop-eval")
                        || lbReason.StartsWith("lb-gate")
                        || lbReason.StartsWith("path-")
                        || lbReason.StartsWith("needs-pull")
                        || lbReason.StartsWith("ap-past")
                        || lbReason.StartsWith("not-descending"))
                    {
                        Debug.Log("[TSO->LB] waiting: " + lbReason + " step=" + step);
                    }
                }

                // STATE_CORRECT: shape residuals, or MTT only on proven infeasibility.
                // Success path is TryDirectLandingBurnHandoff above (not MTT).
                if (step == Step.STATE_CORRECT)
                {
                    Core.Thrust.ThrustOff();
                    Vector3d tgtPosGate = Core.Target.GetPositionTargetPosition();
                    bool gateNeedsPull = ApNeedsPullBack(tgtPosGate);
                    if (TryPadOverflightGate(tgtPosGate, out double padDhCorrect, out _)
                        && padDhCorrect > DELTA_TARGET_ALT_THRESHOLD
                        && !gateNeedsPull && _apReachedForShape
                        && !TerrainHoopShaped())
                    {
                        step = Step.STATE_PROGRADE;
                        throttleCount = 0;
                    }
                    else if (gateNeedsPull)
                    {
                        step = Step.STATE_RETROGRADE;
                        throttleCount = 0;
                    }
                    else if (PadOverflightBehind(tgtPosGate))
                    {
                        // Proven infeasible: pad already behind — MTT owns recovery/CA hop.
                        Debug.Log("[TSO->MTT] infeasible: pad behind while CORRECT");
                        return new MoveToTarget(Core);
                    }
                    else
                    {
                        step = Step.STATE_LATERAL;
                        throttleCount = 0;
                    }
                }

                {
                    double desiredVerticalSpeed = 0; // for now just set to zero vertical velocity

                    // Calculate Thrust Amount and Direction
                    if (TripCruiseActive)
                        TripCruiseCommand(ref thrustDir); // k > 1 only (never active at k == 1.0)
                    else
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

                    MaybeLogGuidanceTick(thrustDir);
                    return this;
                }
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
                _lastHerror = herror;
                _lastDownrange = downrangeVec;

                // Legacy COAST_AP: clear local step; stay in TargetSubOrbit and shape.
                if (step == Step.STATE_COAST_AP)
                {
                    Core.Landing.NoteWarp("TSO exit:coast-ap->lateral");
                    Core.Warp.MinimumWarp();
                    step = Step.STATE_LATERAL;
                }

                // TgtAlt shapes coast height OVER THE PAD (overflight gate so LandingBurn
                // has ZEM-ZEV room). Do not match height at ClosestPoint when that point is
                // still tens of km past the dest - intercept past is allowed.
                // Landing descent curve in setVerticalSpeed is left for lateral modes.
                if (step == Step.STATE_PROGRADE || step == Step.STATE_RETROGRADE)
                {
                    double gateDh = deltaHeight;
                    double gateDt = (!double.IsNaN(tUT) && !double.IsInfinity(tUT))
                        ? Math.Max(5.0, tUT - Planetarium.GetUniversalTime())
                        : 5.0;
                    if (TryPadOverflightGate(targetPos, out double padDh, out double padDt))
                    {
                        gateDh = padDh;
                        gateDt = padDt;
                    }
                    if (!double.IsNaN(gateDh) && !double.IsInfinity(gateDh)
                        && !double.IsNaN(gateDt) && !double.IsInfinity(gateDt) && gateDt > 0)
                    {
                        desiredVerticalSpeed = Math.Max(-150.0, Math.Min(150.0, gateDh / gateDt));
                        // RETROGRADE pulls Ap back: do not command a climb that fights the dump.
                        if (step == Step.STATE_RETROGRADE)
                            desiredVerticalSpeed = Math.Min(0.0, desiredVerticalSpeed);
                    }
                }

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


            // Signed along-track angle from 'from' to 'to' about orbit normal (prograde +).
            private static double SignedOrbitAngle(Vector3d from, Vector3d to, Vector3d orbitNormal)
            {
                Vector3d a = Vector3d.Exclude(orbitNormal, from);
                Vector3d b = Vector3d.Exclude(orbitNormal, to);
                if (a.sqrMagnitude < 1e-12 || b.sqrMagnitude < 1e-12)
                    return 0;
                a.Normalize();
                b.Normalize();
                double sin = Vector3d.Dot(orbitNormal.normalized, Vector3d.Cross(a, b));
                double cos = Vector3d.Dot(a, b);
                return Math.Atan2(sin, cos);
            }

            /// <summary>
            /// True when apoapsis ground-track is still past the landing target along-track.
            /// ClosestPoint / intercept past dest is allowed and must NOT force RETROGRADE.
            /// Hyperbolic / Inf Ap: pad still ahead counts as Ap-past (must dump).
            /// Do not gate on timeToAp<600 - small-body approaches (20:18 ~110 km out)
            /// have Ap still past the pad with timeToAp well over 10 minutes.
            /// </summary>
            private bool ApoapsisPastDestination(Vector3d targetPosBody)
            {
                if (!MechJebModuleLandingAutopilot.IsFiniteVec(targetPosBody))
                    return false;

                Vector3d pos = VesselState.CoM - Vessel.mainBody.position;
                Vector3d n = Vector3d.Cross(pos, VesselState.orbitalVelocity);
                if (n.sqrMagnitude < 1e-6)
                    return false;
                n.Normalize();

                double angPad = SignedOrbitAngle(pos, targetPosBody, n);
                // ~1 km along-track margin at body radius
                double marginRad = 1000.0 / Math.Max(Vessel.mainBody.Radius, 1.0);

                // Inf Ap / hyperbola: no apoapsis to place before the pad - dump while pad ahead.
                if (!MechJebModuleLandingAutopilot.ApoapsisIsFinite(Vessel.orbit))
                    return angPad > marginRad;

                double now = Planetarium.GetUniversalTime();
                // Prefer raw orbit.timeToAp (not VesselState MovingAverage) for geometry.
                double tAp = Vessel.orbit.timeToAp;
                // At/just past Ap (tAp<=0) or glitched tAp: vessel is at the Ap ground-track.
                // If pad is already behind, that Ap was past the pad - keep dumping.
                // (21:12: tAp flicker cleared pastDest and tooLow hopped PROGRADE at frac=2.57.)
                if (double.IsNaN(tAp) || double.IsInfinity(tAp) || tAp <= 0)
                    return angPad < -marginRad;
                Vector3d apPos = Vessel.orbit.getPositionAtUT(now + tAp) - Vessel.mainBody.position;
                if (!MechJebModuleLandingAutopilot.IsFiniteVec(apPos))
                    return false;

                double angAp = SignedOrbitAngle(pos, apPos, n);
                return angAp > angPad + marginRad;
            }

            /// <summary>
            /// Coast height above the target radius at estimated pad overflight (TgtAlt gate)
            /// and dt to that overflight. Used so pro/retro shape path over the pad, not at
            /// ClosestPoint when ClosestPoint is far past dest.
            /// </summary>
            private bool TryPadOverflightGate(Vector3d targetPosBody, out double gateDeltaHeight, out double gateDt)
            {
                gateDeltaHeight = double.NaN;
                gateDt = double.NaN;

                if (!MechJebModuleLandingAutopilot.IsFiniteVec(targetPosBody))
                    return false;

                Vector3d pos = VesselState.CoM - Vessel.mainBody.position;
                Vector3d n = Vector3d.Cross(pos, VesselState.orbitalVelocity);
                if (n.sqrMagnitude < 1e-6)
                    return false;
                n.Normalize();

                double r = pos.magnitude;
                if (double.IsNaN(r) || r < 1.0)
                    return false;

                double hOrbSpeed = Vector3d.Exclude(VesselState.up, VesselState.orbitalVelocity).magnitude;
                if (hOrbSpeed < 10.0)
                    return false;

                double angPad = SignedOrbitAngle(pos, targetPosBody, n);
                double alongTrack = angPad * r; // signed meters along orbit
                // Pad already behind / at: do not clamp a negative dt up to 5s (that
                // would sample ahead of the vessel instead of the pad).
                if (alongTrack <= 0.0)
                    return false;
                // One height-over-pad measure for state logic AND the LB hoop handoff:
                // unpowered-coast height above pad terrain at the ClosestPoint ground-track
                // point (= hoop predH). The old along-track/hSpeed time estimate disagreed with
                // the handoff predH by ~4 km (20261109_025729: 7.8 vs 3.7 km), and ignored
                // crossrange. Pad-behind / no-speed preconditions above are unchanged.
                if (!Core.Landing.TryPredictedHeightOverPad(out double heightOverPad, out double predDt, out _))
                    return false;
                if (predDt > 3600.0)
                    return false;

                gateDeltaHeight = Core.Landing.TgtAlt - heightOverPad;
                gateDt = Math.Max(5.0, predDt);
                return !double.IsNaN(gateDeltaHeight) && !double.IsInfinity(gateDeltaHeight);
            }

            /// <summary>
            /// True when finite signed along-track to pad is clearly behind (~>1 km past).
            /// Same orbit-normal SignedOrbitAngle approach as TryPadOverflightGate.
            /// </summary>
            private bool PadOverflightBehind(Vector3d targetPosBody)
            {
                if (!MechJebModuleLandingAutopilot.IsFiniteVec(targetPosBody))
                    return false;

                Vector3d pos = VesselState.CoM - Vessel.mainBody.position;
                Vector3d n = Vector3d.Cross(pos, VesselState.orbitalVelocity);
                if (n.sqrMagnitude < 1e-6)
                    return false;
                n.Normalize();

                double r = pos.magnitude;
                if (double.IsNaN(r) || r < 1.0)
                    return false;

                double angPad = SignedOrbitAngle(pos, targetPosBody, n);
                double alongTrack = angPad * r;
                if (double.IsNaN(alongTrack) || double.IsInfinity(alongTrack))
                    return false;
                return alongTrack < -1000.0;
            }

            /// <summary>
            /// Along-track fraction of apoapsis toward the pad (angAp/angPad).
            /// ~0.5 = halfway before pad. >1 = Ap past pad. False if unavailable.
            /// </summary>
            private bool TryApAlongTrackFraction(Vector3d targetPosBody, out double fraction, out double timeToAp)
            {
                fraction = double.NaN;
                timeToAp = double.NaN;
                if (!MechJebModuleLandingAutopilot.IsFiniteVec(targetPosBody))
                    return false;
                if (!MechJebModuleLandingAutopilot.ApoapsisIsFinite(Vessel.orbit))
                    return false;

                Vector3d pos = VesselState.CoM - Vessel.mainBody.position;
                Vector3d n = Vector3d.Cross(pos, VesselState.orbitalVelocity);
                if (n.sqrMagnitude < 1e-6)
                    return false;
                n.Normalize();

                double angPad = SignedOrbitAngle(pos, targetPosBody, n);
                double marginRad = 1000.0 / Math.Max(Vessel.mainBody.Radius, 1.0);
                if (angPad <= marginRad)
                    return false;

                double tAp = Vessel.orbit.timeToAp;
                if (double.IsNaN(tAp) || double.IsInfinity(tAp) || tAp <= 0)
                    return false;
                timeToAp = tAp;

                double now = Planetarium.GetUniversalTime();
                Vector3d apPos = Vessel.orbit.getPositionAtUT(now + tAp) - Vessel.mainBody.position;
                if (!MechJebModuleLandingAutopilot.IsFiniteVec(apPos))
                    return false;

                double angAp = SignedOrbitAngle(pos, apPos, n);
                fraction = angAp / angPad;
                return !double.IsNaN(fraction) && !double.IsInfinity(fraction);
            }

            /// <summary>
            /// Ricardo (Nov 9): hoop raised for terrain => TSO has nothing better to shape in
            /// height. True when the hoop is terrain-driven AND the inbound coast clears terrain;
            /// PRO/RETRO are then done (go LATERAL_FINE) and the padDh band no longer applies.
            /// </summary>
            private bool TerrainHoopShaped()
            {
                bool driven = Core.Landing.LandingHoopIsTerrainDriven(out bool pathClear);
                return driven && pathClear;
            }

            // CA enters PROGRADE when the coast is below 900 + 1.1*floor (margin < 0) and used to leave
            // at margin >= 0, i.e. exactly on the threshold (171027: minMargin -31/+2/0/+1 m ->
            // FINE->PRO->FINE->PRO->FINE, TgtAlt +800 m per re-entry). Leave only with +100 m.
            private const double SUBORBIT_PATH_MIN_ALTITUDE_M = 900.0; // mirror LA SUBORBIT_PATH_MIN_ALTITUDE
            private const double TERRAIN_HOOP_EXIT_MARGIN_M = 100.0;

            private bool TerrainClearWithExitMargin()
            {
                try { return Core.Landing.SubOrbitPathClearToTarget(SUBORBIT_PATH_MIN_ALTITUDE_M + TERRAIN_HOOP_EXIT_MARGIN_M); }
                catch (System.Exception) { return true; } // fall back to the previous exit
            }

            private bool ApApproximatelyHalfwayBeforePad(Vector3d targetPosBody)
            {
                if (!TryApAlongTrackFraction(targetPosBody, out double frac, out _))
                    return false;
                return frac >= AP_HALFWAY_LO && frac <= AP_HALFWAY_HI;
            }

            /// <summary>
            /// Instantaneous Ap-past / frac>~0.5 check (no latch). ClosestPoint/intercept
            /// past is allowed and does not set this.
            /// </summary>
                        private bool ApNeedsPullBackRaw(Vector3d targetPosBody, out double frac)
            {
                frac = double.NaN;
                // 101149: padDh already in band (-122) but frac noise 28.27 + apPast flicker
                // armed DUMP RETRO and zeroed H at ~32 km. Once overflight is shaped,
                // never pull on frac/ApPast noise — hand LB or coast; LB owns braking.
                if (TryPadOverflightGate(targetPosBody, out double padDhShaped, out _))
                {
                    TryApAlongTrackFraction(targetPosBody, out frac, out _);
                    if (Math.Abs(padDhShaped) <= DELTA_TARGET_ALT_THRESHOLD)
                        return false;
                    // Still too high: real dump needed - gate-driven hoop only. Terrain-driven
                    // hoop has no height band (Nov 9 rule); dumping height would re-block the
                    // terrain the hoop was raised for. Ap-past / frac checks below still apply.
                    // Terrain blocks the coast (INIT decision): dumping height only re-blocks it.
                    bool terrainHold = _terrainBlockedAtInit
                        && (LateralFirstPending() || step == Step.STATE_PROGRADE);
                    if (padDhShaped < -DELTA_TARGET_ALT_THRESHOLD && !TerrainHoopShaped() && !terrainHold)
                        return true;
                }
                if (ApoapsisPastDestination(targetPosBody))
                {
                    TryApAlongTrackFraction(targetPosBody, out frac, out _);
                    return true;
                }
                if (TryApAlongTrackFraction(targetPosBody, out frac, out _))
                {
                    if (frac > AP_HALFWAY_TARGET)
                        return true;
                }
                return false;
            }

            private void LogApPullLatch(string action, double frac, bool haveFrac)
            {
                double now = Planetarium.GetUniversalTime();
                if (now - _lastApPullLatchLogUt < 1.0)
                    return;
                _lastApPullLatchLogUt = now;
                Debug.Log("TargetSubOrbit ApPullLatch " + action
                    + " frac=" + (haveFrac ? frac.ToString("F2") : "n/a")
                    + " streak=" + _apPullClearStreak);
            }

            /// <summary>
            /// Once per UT: set latch on Ap past / frac>~0.5; clear only after
            /// AP_PULL_CLEAR_TICKS consecutive ticks with known frac in [AP_HALFWAY_LO, AP_HALFWAY_TARGET]
            /// and ApoapsisPastDestination false. Never clear on negative/NaN/frac>0.5 (04:13).
            /// Survives pastDest / tAp flicker that would otherwise clear and allow tooLow->PROGRADE.
            /// </summary>
            private void UpdateApPullLatch(Vector3d targetPosBody)
            {
                double now = Planetarium.GetUniversalTime();
                // Same-UT re-entry from multiple call sites: do not advance clear streak.
                if (now == _apPullLatchEvalUt)
                    return;
                _apPullLatchEvalUt = now;

                bool instant = ApNeedsPullBackRaw(targetPosBody, out double frac);
                bool haveFrac = !double.IsNaN(frac);
                if (!haveFrac)
                    haveFrac = TryApAlongTrackFraction(targetPosBody, out frac, out _);

                if (instant)
                {
                    if (!_apPullLatch)
                    {
                        _apPullLatch = true;
                        _apPullClearStreak = 0;
                        LogApPullLatch("SET", frac, haveFrac);
                    }
                    else
                        _apPullClearStreak = 0;
                    return;
                }

                if (!_apPullLatch)
                    return;

                // Latched: clear ONLY with proven Ap progress (frac in halfway band).
                // 230322: padBandOk alone (|padDh|<=600) cleared latch in ~0.1s while frac
                // was negative, bouncing RETRO->LATERAL_FINE 28x. Wild-frac dumps still
                // clear via RETROGRADE heightOk+hNearNeeded (CLEAR_H).
                bool pastDest = ApoapsisPastDestination(targetPosBody);
                bool inClearBand = haveFrac
                    && frac >= AP_HALFWAY_LO
                    && frac <= AP_HALFWAY_TARGET;
                bool padBandOk = false;
                if (TryPadOverflightGate(targetPosBody, out double padDhClear, out _))
                    padBandOk = Math.Abs(padDhClear) <= DELTA_TARGET_ALT_THRESHOLD;
                if (!pastDest && inClearBand) // 230322: padBandOk alone cleared on neg frac
                {
                    _apPullClearStreak++;
                    if (_apPullClearStreak >= AP_PULL_CLEAR_TICKS)
                    {
                        _apPullLatch = false;
                        _apPullClearStreak = 0;
                        LogApPullLatch(padBandOk ? "CLEAR_HALFWAY_PADOK" : "CLEAR_HALFWAY", frac, haveFrac);
                    }
                }
                else
                {
                    // Missing/negative/NaN frac, past dest, or not yet halfway: keep latch.
                    _apPullClearStreak = 0;
                }            }

            /// <summary>
            /// True while Ap along-track is past the pad or beyond ~halfway to pad,
            /// OR while pull latch is held. While true: ONLY RETROGRADE to pull Ap back
            /// toward ~0.5 - never PROGRADE, LATERAL_FINE, or terrain TgtAlt climb
            /// (21:12/21:35 retro immediately flipped to prograde via tooLow then LATERAL_FINE).
            /// ClosestPoint/intercept past is allowed and does not set this.
            /// </summary>
            private bool ApNeedsPullBack(Vector3d targetPosBody, out double frac)
            {
                // k > 1 trip cruise only: a level cruise faster than circular sits at periapsis, so
                // Ap-past/frac would force RETRO and undo the cruise (deadlock). The cruise owns height;
                // its abort hands back to these rules. Never true at k == 1.0.
                if (TripCruiseActive)
                {
                    frac = double.NaN;
                    return false;
                }
                UpdateApPullLatch(targetPosBody);
                frac = double.NaN;
                bool haveFrac = TryApAlongTrackFraction(targetPosBody, out frac, out _);
                // Gate already shaped: drop latch and do not dump (101149).
                if (TryPadOverflightGate(targetPosBody, out double padDhDone, out _)
                    && Math.Abs(padDhDone) <= DELTA_TARGET_ALT_THRESHOLD
                    && (Core.Landing.TgtAlt - padDhDone) >= Core.Landing.MinSafeLandingBurnOverheadM())
                {
                    if (_apPullLatch)
                    {
                        _apPullLatch = false;
                        _apPullClearStreak = 0;
                        LogApPullLatch("CLEAR_SHAPED", frac, haveFrac);
                    }
                    return false;
                }
                if (_apPullLatch)
                {
                    if (!haveFrac)
                        ApNeedsPullBackRaw(targetPosBody, out frac);
                    return true;
                }
                return ApNeedsPullBackRaw(targetPosBody, out frac);
            }

            private bool ApNeedsPullBack(Vector3d targetPosBody)
            {
                return ApNeedsPullBack(targetPosBody, out _);
            }


            private void LogApSeq(string msg)
            {
                double now = Planetarium.GetUniversalTime();
                if (now - _lastApSeqLogUt < 1.0)
                    return;
                _lastApSeqLogUt = now;
                Debug.Log("TargetSubOrbit ApSeq " + msg);
            }

            private void LogApPhase(string phase, string detail)
            {
                double now = Planetarium.GetUniversalTime();
                if (phase == _lastApPhaseLogged && now - _lastApPhaseLogUt < 1.0)
                    return;
                _lastApPhaseLogUt = now;
                _lastApPhaseLogged = phase;
                Debug.Log("TargetSubOrbit phase=" + phase
                    + (string.IsNullOrEmpty(detail) ? "" : (" " + detail)));
            }

            /// <summary>
            /// While shaping: bump TgtAlt so pad-overflight gate climbs over a ridge.
            /// Never bump while Ap pull/dump is active.
            /// </summary>
            private void BumpTgtAltForTerrainClear()
            {
                // Raise hoop clearNeed floor (TerrainHoopFloorM); SyncTgtAltToLandingHoop aims TgtAlt at hoop.
                Vector3d tgt = Core.Target.GetPositionTargetPosition();
                if (!_apReachedForShape)
                    return;
                if (ApNeedsPullBack(tgt))
                    return;
                double now = Planetarium.GetUniversalTime();
                if (now - _lastTerrainRaiseUt < TERRAIN_RAISE_INTERVAL_S)
                    return;
                _lastTerrainRaiseUt = now;
                double raised = Math.Min(
                    TERRAIN_RAISE_MAX_TGTALT_M,
                    Math.Max(Core.Landing.TerrainHoopFloorM, Core.Landing.TgtAlt) + TERRAIN_RAISE_STEP_M);
                if (raised > Core.Landing.TerrainHoopFloorM)
                {
                    Core.Landing.TerrainHoopFloorM = raised;
                    Core.Landing.SyncTgtAltToLandingHoop();
                    LogApSeq("terrain-after-Ap raise hoopFloor=" + Core.Landing.TerrainHoopFloorM.ToString("F0")
                        + " TgtAlt=" + Core.Landing.TgtAlt.ToString("F0"));
                }
            }

            /// <summary>
            /// While shaping: if local or far path clips terrain, raise TgtAlt and climb in
            /// PROGRADE. Stays in TargetSubOrbit - never MoveToTarget, never early LandingBurn.
            /// </summary>
            private const double CA_PADDH_BUMP_CAP_M = 5000.0; // stop raising TgtAlt once padDh already demands big climb
            private const double CA_LOCAL_AGL_M = 2500.0;

            private AutopilotStep ApplyShapingTerrainClearance(Vector3d targetPosBody)
            {
                // 094807: RETRO dump skipped CA (ApNeedsPullBack) and hit terrain at
                // vSpd~-78 / AGL~40 while pad still 87 km away. Local suicide always checked.
                try
                {
                    (double deltaV, bool willCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, 900.0);
                    if ((willCrash || deltaV > 0)
                        && VesselState.altitudeTrue < CA_LOCAL_AGL_M
                        && VesselState.speedVertical < -5.0)
                    {
                        Debug.Log("[TSO->MTT] infeasible: local collision while "
                            + step + " AGL=" + VesselState.altitudeTrue.ToString("F0")
                            + " vSpd=" + VesselState.speedVertical.ToString("F1")
                            + " dV=" + deltaV.ToString("F0")
                            + " pull=" + ApNeedsPullBack(targetPosBody));
                        Core.Thrust.ThrustOff();
                        return new MoveToTarget(Core);
                    }
                }
                catch (System.Exception e)
                {
                    Debug.Log("[TSO-CA] local check failed: " + e.Message);
                }

                if (!_apReachedForShape || ApNeedsPullBack(targetPosBody))
                    return null;

                // k > 1 trip cruise: level hold ASL >= terrain Ap floor and the CA heading scan own
                // terrain (TripCruiseCommand aborts on CA); no TgtAlt bumps. Never true at k == 1.0.
                if (TripCruiseActive)
                    return null;

                // Ricardo order: LATERAL first (point to the pad), then PRO/RETRO shape length and
                // terrain. 20261109_025729: forcing PROGRADE here from INIT skipped LATERAL and
                // left ~20 deg heading error / 41.5 km crossrange at handoff. Local collision
                // check above still applies in every state.
                if (LateralFirstPending())
                    return null;

                bool pathClear;
                try { pathClear = Core.Landing.SubOrbitPathClearToTarget(); }
                catch (System.Exception e)
                {
                    Debug.Log("[TSO-CA] terrain check failed: " + e.Message);
                    return null;
                }
                if (pathClear)
                    return null;

                // Keep TgtAlt at/above gravity-scaled LB overhead floor before applying bump cap.
                // Cap must not freeze a crushed/low TgtAlt that leaves heightOverPad < min gate.
                Core.Landing.EnsureTgtAltMeetsLbOverheadFloor();

                // 094807: CA ratcheted TgtAlt 665->20661 while AGL~2 km, leaving padDh=+10 km
                // permanently out of LB band. Stop bumping once pad already demands a big climb.
                // Exception: still raise while TgtAlt is below the LB overhead floor.
                if (TryPadOverflightGate(targetPosBody, out double padDhCap, out _)
                    && padDhCap > CA_PADDH_BUMP_CAP_M
                    && Core.Landing.TgtAlt >= Core.Landing.MinSafeLandingBurnOverheadM())
                {
                    if (Planetarium.GetUniversalTime() - _lastApSeqLogUt >= 1.0)
                        LogApSeq("CA bump capped padDh=" + padDhCap.ToString("F0")
                            + " TgtAlt=" + Core.Landing.TgtAlt.ToString("F0"));
                    if (step != Step.STATE_PROGRADE)
                    {
                        step = Step.STATE_PROGRADE;
                        throttleCount = 0;
                    }
                    return null;
                }

                // Still blocked at TgtAlt ceiling: TSO cannot shape over terrain — MTT owns CA hop.
                if (Core.Landing.TgtAlt >= TERRAIN_RAISE_MAX_TGTALT_M - 1.0)
                {
                    Debug.Log("[TSO->MTT] infeasible: path blocked at max TgtAlt="
                        + Core.Landing.TgtAlt.ToString("F0")
                        + " AGL=" + VesselState.altitudeTrue.ToString("F0"));
                    Core.Thrust.ThrustOff();
                    return new MoveToTarget(Core);
                }

                BumpTgtAltForTerrainClear();
                if (step != Step.STATE_PROGRADE)
                {
                    step = Step.STATE_PROGRADE;
                    throttleCount = 0;
                }
                if (Planetarium.GetUniversalTime() - _lastApSeqLogUt >= 1.0)
                {
                    LogApSeq("CA shape raise path-blocked TgtAlt="
                        + Core.Landing.TgtAlt.ToString("F0")
                        + " AGL=" + VesselState.altitudeTrue.ToString("F0"));
                }
                return null;
            }

            /// <summary>
            /// TSO done: pad overflight near TgtAlt, Ap not past / no pull, path clear.
            /// Vertical crest (v~0 at Ap) is valid for a shaped ballistic path — LB owns
            /// braking. Do not require arbitrary v<-0.5 (101149 blocked for minutes).
            /// </summary>
            private bool TryDirectLandingBurnHandoff(Vector3d targetPosBody, out string reason)
            {
                reason = "";
                if (TripCruiseActive)
                    return TryTripCruiseHandoff(targetPosBody, out reason); // k > 1 only
                if (!_apReachedForShape)
                {
                    reason = "shape-not-ready";
                    return false;
                }
                if (ApNeedsPullBack(targetPosBody, out double frac))
                {
                    reason = "needs-pull frac=" + (double.IsNaN(frac) ? "n/a" : frac.ToString("F2"));
                    return false;
                }
                if (PadOverflightBehind(targetPosBody))
                {
                    reason = "pad-behind";
                    return false;
                }
                if (ApoapsisPastDestination(targetPosBody))
                {
                    reason = "ap-past-pad";
                    AutotestHoopCenter = double.NaN;
                    AutotestPredH = double.NaN;
                    AutotestHoopMiss = double.NaN;
                    return false;
                }

                // Landing hoop: unpowered coast through gate-above-pad (ComputeOptimal) raised
                // for inbound terrain clearance. Replaces padDh-vs-TgtAlt band + crest/vSpd.
                if (!Core.Landing.TryEvaluateLandingHoop(
                        out double hoopCenter, out double gate, out double clearNeed,
                        out double predH, out double miss, out double predDt,
                        out bool pathHits, out bool hoopOk, out string hoopWhy))
                {
                    reason = "hoop-eval-failed " + hoopWhy;
                    return false;
                }
                AutotestHoopCenter = hoopCenter;
                AutotestPredH = predH;
                AutotestHoopMiss = miss;
                AutotestPadDh = double.IsNaN(predH) || double.IsNaN(hoopCenter) ? double.NaN : (hoopCenter - predH);
                AutotestPadDt = predDt;

                if (!hoopOk || pathHits)
                {
                    reason = "hoop-fail " + hoopWhy + " apPast=0";
                    return false;
                }

                // Shared intercept / path checks; SKIP far-range brake gate.
                if (!Core.Landing.LandingBurnEntryAllowedFromTargetSubOrbit(
                        out bool localBlocked, out bool pathBlocked, out string gateDetail))
                {
                    reason = "lb-gate " + gateDetail
                        + " local=" + localBlocked + " path=" + pathBlocked
                        + " hoopCenter=" + hoopCenter.ToString("F0")
                        + " predH=" + predH.ToString("F0");
                    return false;
                }

                reason = "hoopCenter=" + hoopCenter.ToString("F0")
                    + " gate=" + gate.ToString("F0")
                    + " clearNeed=" + clearNeed.ToString("F0")
                    + " predH=" + predH.ToString("F0")
                    + " miss=" + miss.ToString("F0")
                    + " apPast=0"
                    + " ok=1"
                    + (hoopWhy.StartsWith("ok terrain-hoop") ? " mode=terrain-hoop" : " mode=gate-band")
                    + " frac=" + (TryApAlongTrackFraction(targetPosBody, out double fOut, out _) ? fOut.ToString("F2") : "n/a")
                    + " " + gateDetail;
                return true;
            }


            /// <summary>
            /// Ricardo: once TargetSubOrbit is entered, stay and shape pad overflight to TgtAlt.
            /// MoveToTarget owns the initial coast-to-Ap handoff only. If Ap ends past pad:
            /// RETROGRADE dump in place, then continue shaping - never hand back for Ap/coast.
            /// Returns non-null only if Drive should switch steps (no longer returns MoveToTarget).
            /// </summary>
            // INIT/LATERAL own direction first (Ricardo order 1). Ap-dump and terrain shaping
            // are PRO/RETRO work and must not pre-empt them.
            private bool LateralFirstPending()
            {
                return step == Step.STATE_INIT || step == Step.STATE_LATERAL;
            }

            // Skip ClosestPoint samples at tUT~now (Ap flip branch: floor/far checks
            // return "clear" there, LA:1555/1692). Sticky: never clears inside TSO.
            private void SampleInitTerrainDecision()
            {
                if (_terrainBlockedAtInit) return;
                if (!Core.Landing.TryPredictedHeightOverPad(out _, out double dtT, out _) || dtT <= 1.0)
                    return;
                _terrainInitSampleValid = true;
                _terrainInitLastPredDt = dtT;
                try { _terrainBlockedAtInit = !Core.Landing.SubOrbitPathClearToTarget(); }
                catch { return; }
                if (_terrainBlockedAtInit)
                    Debug.Log("[TSO-init] terrain blocks coast -> PROGRADE after LATERAL step=" + step
                        + " predDt=" + dtT.ToString("F0"));
            }

            private AutopilotStep ApplyApOwnershipGate(Vector3d targetPosBody)
            {
                // Hyperbola / Inf Ap: keep existing dump / shape path in place.
                if (!MechJebModuleLandingAutopilot.ApoapsisIsFinite(Vessel.orbit))
                {
                    _establishingAp = ApNeedsPullBack(targetPosBody);
                    if (_establishingAp)
                    {
                        // Hold shape flag off so dump is not fought by PROGRADE climbs.
                        _apReachedForShape = false;
                        // Ricardo order: LATERAL runs first; its exit chooses RETROGRADE
                        // (hyperbolic / ApNeedsPullBack). Do not pre-empt INIT/LATERAL here.
                        if (step != Step.STATE_RETROGRADE && !LateralFirstPending())
                        {
                            step = Step.STATE_RETROGRADE;
                            throttleCount = 0;
                        }
                        LogApPhase("DUMP", "InfAp RETROGRADE dump");
                    }
                    else
                    {
                        _apReachedForShape = true;
                        LogApPhase("SHAPE", "InfAp shape");
                    }
                    return null;
                }

                bool haveFrac = TryApAlongTrackFraction(targetPosBody, out double frac, out double tAp);
                bool needsPull = ApNeedsPullBack(targetPosBody, out double pullFrac);
                if (needsPull && !haveFrac && !double.IsNaN(pullFrac))
                {
                    haveFrac = true;
                    frac = pullFrac;
                }
                string fracS = haveFrac ? frac.ToString("F2") : "n/a";

                // Ap past pad / frac>~0.5: RETROGRADE dump in place. Stay in TargetSubOrbit.
                if (needsPull)
                {
                    _apReachedForShape = false;
                    _establishingAp = true;
                    // Ricardo order: LATERAL runs first; its exit chooses RETROGRADE via
                    // ApNeedsPullBack. Do not pre-empt INIT/LATERAL here.
                    if (step != Step.STATE_RETROGRADE && !LateralFirstPending())
                    {
                        step = Step.STATE_RETROGRADE;
                        throttleCount = 0;
                    }
                    bool past = ApoapsisPastDestination(targetPosBody);
                    LogApPhase("DUMP", (past ? "RETROGRADE Ap past pad frac=" : "RETROGRADE pull Ap halfway frac=") + fracS);
                    LogApSeq((past ? "RETROGRADE Ap past pad frac=" : "RETROGRADE pull Ap halfway frac=") + fracS);
                    return null;
                }

                // Ap before pad (or pull done): shape to TgtAlt at pad - never hand back to MoveToTarget.
                if (!_apReachedForShape || _establishingAp)
                    LogApSeq("shape TargetSubOrbit TgtAlt at pad frac=" + fracS);
                _apReachedForShape = true;
                _establishingAp = false;
                if (step == Step.STATE_COAST_AP)
                {
                    Core.Landing.NoteWarp("TSO exit:coast-ap->lateral");
                    Core.Warp.MinimumWarp();
                    step = Step.STATE_LATERAL;
                    throttleCount = 0;
                }
                LogApPhase("SHAPE", "frac=" + fracS + " step=" + step);
                return null;
            }

            private void MaybeLogGuidanceTick(Vector3d thrustDir)
            {
                double now = Planetarium.GetUniversalTime();
                Vector3d targetPos = Core.Target.GetPositionTargetPosition();
                double padDh = double.NaN, padDt = double.NaN;
                bool havePad = TryPadOverflightGate(targetPos, out padDh, out padDt);
                double frac = double.NaN;
                bool haveFrac = TryApAlongTrackFraction(targetPos, out frac, out _);
                bool apPast = ApoapsisPastDestination(targetPos);
                string burnAxis = DescribeBurnAxis(thrustDir, _lastHerror, _lastDownrange);

                AutotestStepName = step.ToString();
                AutotestPadDh = havePad ? padDh : double.NaN;
                AutotestPadDt = havePad ? padDt : double.NaN;
                AutotestApFrac = haveFrac ? frac : double.NaN;
                AutotestApPast = apPast;
                AutotestBurnAxis = burnAxis;

                string handoff = "";
                if (step != _prevLoggedStep)
                {
                    handoff = _prevLoggedStep + "->" + step;
                    AutotestHandoff = handoff;
                    Debug.Log("[TSO-handoff] " + handoff
                        + " dist=" + distanceToTarget.ToString("F0")
                        + " padDh=" + (havePad ? padDh.ToString("F0") : "n/a")
                        + " frac=" + (haveFrac ? frac.ToString("F2") : "n/a")
                        + " apPast=" + apPast);
                    _prevLoggedStep = step;
                }
                else
                {
                    AutotestHandoff = "";
                }

                if (now - _lastGuidanceLogUt < 1.0)
                    return;
                _lastGuidanceLogUt = now;

                double dist = double.NaN;
                try { dist = Core.Landing.getHDistanceToTarget(); } catch { dist = distanceToTarget; }

                Debug.Log("[TSO-1Hz] step=" + step
                    + " padDh=" + (havePad ? padDh.ToString("F0") : "n/a")
                    + " padDt=" + (havePad ? padDt.ToString("F1") : "n/a")
                    + " dist=" + (double.IsNaN(dist) ? "n/a" : dist.ToString("F0"))
                    + " hSpd=" + VesselState.speedSurfaceHorizontal.ToString("F1")
                    + " vSpd=" + VesselState.speedVertical.ToString("F1")
                    + " ApA=" + VesselState.orbitApA.ToString("F0")
                    + " frac=" + (haveFrac ? frac.ToString("F2") : "n/a")
                    + " apPast=" + apPast
                    + " burn=" + burnAxis
                    + " thr=" + lastHorizontalThrust.ToString("F2")
                    + " TgtAlt=" + Core.Landing.TgtAlt.ToString("F0")
                    + (string.IsNullOrEmpty(handoff) ? "" : (" handoff=" + handoff)));
            }

            private string DescribeBurnAxis(Vector3d thrustDir, Vector3d herror, Vector3d downrange)
            {
                if (step == Step.STATE_PROGRADE) return "prograde";
                if (step == Step.STATE_TRIP_HOOP) return "trip-hoop";
                if (step == Step.STATE_TRIP_DESCEND) return "trip-descend";
                if (step == Step.STATE_RETROGRADE) return "retrograde";
                if (step == Step.STATE_CORRECT) return "idle-correct";
                if (step == Step.STATE_COAST_AP) return "coast-ap";
                if (step == Step.STATE_LATERAL_FINE)
                {
                    double angN = Vector3d.Angle(VesselState.normalPlus, herror == Vector3d.zero ? VesselState.normalPlus : herror);
                    return angN > 90 ? "lateral-normalMinus" : "lateral-normalPlus";
                }
                if (step == Step.STATE_LATERAL)
                    return "lateral-crossrange";

                Vector3d t = thrustDir.sqrMagnitude > 1e-12 ? thrustDir.normalized : Vector3d.zero;
                if (t == Vector3d.zero) return "none";
                double upDot = Vector3d.Dot(t, VesselState.up);
                Vector3d th = Vector3d.Exclude(VesselState.up, t);
                double drDot = 0;
                if (downrange.sqrMagnitude > 1e-12 && th.sqrMagnitude > 1e-12)
                    drDot = Vector3d.Dot(th.normalized, downrange.normalized);
                if (Math.Abs(upDot) > 0.85) return upDot > 0 ? "radial-out" : "radial-in";
                if (drDot > 0.5) return "prograde-ish";
                if (drDot < -0.5) return "retrograde-ish";
                return "lateral-ish";
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
                deltaDistance = closestDistance.magnitude - targetDistance.magnitude; // Difference in distance to target and closest approach
                deltaHeight = Core.Landing.TgtAlt - height;

                switch (step)
                {
                    case Step.STATE_COAST_AP:
                        // Legacy enum value only - ApplyApOwnershipGate clears to LATERAL; no coast ownership.
                        forceHoriz = true;
                        desiredHorizontalVel = Vector3d.zero;
                        herror = Vector3d.zero;
                        rc = 0;
                        step = Step.STATE_LATERAL;
                        break;
                    default:
                    case Step.STATE_INIT: // Initial run - setup for lateral burn
                        forceHoriz = true; // Force horizontal thrust
                        step = Step.STATE_LATERAL;
                        if (!_terrainBlockedAtInit)
                            Debug.Log("[TSO-init] INIT done: " + (_terrainInitSampleValid ? "valid sample clear" : "no valid sample yet")
                                + " predDt=" + (double.IsNaN(_terrainInitLastPredDt) ? "n/a" : _terrainInitLastPredDt.ToString("F0"))
                                + " -> LATERAL re-samples");
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
                            bool heightFinite = !double.IsNaN(deltaHeight) && !double.IsInfinity(deltaHeight)
                                && !double.IsNaN(height) && !double.IsInfinity(height);
                            bool hyperbolic = !MechJebModuleLandingAutopilot.ApoapsisIsFinite(Vessel.orbit);
                            // Inf apoapsis / NaN closest approach: do not default to prograde
                            // (that adds energy and there is no Ap to raise). Retrograde until
                            // intercept math has a finite closest approach near cruise alt.
                            if (!heightFinite)
                                step = Step.STATE_RETROGRADE;
                            else if (hyperbolic && (deltaHeight < -DELTA_TARGET_ALT_THRESHOLD))
                                step = Step.STATE_RETROGRADE;
                            else if (hyperbolic && (VesselState.speedVertical > 1) && (deltaDistance < -1000))
                                step = Step.STATE_RETROGRADE;
                            else if (ApNeedsPullBack(targetPos))
                                step = Step.STATE_RETROGRADE; // Ap past / frac>~0.5: pull back
                            else if (_terrainBlockedAtInit)
                                step = Step.STATE_PROGRADE;   // terrain blocks coast: RETRO only shortens/lowers it
                            else if (TryPadOverflightGate(targetPos, out double padDhLat, out _)
                                     && padDhLat < -DELTA_TARGET_ALT_THRESHOLD)
                                step = Step.STATE_RETROGRADE; // overflight too high
                            else if ((deltaHeight < -DELTA_TARGET_ALT_THRESHOLD) && (deltaDistance > -2000))
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

                            // Reduce the error as we approach periapsis to avoid over correction.
                            // (0.5*ApA - PeA)/(0.01*ApA) is Inf/Inf = NaN on hyperbola and NaN'd herror.
                            double peScale;
                            if (MechJebModuleLandingAutopilot.ApoapsisIsFinite(Vessel.orbit)
                                && Math.Abs(VesselState.orbitApA) > 1)
                            {
                                peScale = Mathf.Clamp01((float)((0.5 * VesselState.orbitApA - VesselState.orbitPeA) / (0.01 * VesselState.orbitApA)));
                            }
                            else
                            {
                                double tPe = Vessel.orbit.timeToPe;
                                if (double.IsNaN(tPe) || double.IsInfinity(tPe) || tPe <= 0)
                                    peScale = 0;
                                else
                                    peScale = Mathf.Clamp01((float)(tPe / 60.0));
                            }
                            herror = 20 * herror * peScale;

                            Debug.Log(" prograde deltaHeight:" + deltaHeight.ToString("F0") + "  distance:" + distanceToTarget.ToString("F0") + "  deltaDistance" + deltaDistance.ToString("F1"));

                            // Switch to fine lateral control when close to target or if periapsis is above half target altitude
                            // Ap past / frac>~0.5: do not fine-tune / CORRECT - pull Ap back first.
                            if (ApNeedsPullBack(targetPos))
                            {
                                ProgradeCount = throttleCount;
                                throttleCount = 0;
                                step = Step.STATE_RETROGRADE;
                            }
                            else if (_establishingAp && ApApproximatelyHalfwayBeforePad(targetPos))
                            {
                                // Ap ~halfway: stop placing; stay in TargetSubOrbit and continue shaping.
                                ProgradeCount = throttleCount;
                                throttleCount = 0;
                                _establishingAp = false;
                            }
                            else if (_establishingAp)
                            {
                                // Keep PROGRADE until halfway band ├óΓé¼ΓÇ¥ do not LATERAL_FINE mid-place.
                            }
                            else if (Core.Landing.TripSpeedOn && _apReachedForShape && !_tripCruise
                                     && TripFloorReached() && TryStartTripCruise("trip floor reached"))
                            {
                                // k > 1 only (Ricardo t257u): at the trip floor the powered level cruise starts
                                // without waiting for an unpowered coast to clear the k == 1.0 LB-floor margin.
                                // k > 1: TryStartTripCruise enters STATE_TRIP_HOOP; TripCruiseCommand owns the burn.
                            }
                            else if (_apReachedForShape && TerrainHoopShaped()
                                     && TerrainClearWithExitMargin())
                            {
                                if (Core.Landing.TripSpeedOn && TryStartTripCruise("terrain-hoop path clear"))
                                {
                                    // k > 1: TryStartTripCruise enters STATE_TRIP_HOOP; TripCruiseCommand owns the burn.
                                }
                                else
                                {
                                // Terrain-driven hoop and inbound coast now clears terrain:
                                // nothing better to shape in height (Nov 9). Finish crossrange.
                                ProgradeCount = throttleCount;
                                throttleCount = 0;
                                Debug.Log("PROGRADE exit -> LATERAL_FINE terrain-hoop path clear");
                                step = Step.STATE_LATERAL_FINE;
                                }
                            }
                            else if (_apReachedForShape && ((throttleCount >= 4) || (Orbit.PeA > 0.5 * Core.Landing.TargetAltitude) || ((distanceToTarget < TARGET_FAR_DISTANCE_THRESHOLD) && (deltaHeight < -DELTA_TARGET_ALT_THRESHOLD))))
                            {
                                // 091142: ClosestPoint deltaHeight was high while padDh still
                                // +11 km (TgtAlt CA-bumped). Do not leave PROGRADE for FINE
                                // until pad overflight is no longer too low.
                                bool padStillTooLow = TryPadOverflightGate(targetPos, out double padDhPg, out _)
                                    && padDhPg > DELTA_TARGET_ALT_THRESHOLD;
                                if (!padStillTooLow)
                                {
                                    if (Core.Landing.TripSpeedOn && TryStartTripCruise("prograde done"))
                                    {
                                        // k > 1: TryStartTripCruise enters STATE_TRIP_HOOP; TripCruiseCommand owns the burn.
                                    }
                                    else
                                    {
                                    ProgradeCount = throttleCount;
                                    throttleCount = 0;
                                    step = Step.STATE_LATERAL_FINE;
                                    }
                                }
                            }
                            // Calculate horizontal correction angle using horizontal herror.
                            rc = Math.Max(0, Math.Min(_hCorrectionUpAngle, baseGain * H_CORRECTION_ANGLE_CONSTANT * herror.magnitude));
                        }
                        break;
                    case Step.STATE_RETROGRADE:
                        // Closest position is close target - do final retrograde burn to get it to pass close to target altitude
                        {
                            // 192439: forceHoriz=false used |herror|/10000 thr uncapped -> thr=1
                            // reversed H 5->1235 m/s. Same g/a_max thrCap as LATERAL via forceHoriz.
                            forceHoriz = true;

                            double hOrbitalVelocity = Vector3d.Exclude(VesselState.up, VesselState.orbitalVelocity).magnitude; // Get current horizontal orbital velocity
                            double desiredHorizontal = 0.9*hOrbitalVelocity;
                            desiredHorizontalVel = desiredHorizontal * downrangeVec.normalized;

                            // Burn retrograde until height at closest position is close to target altitude, but only if it's crosses at a higher altitude.
                            Debug.Log(" retrograde deltaHeight:" + deltaHeight.ToString("F0") + "  distance:" + distanceToTarget.ToString("F0") + "  deltaDistance" + deltaDistance.ToString("F1"));

                            // Ap past pad or frac > ~0.5: hard-hold RETROGRADE (21:12 tooLow
                            // hopped PROGRADE one tick after ApSeq frac=2.57, undoing the dump).
                            bool needsPull = ApNeedsPullBack(targetPos, out double apFracExit);

                            // Always anti-downrange in RETROGRADE. Signed deltaHeight became
                            // prograde once height crossed, which left Ap past the pad.
                            double retroMag = Math.Abs(deltaHeight);
                            if (needsPull)
                                retroMag = Math.Max(retroMag, 2000.0);
                            else if (retroMag < 1.0)
                                retroMag = 1.0;

                            // Surface H toward pad; never reverse past zero relative to pad.
                            Vector3d hVelSurf = Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);
                            double towardPad = Vector3d.Dot(hVelSurf, downrangeVec.normalized);
                            double circH = Core.Landing.GetCircularOrbitSpeed(
                                Math.Max(0.0, VesselState.altitudeASL), MainBody);
                            double hNullEps = Math.Max(2.0, 0.02 * Math.Max(1.0, circH));
                            bool hNearNeeded = towardPad <= hNullEps;
                            if (towardPad <= 0)
                            {
                                // Already nulled or reversed past pad direction: kill anti-H burn.
                                herror = Vector3d.zero;
                            }
                            else
                            {
                                herror = -retroMag * downrangeVec.normalized;
                            }
                            rc = 100; // we only want horizontal thrust here

                            // Exit on pad-overflight height gate when pad still ahead.
                            // Band around TgtAlt (|padDh| <= 600): low side required so
                            // LandingBurn ZEM-ZEV has room. Keep dumping while ApNeedsPullBack;
                            // ClosestPoint/intercept past allowed. Pad behind: never OK.
                            bool heightOk;
                            bool tooLow = false;
                            double padDhExit = double.NaN;
                            if (TryPadOverflightGate(targetPos, out padDhExit, out _))
                            {
                                // Band: TgtAlt-600 <= heightOverPad <= TgtAlt+600 (|padDh| <= 600).
                                heightOk = Math.Abs(padDhExit) <= DELTA_TARGET_ALT_THRESHOLD;
                                tooLow = padDhExit > DELTA_TARGET_ALT_THRESHOLD;
                            }
                            else if (PadOverflightBehind(targetPos))
                                heightOk = false;
                            else
                            {
                                // Pad ahead but overflight gate unavailable: do NOT treat
                                // ClosestPoint |deltaHeight| as OK - that point may be tens of
                                // km past the pad while Ap is still past (20:18: dh -599 with
                                // deltaDistance +45 km). Keep other exits (throttleCount>=4).
                                heightOk = false;
                                tooLow = false;
                            }

                            // Clean exit: padDh in band and H toward pad near needed/zero.
                            // Clears wild-frac latch without waiting for frac in [0.35,0.5].
                            if (heightOk && hNearNeeded && !ApoapsisPastDestination(targetPos))
                            {
                                if (_apPullLatch)
                                {
                                    _apPullLatch = false;
                                    _apPullClearStreak = 0;
                                    LogApPullLatch("CLEAR_H", apFracExit, !double.IsNaN(apFracExit));
                                }
                                RetrogradeCount = throttleCount;
                                throttleCount = 0;
                                _establishingAp = false;
                                double nowH = Planetarium.GetUniversalTime();
                                if (nowH - _lastRetroExitLogUt >= 1.0)
                                {
                                    _lastRetroExitLogUt = nowH;
                                    Debug.Log("RETROGRADE exit -> "
                                        + (tooLow ? "PROGRADE" : "LATERAL_FINE")
                                        + " padDh=" + (double.IsNaN(padDhExit) ? "n/a" : padDhExit.ToString("F0"))
                                        + " towardPad=" + towardPad.ToString("F1")
                                        + " hNullEps=" + hNullEps.ToString("F1")
                                        + " frac=" + (double.IsNaN(apFracExit) ? "n/a" : apFracExit.ToString("F2")));
                                }
                                step = tooLow ? Step.STATE_PROGRADE : Step.STATE_LATERAL_FINE;
                                break;
                            }
                            // Terrain-driven hoop, path clear, no Ap pull: no height band to hit
                            // (Nov 9) - finish crossrange in LATERAL_FINE.
                            if (!needsPull && !_apPullLatch && _apReachedForShape && !_establishingAp
                                && !ApoapsisPastDestination(targetPos) && TerrainHoopShaped())
                            {
                                RetrogradeCount = throttleCount;
                                throttleCount = 0;
                                double nowT = Planetarium.GetUniversalTime();
                                if (nowT - _lastRetroExitLogUt >= 1.0)
                                {
                                    _lastRetroExitLogUt = nowT;
                                    Debug.Log("RETROGRADE exit -> LATERAL_FINE terrain-hoop path clear"
                                        + " padDh=" + (double.IsNaN(padDhExit) ? "n/a" : padDhExit.ToString("F0")));
                                }
                                step = Step.STATE_LATERAL_FINE;
                                break;
                            }
                            if (needsPull || _apPullLatch)
                            {
                                // Stay RETROGRADE while Ap past / latch held. No shape climbs.
                                _establishingAp = true;
                            }
                            else if (_establishingAp && ApApproximatelyHalfwayBeforePad(targetPos))
                            {
                                // Pull done / Ap before pad: stay in TargetSubOrbit and shape to TgtAlt.
                                RetrogradeCount = throttleCount;
                                throttleCount = 0;
                                _establishingAp = false;
                                double nowExit = Planetarium.GetUniversalTime();
                                if (nowExit - _lastRetroExitLogUt >= 1.0)
                                {
                                    _lastRetroExitLogUt = nowExit;
                                    Debug.Log("RETROGRADE exit -> shape stay TargetSubOrbit Ap halfway"
                                        + " frac=" + (double.IsNaN(apFracExit) ? "n/a" : apFracExit.ToString("F2"))
                                        + " deltaHeight=" + deltaHeight.ToString("F0")
                                        + " deltaDistance=" + deltaDistance.ToString("F0"));
                                }
                                if (tooLow)
                                    step = Step.STATE_PROGRADE;
                                else if (heightOk)
                                    step = Step.STATE_LATERAL_FINE;
                            }
                            else if (!_apReachedForShape)
                            {
                                // During dump: hold RETROGRADE - never LATERAL_FINE / tooLow PROGRADE.
                                _establishingAp = true;
                            }
                            else if (tooLow && !_establishingAp && !_apPullLatch)
                            {
                                // Below TgtAlt-600 after Ap: climb/shape via PROGRADE.
                                RetrogradeCount = throttleCount;
                                throttleCount = 0;
                                step = Step.STATE_PROGRADE;
                            }
                            else if (!_establishingAp && !_apPullLatch && ((throttleCount >= 4) || heightOk))
                            {
                                RetrogradeCount = throttleCount;
                                throttleCount = 0;
                                double nowExit = Planetarium.GetUniversalTime();
                                if (nowExit - _lastRetroExitLogUt >= 1.0)
                                {
                                    _lastRetroExitLogUt = nowExit;
                                    Debug.Log("RETROGRADE exit -> LATERAL_FINE heightOk=" + heightOk
                                        + " throttleCount=" + RetrogradeCount
                                        + " deltaHeight=" + deltaHeight.ToString("F0")
                                        + " deltaDistance=" + deltaDistance.ToString("F0"));
                                }
                                // Now will perform final correction step.
                                step = Step.STATE_LATERAL_FINE;
                            }
                        }
                        break;
                    case Step.STATE_TRIP_HOOP:
                    case Step.STATE_TRIP_DESCEND:
                        // TripCruiseCommand owns the burn while _tripCruise is set; this is only a
                        // safety no-op if SetPathToTarget is reached with these steps.
                        forceHoriz = false;
                        desiredHorizontalVel = Vector3d.zero;
                        herror = Vector3d.zero;
                        rc = 0;
                        break;
                    case Step.STATE_LATERAL_FINE: // Fine tune horizontal velocity to target using closest approach vector instead of crossRange vector
                        {
                            forceHoriz = true; // Force horizontal thrust
                            desiredHorizontalVel = Vector3d.zero;

                            herror = calculateHerror(ref closestPos, ref targetPos, ref distanceToTarget, ref surfaceVel);
                            herror = (Vector3d.Angle(VesselState.normalPlus, herror) > 90) ? -VesselState.normalPlus : VesselState.normalPlus;
                            MechJebModuleDebugArrows.debugVector2 = herror;
                            Debug.Log("Lateral Fine HError Mag:" + distanceToTarget.ToString("F0"));

                            // Hard sequence: never fine-tune before Ap reached for shape.
                            if (!_apReachedForShape)
                            {
                                LateralFineCount = throttleCount;
                                throttleCount = 0;
                                // During dump / before shape flag: dump if needed; else stay LATERAL (no handback).
                                step = (_apPullLatch || ApNeedsPullBack(targetPos)) ? Step.STATE_RETROGRADE : Step.STATE_LATERAL;
                                break;
                            }

                            // Do not declare CORRECT while Ap past pad or frac > ~0.5.
                            if (ApNeedsPullBack(targetPos, out double apFracFine))
                            {
                                LateralFineCount = throttleCount;
                                throttleCount = 0;
                                double nowBounce = Planetarium.GetUniversalTime();
                                if (nowBounce - _lastApPastBounceLogUt >= 1.0)
                                {
                                    _lastApPastBounceLogUt = nowBounce;
                                    Debug.Log("LATERAL_FINE bounce -> RETROGRADE Ap needs pull"
                                        + " frac=" + (double.IsNaN(apFracFine) ? "n/a" : apFracFine.ToString("F2"))
                                        + " deltaHeight=" + deltaHeight.ToString("F0")
                                        + " deltaDistance=" + deltaDistance.ToString("F0"));
                                }
                                step = Step.STATE_RETROGRADE;
                            }
                            else if ((throttleCount >= 4) || (distanceToTarget < TARGET_DISTANCE_THRESHOLD) || (herror == Vector3d.zero) )
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

            // ================= Trip-speed lever (k > 1 only) =================

            private void LogTrip(string msg, bool force = false)
            {
                double now = Planetarium.GetUniversalTime();
                if (!force && now - _lastTripLogUt < 1.0)
                    return;
                if (!force)
                    _lastTripLogUt = now; // forced gate lines (own 1 s eval timer) don't starve the cruise line
                Debug.Log("[TripSpeed] " + msg);
            }

            /// LandingBurn ignition distance on LB's own budget (LandingBurn.ComputeBrakeBudget /
            /// ShouldStartBurn d_horiz; same formula as LA CoastBelowLbFloorBeforeIgnition).
            private double TripIgnitionDistance(double vH)
            {
                double aMax = Math.Max(VesselState.limitedMaxThrustAccel, 0.1);
                double g = Math.Max(VesselState.localg, 0.05);
                double aPlan = Math.Min(8.0 * g, 0.7 * aMax);
                aPlan = Math.Max(aPlan, g + 0.05);
                aPlan = Math.Min(aPlan, aMax);
                double bm = Core.Landing.BurnMarginPerc / 100.0;
                return Math.Max(MechJebModuleLandingAutopilot.LANDING_BURN_AGL_FLOOR_M,
                    (1.0 + bm) * vH * vH / (2.0 * aPlan));
            }

            private double TripPlanAccel()
            {
                double aMax = Math.Max(VesselState.limitedMaxThrustAccel, 0.1);
                double g = Math.Max(VesselState.localg, 0.05);
                double aPlan = Math.Min(8.0 * g, 0.7 * aMax);
                aPlan = Math.Max(aPlan, g + 0.05);
                return Math.Min(aPlan, aMax);
            }

            /// <summary>
            /// Step 4 entry (k > 1 only): called where PROGRADE would exit to LATERAL_FINE (path to the
            /// pad clear). Latches vClear, vTarget = min(k vClear, LB-braking cap) and the hold ASL =
            /// max(ASL, Ap (<= +2 km), terrain Ap floor). Refuses (today's exit runs) when CA is not quiet,
            /// the hold would need a big climb, or the speed-up is negligible.
            /// </summary>
            private bool TryStartTripCruise(string why)
            {
                if (!Core.Landing.TripSpeedOn || _tripCruise)
                    return false;
                try
                {
                    double k = Core.Landing.TripSpeedK;
                    double vClear = VesselState.speedSurfaceHorizontal;
                    double asl = VesselState.altitudeASL;
                    // Ricardo t257u: hold = trip cruise floor max(ridge ahead + 2.5 km, pad + 4.2 km),
                    // replacing max(asl, min(Ap, asl + 2 km), ridge + 900 + gate). This function only
                    // runs at k > 1 (TripSpeedOn checked above).
                    double terrainFloor = Core.Landing.TripCruiseFloorAsl(out double ridgeAsl, out double padAsl);
                    if (double.IsNaN(terrainFloor) || double.IsInfinity(terrainFloor) || !(terrainFloor > 0))
                    {
                        LogTrip("TSO cruise skip (" + why + "): no trip floor", true);
                        return false;
                    }
                    double hold = terrainFloor;
                    if (hold > asl + TRIP_HOLD_MAX_RAISE_M)
                    {
                        LogTrip("TSO cruise skip (" + why + "): hold " + hold.ToString("F0") + " >> asl " + asl.ToString("F0"), true);
                        return false;
                    }
                    double dist = Core.Landing.getHDistanceToTarget();
                    double bm = Core.Landing.BurnMarginPerc / 100.0;
                    // LB braking cap: ignition distance at vTarget within half of the remaining distance.
                    double vBrakeCap = Math.Sqrt(2.0 * TripPlanAccel() * 0.5 * Math.Max(dist, 0.0) / (1.0 + bm));
                    // Speed reference: surface circular speed at the hold height (was vClear).
                    double vRef = TripSurfaceCircularSpeed(hold);
                    double vTarget = Math.Min(k * vRef, vBrakeCap);
                    if (!(vTarget > vClear + 1.0))
                    {
                        LogTrip("TSO cruise skip (" + why + "): vTarget " + vTarget.ToString("F1") + " vClear " + vClear.ToString("F1"), true);
                        return false;
                    }
                    (double caDv, bool caCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, SUBORBIT_PATH_MIN_ALTITUDE_M);
                    if (caCrash || caDv > 0)
                    {
                        LogTrip("TSO cruise skip (" + why + "): CA not quiet dV=" + caDv.ToString("F0"), true);
                        return false;
                    }
                    _tripCruise = true;
                    step = Step.STATE_TRIP_HOOP;
                    _tripVClear = vClear;
                    _tripVTarget = vTarget;
                    _tripHoldAsl = hold;
                    LogTrip("TSO cruise START (" + why + ") k=" + k.ToString("F2")
                        + " vClear=" + vClear.ToString("F1") + " vTarget=" + vTarget.ToString("F1")
                        + " vRef=" + vRef.ToString("F1")
                        + " vBrakeCap=" + vBrakeCap.ToString("F1") + " holdAsl=" + hold.ToString("F0")
                        + " asl=" + asl.ToString("F0") + " tripFloor=" + terrainFloor.ToString("F0")
                        + " ridge=" + ridgeAsl.ToString("F0") + " pad=" + padAsl.ToString("F0")
                        + " dist=" + dist.ToString("F0") + " dIgnAtTarget=" + TripIgnitionDistance(vTarget).ToString("F0"), true);
                    return true;
                }
                catch (Exception e)
                {
                    LogTrip("TSO cruise start failed: " + e.Message, true);
                    return false;
                }
            }

            /// <summary>
            /// k > 1 only: surface-relative circular speed along the pad bearing at holdAsl
            /// (vCirc(R + hold) minus the surface rotation component along the pad bearing, scaled to hold radius).
            /// </summary>
            private double TripSurfaceCircularSpeed(double holdAsl)
            {
                Vector3d up = VesselState.up;
                Vector3d posBody = VesselState.CoM - MainBody.position;
                double r = Math.Max(posBody.magnitude, 1.0);
                double rHold = Math.Max(MainBody.Radius + holdAsl, 1.0);
                double vCirc = Math.Sqrt(MainBody.gravParameter / rHold);
                Vector3d tgtBody = Core.Target.GetPositionTargetPosition(); // body-relative
                Vector3d toPad = Vector3d.Exclude(up, tgtBody - posBody);
                Vector3d vRot = Vector3d.Exclude(up, VesselState.orbitalVelocity - VesselState.surfaceVelocity);
                double along = (toPad.sqrMagnitude > 1.0) ? Vector3d.Dot(vRot, toPad.normalized) : 0.0;
                return vCirc - along * rHold / r;
            }

            /// <summary>
            /// k > 1 only (Edit 4): at/near the trip cruise floor (within 150 m). Floor cached 1 s
            /// (192-sample scan); start attempts rate-limited to 1 Hz.
            /// </summary>
            private bool TripFloorReached()
            {
                if (!Core.Landing.TripSpeedOn)
                    return false;
                double now = Planetarium.GetUniversalTime();
                if (now - _tripFloorTryUt < 1.0)
                    return false;
                if (now - _tripFloorUt >= 1.0)
                {
                    _tripFloorUt = now;
                    try { _tripFloorAsl = Core.Landing.TripCruiseFloorAsl(); }
                    catch (Exception) { _tripFloorAsl = 0.0; }
                }
                if (!(_tripFloorAsl > 0))
                    return false;
                if (VesselState.altitudeASL < _tripFloorAsl - 150.0)
                    return false;
                _tripFloorTryUt = now;
                return true;
            }

            private void AbortTripCruise(string why)
            {
                _tripCruise = false;
                ProgradeCount = throttleCount;
                throttleCount = 0;
                step = Step.STATE_LATERAL_FINE; // today's next state; normal Ap/hoop rules resume
                Core.Thrust.ThrustOff();
                LogTrip("TSO cruise ABORT " + why + " -> LATERAL_FINE (normal rules)", true);
            }

            /// <summary>
            /// k > 1 only: PROGRADE -> TRIP_HOOP once cruise is latched, then TRIP_HOOP ->
            /// TRIP_DESCEND when padAsl+hoopCenter has >= 2000 m clearance over the remaining
            /// ridge ahead and the drop is reachable before 0.97*dIgn. hoopOk/pathHits are not
            /// required (unpowered coast is not a valid predictor at cruise H).
            /// </summary>
            private void UpdateTripHoopState()
            {
                if (!TripCruiseActive)
                    return;
                if (step == Step.STATE_PROGRADE)
                    step = Step.STATE_TRIP_HOOP;
                if (step != Step.STATE_TRIP_HOOP && step != Step.STATE_TRIP_DESCEND)
                    return;

                double now = Planetarium.GetUniversalTime();
                bool due = now - _lastTripHoopEvalUt >= 1.0;
                if (!(due || double.IsNaN(_tripTargetAsl)))
                    return;

                _lastTripHoopEvalUt = now;
                if (!TryEvalTripHoopTarget(out double targetAsl, out double ridgeAsl, out double padAsl,
                        out double remClear, out double tAvail, out double vNeed, out string why))
                {
                    LogTrip("TSO hoop wait " + why + " hoopCenter=" + AutotestHoopCenter.ToString("F0"), true);
                    return;
                }
                _tripTargetAsl = targetAsl;
                _tripRidgeAsl = ridgeAsl;

                LogTrip("TSO hoop gate remClear=" + remClear.ToString("F0")
                    + " need=" + TRIP_HOOP_CLEARANCE_M.ToString("F0")
                    + " targetAsl=" + targetAsl.ToString("F0")
                    + " hoopCenter=" + AutotestHoopCenter.ToString("F0")
                    + " remRidge=" + ridgeAsl.ToString("F0")
                    + " pad=" + padAsl.ToString("F0")
                    + " tAvail=" + tAvail.ToString("F0")
                    + " vNeed=" + vNeed.ToString("F2")
                    + " asl=" + VesselState.altitudeASL.ToString("F0")
                    + " step=" + (step == Step.STATE_TRIP_DESCEND ? "descend" : "hoop"), true);
                if (step != Step.STATE_TRIP_HOOP)
                    return;
                if (remClear >= TRIP_HOOP_CLEARANCE_M - 1e-3
                    && tAvail > 5.0
                    && vNeed <= TRIP_DESCEND_VZ_MAX)
                {
                    step = Step.STATE_TRIP_DESCEND;
                    LogTrip("TSO hoop -> DESCEND targetAsl=" + targetAsl.ToString("F0")
                        + " clear=" + remClear.ToString("F0")
                        + " tAvail=" + tAvail.ToString("F0")
                        + " vNeed=" + vNeed.ToString("F2"), true);
                }
            }

            private bool TryEvalTripHoopTarget(out double targetAsl, out double ridgeAsl, out double padAsl,
                out double remClear, out double tAvail, out double vNeed, out string why)
            {
                targetAsl = double.NaN;
                ridgeAsl = double.NaN;
                padAsl = double.NaN;
                remClear = double.NaN;
                tAvail = double.NaN;
                vNeed = double.NaN;
                why = "";
                try
                {
                    // Remaining-path ridge: same great-circle scan as the trip cruise floor
                    // (current position -> pad, pad false-fire excluded).
                    Core.Landing.TripCruiseFloorAsl(out ridgeAsl, out padAsl);
                    if (double.IsNaN(ridgeAsl) || double.IsNaN(padAsl)
                        || double.IsInfinity(ridgeAsl) || double.IsInfinity(padAsl))
                    {
                        why = "no-ridge";
                        return false;
                    }

                    if (!Core.Landing.TryEvaluateLandingHoop(
                            out double hoopCenter, out _, out _,
                            out double predH, out double miss, out _,
                            out _, out _, out string hoopWhy))
                    {
                        why = "hoop-eval-failed " + hoopWhy;
                        return false;
                    }
                    if (double.IsNaN(hoopCenter) || double.IsInfinity(hoopCenter) || hoopCenter <= 0)
                    {
                        why = "no-hoopCenter " + hoopWhy;
                        return false;
                    }
                    // Live columns only; do not require hoopOk / !pathHits.
                    AutotestHoopCenter = hoopCenter;
                    AutotestPredH = predH;
                    AutotestHoopMiss = miss;

                    // Ricardo t294u: never target below remRidge + margin (pad sits in a crater below the rim).
                    targetAsl = Math.Max(padAsl + hoopCenter, ridgeAsl + TRIP_DESCEND_TERRAIN_MARGIN_M);
                    remClear = targetAsl - ridgeAsl;
                    double asl = VesselState.altitudeASL;
                    double vH = Math.Max(VesselState.speedSurfaceHorizontal, 1.0);
                    double dIgn = TripIgnitionDistance(vH);
                    double dist = Core.Landing.getHDistanceToTarget();
                    tAvail = (dist - 0.97 * dIgn) / vH;
                    double drop = asl - targetAsl;
                    vNeed = (tAvail > 0.0 && drop > 0.0) ? drop / tAvail : 0.0;
                    why = "ok";
                    return true;
                }
                catch (Exception e)
                {
                    why = "eval-failed " + e.Message;
                    return false;
                }
            }
            /// <summary>
            /// Steps 3+4 (k > 1 only): powered level cruise toward the pad at _tripVTarget.
            /// Vertical: a_up = g - vOrbH^2/r + 0.2 (vzCmd - vz) (down-thrust when faster than circular),
            /// vzCmd holds _tripHoldAsl; -3 m/s pre-descent within TRIP_PREDESCENT_M of dIgn so LB sees
            /// Vy >= 1 at handoff. Horizontal: 0.1 s^-1 tracking of vTarget along the pad bearing (nulls
            /// crossrange). Accel capped at 8 g (same g/a_max thrCap as every TSO burn).
            /// Aborts to today's rules on CA (15 s / 900 m incl. heading scan) or AGL < coast need.
            /// </summary>
            private void TripCruiseCommand(ref Vector3d desiredThrustVector)
            {
                try
                {
                    Vector3d up = VesselState.up;
                    Vector3d posBody = VesselState.CoM - MainBody.position;
                    double r = posBody.magnitude;
                    double asl = VesselState.altitudeASL;
                    double vz = VesselState.speedVertical;
                    double g = Math.Max(VesselState.localg, 0.05);

                    (double caDv, bool caCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, SUBORBIT_PATH_MIN_ALTITUDE_M);
                    // Ricardo t257u: powered cruise abort = obstacle clearance 1200 m AGL (was 900 + 1.1 * LB floor = 4200,
                    // the pad-relative LB margin applied as AGL). Only runs at k > 1.
                    double coastNeed = MechJebModuleLandingAutopilot.TRIP_CRUISE_ABORT_AGL_M;
                    if (caCrash || caDv > 0)
                    {
                        AbortTripCruise("CA dV=" + caDv.ToString("F0") + " crash=" + caCrash);
                        desiredThrustVector = VesselState.forward;
                        return;
                    }
                    if (VesselState.altitudeTrue < coastNeed)
                    {
                        AbortTripCruise("AGL " + VesselState.altitudeTrue.ToString("F0") + " < " + coastNeed.ToString("F0"));
                        desiredThrustVector = VesselState.forward;
                        return;
                    }

                    UpdateTripHoopState();

                    Vector3d tgtBody = Core.Target.GetPositionTargetPosition(); // body-relative
                    Vector3d toPad = Vector3d.Exclude(up, tgtBody - posBody);
                    Vector3d dir = (toPad.sqrMagnitude > 1.0) ? toPad.normalized
                        : Vector3d.Exclude(up, VesselState.surfaceVelocity).normalized;
                    Vector3d vHs = Vector3d.Exclude(up, VesselState.surfaceVelocity);
                    double vH = vHs.magnitude;
                    double dist = Core.Landing.getHDistanceToTarget();
                    double dIgn = TripIgnitionDistance(vH);

                    double aCap = Math.Min(Math.Max(VesselState.limitedMaxThrustAccel, 1e-3), 8.0 * g);
                    Vector3d aH = 0.1 * (_tripVTarget * dir - vHs);
                    if (aH.magnitude > 0.7 * aCap)
                        aH *= 0.7 * aCap / aH.magnitude;

                    double vzCmd;
                    if (step == Step.STATE_TRIP_DESCEND && !double.IsNaN(_tripTargetAsl) && _tripTargetAsl > 0)
                    {
                        double tGo = Math.Max((dist - 0.97 * dIgn) / Math.Max(vH, 1.0), 5.0);
                        vzCmd = (_tripTargetAsl - asl) / tGo;
                        if (vzCmd < -TRIP_DESCEND_VZ_MAX) vzCmd = -TRIP_DESCEND_VZ_MAX;
                        if (vzCmd > TRIP_DESCEND_VZ_MAX) vzCmd = TRIP_DESCEND_VZ_MAX;
                    }
                    else
                    {
                        // STATE_TRIP_HOOP and default hold: keep _tripHoldAsl
                        vzCmd = Math.Max(-5.0, Math.Min(5.0, (_tripHoldAsl - asl) / 30.0));
                    }
                    if (dist <= dIgn + TRIP_PREDESCENT_M)
                        vzCmd = Math.Min(vzCmd, (dist <= dIgn) ? -5.0 : -3.0);
                    double vOrbH = Vector3d.Exclude(up, VesselState.orbitalVelocity).magnitude;
                    double gEff = VesselState.localg - vOrbH * vOrbH / Math.Max(r, 1.0);
                    double aV = gEff + 0.2 * (vzCmd - vz);

                    Vector3d a = aH + aV * up;
                    if (a.magnitude > aCap)
                        a *= aCap / a.magnitude;
                    desiredThrustVector = (a.sqrMagnitude > 1e-12) ? a.normalized : up;

                    double aMaxCap = Math.Max(1e-3, VesselState.limitedMaxThrustAccel);
                    double thrCap = Math.Max(0.03, Math.Min(1.0, 8.0 * g / aMaxCap));
                    double thr = a.magnitude / Math.Max(VesselState.maxThrustAccel, 1e-6);
                    if (thr > thrCap) thr = thrCap;
                    lastHorizontalThrust = (float)thr;
                    forceHoriz = false;
                    Core.Thrust.Tmode = MechJebModuleThrustController.TMode.OFF;
                    Core.Thrust.TransKillH = false;
                    Core.Thrust.RequestActiveThrottle((float)thr);

                    string tripMode = (step == Step.STATE_TRIP_DESCEND) ? "descend"
                        : (step == Step.STATE_TRIP_HOOP) ? "hoop" : "cruise";
                    Status = "Trip " + tripMode + " k=" + Core.Landing.TripSpeedK.ToString("F2")
                        + " vH " + vH.ToString("F0") + "/" + _tripVTarget.ToString("F0")
                        + " hold " + _tripHoldAsl.ToString("F0");
                    LogTrip("TSO " + tripMode + " vH=" + vH.ToString("F1") + " vT=" + _tripVTarget.ToString("F1")
                        + " asl=" + asl.ToString("F0") + " hold=" + _tripHoldAsl.ToString("F0") + " tgt=" + (double.IsNaN(_tripTargetAsl) ? "n/a" : _tripTargetAsl.ToString("F0"))
                        + " vz=" + vz.ToString("F2") + " vzCmd=" + vzCmd.ToString("F1")
                        + " gEff=" + gEff.ToString("F3") + " aH=" + aH.magnitude.ToString("F3")
                        + " aV=" + aV.ToString("F3") + " thr=" + thr.ToString("F3")
                        + " dist=" + dist.ToString("F0") + " dIgn=" + dIgn.ToString("F0")
                        + " AGL=" + VesselState.altitudeTrue.ToString("F0"));
                }
                catch (Exception e)
                {
                    AbortTripCruise("exception " + e.Message);
                    desiredThrustVector = VesselState.forward;
                }
            }

            /// <summary>
            /// Step 4 handoff (k > 1 only): replaces the hoop/Ap-past test, which assumes an unpowered
            /// coast (a faster-than-circular level cruise never descends into the hoop). Hand LB the
            /// ignition point on LB's own formula: dist <= 0.97 dIgn(vH), descending (vz <= -1, LB needs
            /// Vy >= 1 to light on d_horiz), local CA clear. Pad behind -> "pad-behind" (MTT recovery).
            /// </summary>
            /// <summary>
            /// Ricardo t301u/t304u (k > 1 only, STATE_TRIP_DESCEND): widened hoop on the CURRENT powered
            /// state. Hand off to LB (engines off, LB coasts / warps) once dist <= 3*dIgn,
            /// floor <= asl <= ceiling, sinking >= 1 m/s, CA quiet, the engine-off coast stays above the
            /// two-tier LB AGL floor until the LB ignition point (>= 2 km beyond 1.85*dIgn, >= 1.5 km
            /// inside it) and climbs no faster than 20 m/s there (Ricardo Nov 14; LB's narrow climb
            /// guard lets a fast horizontal coast distance-ignite). Evaluated 1/s; one forced log line.
            /// floor = max(pad + gate, vicinity terrain + 2 km): the pad term sets it on flat/non-crater
            /// pads (vicinity ~ pad), the terrain term inside craters; both and the winner are logged.
            /// </summary>
            private bool TryTripWideHoop(double dist, double vH, double dIgn, out string why)
            {
                why = "";
                if (step != Step.STATE_TRIP_DESCEND)
                    return false;
                double lim = TRIP_HOOP_LIMIT_DIGN * dIgn;
                if (dist > lim)
                    return false;
                double now = Planetarium.GetUniversalTime();
                if (now - _lastTripWideEvalUt < 1.0)
                    return false;
                _lastTripWideEvalUt = now;

                double asl = VesselState.altitudeASL;
                double vz = VesselState.speedVertical;
                double vicAsl = Core.Landing.TripPadVicinityTerrainAsl(dIgn, out double padAsl);
                double gate = Core.Landing.ComputeOptimalDeorbitAltitude();
                if (double.IsNaN(vicAsl) || double.IsNaN(padAsl) || double.IsNaN(gate) || double.IsInfinity(gate))
                {
                    LogTrip("TSO wide-hoop wait no-terrain dist=" + dist.ToString("F0"), true);
                    return false;
                }
                double padTerm = padAsl + gate;
                double vicTerm = vicAsl + TRIP_DESCEND_TERRAIN_MARGIN_M;
                double floorAsl = Math.Max(padTerm, vicTerm);
                string floorBy = padTerm >= vicTerm ? "pad" : "terrain";
                double ceilAsl = floorAsl + TRIP_HOOP_CEIL_ABOVE_M;
                bool inWin = asl >= floorAsl && asl <= ceilAsl;
                bool sinking = vz <= -1.0;
                (double caDv, bool caCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, SUBORBIT_PATH_MIN_ALTITUDE_M);
                bool caQuiet = !caCrash && caDv <= 0;
                double minAglFar = double.NaN, minAglNear = double.NaN, vyIgn = double.NaN, tIgn = double.NaN;
                bool coastOk = inWin && sinking && caQuiet
                    && Core.Landing.TripCoastToIgnition(dIgn, TRIP_HOOP_COAST_MAX_S, out minAglFar, out minAglNear, out vyIgn, out tIgn);
                bool aglOk = coastOk
                    && minAglFar >= MechJebModuleLandingAutopilot.LANDING_BURN_AGL_FAR_M
                    && minAglNear >= MechJebModuleLandingAutopilot.LANDING_BURN_IGN_PROX_AGL_M;
                bool vyOk = coastOk && vyIgn >= TRIP_HOOP_VY_IGN_MIN;
                bool pass = inWin && sinking && caQuiet && coastOk && aglOk && vyOk;
                why = "dist=" + dist.ToString("F0") + " lim=" + lim.ToString("F0") + " dIgn=" + dIgn.ToString("F0")
                    + " asl=" + asl.ToString("F0") + " floor=" + floorAsl.ToString("F0") + " ceil=" + ceilAsl.ToString("F0")
                    + " pad=" + padAsl.ToString("F0") + " gate=" + gate.ToString("F0") + " vic=" + vicAsl.ToString("F0")
                    + " padTerm=" + padTerm.ToString("F0") + " vicTerm=" + vicTerm.ToString("F0") + " floorBy=" + floorBy
                    + " floorOverPad=" + (floorAsl - padAsl).ToString("F0") + " floorOverVic=" + (floorAsl - vicAsl).ToString("F0")
                    + " hOverPad=" + (asl - padAsl).ToString("F0") + " hOverVic=" + (asl - vicAsl).ToString("F0")
                    + " vz=" + vz.ToString("F2") + " vH=" + vH.ToString("F1") + " caDv=" + caDv.ToString("F0") + " crash=" + caCrash
                    + " coastMinAglFar=" + minAglFar.ToString("F0") + " coastMinAglNear=" + minAglNear.ToString("F0")
                    + " vyIgn=" + vyIgn.ToString("F2") + " tIgn=" + tIgn.ToString("F0")
                    + " inWin=" + inWin + " sink=" + sinking + " ca=" + caQuiet + " coast=" + coastOk
                    + " agl=" + aglOk + " vy=" + vyOk + " pass=" + pass;
                LogTrip("TSO wide-hoop " + why, true);
                return pass;
            }

            private bool TryTripCruiseHandoff(Vector3d targetPosBody, out string reason)
            {
                reason = "";
                try
                {
                    if (PadOverflightBehind(targetPosBody))
                    {
                        reason = "pad-behind";
                        return false;
                    }
                    double dist = Core.Landing.getHDistanceToTarget();
                    double vH = VesselState.speedSurfaceHorizontal;
                    double dIgn = TripIgnitionDistance(vH);
                    if (dist > 0.97 * dIgn)
                    {
                        if (TryTripWideHoop(dist, vH, dIgn, out string hoopWhy))
                        {
                            reason = "trip-hoop k=" + Core.Landing.TripSpeedK.ToString("F2") + " " + hoopWhy;
                            LogTrip("TSO->LB " + reason, true);
                            return true;
                        }
                        reason = "trip-cruise dist=" + dist.ToString("F0") + " dIgn=" + dIgn.ToString("F0");
                        return false;
                    }
                    if (VesselState.speedVertical > -1.0)
                    {
                        reason = "trip-predescent vz=" + VesselState.speedVertical.ToString("F2");
                        LogTrip("TSO->LB wait " + reason + " dist=" + dist.ToString("F0") + " dIgn=" + dIgn.ToString("F0"));
                        return false;
                    }
                    (double caDv, bool caCrash) = Core.Landing.PredictAvoidCollisionDeltaV(15, SUBORBIT_PATH_MIN_ALTITUDE_M);
                    if (caCrash || caDv > 0)
                    {
                        reason = "trip-local-terrain dV=" + caDv.ToString("F0");
                        return false;
                    }
                    reason = "trip-cruise k=" + Core.Landing.TripSpeedK.ToString("F2")
                        + " vH=" + vH.ToString("F1") + " dist=" + dist.ToString("F0")
                        + " dIgn=" + dIgn.ToString("F0") + " vz=" + VesselState.speedVertical.ToString("F2")
                        + " AGL=" + VesselState.altitudeTrue.ToString("F0");
                    LogTrip("TSO->LB " + reason, true);
                    return true;
                }
                catch (Exception e)
                {
                    reason = "trip-handoff-failed " + e.Message;
                    return false;
                }
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
                    // Gravity-aware lateral thr cap: 1 s at a_max is ~11% of Vesta v_circ.
                    // thr_cap = k*g/a_max (Vesta ~0.07 with k=8; Kerbin clamps to 1).
                    // Cap the vessel Request, not only lastHorizontalThrust: debug11/g
                    // (craft debug11=0.01 on Vesta => throttleDiv~0.04) undoes a pre-div
                    // cap (204916: lastH=0.07 but CSV thr=1 wiped H 98->0 in ~3s).
                    double aMaxCap = Math.Max(1e-3, VesselState.limitedMaxThrustAccel);
                    double gCap = Math.Max(0.05, Core.Landing.g);
                    double thrCap = Math.Max(0.03, Math.Min(1.0, 8.0 * gCap / aMaxCap));
                    if (thrust > thrCap) thrust = thrCap;
                    lastHorizontalThrust = (float)thrust;
                    double throttleDiv = Core.Landing.debug11 / Core.Landing.g;
                    if (double.IsNaN(throttleDiv) || throttleDiv < 1e-6) throttleDiv = 1e-6;
                    float reqThr = lastHorizontalThrust / (float)throttleDiv;
                    if (reqThr > thrCap) reqThr = (float)thrCap;
                    Core.Thrust.RequestActiveThrottle(reqThr);
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
                    // All forceHoriz=false paths (PROGRADE + RETRO belt): same g/a_max thrCap
                    // on the actual Request. 211436: uncapped PROGRADE thr=1 for ~3s raised
                    // H 26->103 and circularized (alt~Ap~43km); LB then had t_go~300s /
                    // a_cmd~0.2 and timed out at 1.4km still 32km AGL. Cap matches LATERAL.
                    {
                        double aMaxCapR = Math.Max(1e-3, VesselState.limitedMaxThrustAccel);
                        double gCapR = Math.Max(0.05, Core.Landing.g);
                        double thrCapR = Math.Max(0.03, Math.Min(1.0, 8.0 * gCapR / aMaxCapR));
                        if (verticalThrust > thrCapR) verticalThrust = thrCapR;
                        lastHorizontalThrust = (float)verticalThrust;
                        double throttleDivR = Core.Landing.debug11 / Core.Landing.g;
                        if (double.IsNaN(throttleDivR) || throttleDivR < 1e-6) throttleDivR = 1e-6;
                        float reqThrR = lastHorizontalThrust / (float)throttleDivR;
                        if (reqThrR > thrCapR) reqThrR = (float)thrCapR;
                        Core.Thrust.RequestActiveThrottle(reqThrR);
                    }
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
