using System;
using MuMech.Landing;
using UnityEngine;
using static alglib;
using static DishController;

namespace MuMech.LandingAutopilot
{
    /// <summary>
    /// MechJeb autopilot step for powered descent using vector-based ZEM/ZEV guidance.
    /// Controls thrust direction and throttle during final descent.
    /// </summary>
    public class LandingBurn : AutopilotStep
    {
        // Used to print every 50 times
        private long debugCounter = 0;

        // Flag to indicate if burn has started (latches on once triggered)
        private bool shouldBurnStarted = false;
        private double densityAtBurnStart = 0;

        // Flag: is time warp currently active?
        private bool warpOn = false;
        private bool warpOnP = false;

        // Flag: is time warp allowed right now?
        private bool checkWarp = true;

        // Minimum altitude (meters) below which warp is disabled for safety
        private const double ALT_MIN_WARP_CONSTANT = 5000.0;

        // Downrange distance (meters) below which vertical ZEV gain is reduced for smoother final descent (used if ZevV<0 for conditional vertical braking boost)
        private const float DOWNRANGE_THRESHOLD_CONSTANT = 1500.0f;

        private const double LB_AIM_ABOVE_PAD_M = 100;
        private const float ZEV_FULL_RANGE_M = 3000.0f;

        private double targetOffset;      // Removed once reached.
        //private double zevCounter;        // Used to enable zevV via a smoothing function.
        private float  minDownRange;      // Minimum downrange distance to target for full ZevV application (used if ZevV<0 for conditional vertical braking boost)
        private double lateralGain = 0.4; // Lateral gain correction
        private double prev_t_go = 0;     // Previous t_go used to force timep step changes for better guidance convergence (not strictly necessary)
        private double lastDesiredAcceleration = 0;

        // Autotest / KSP.log 1 Hz guidance snapshot (no behavior change).
        private double _lastGuidanceLogUt = double.NegativeInfinity;
        public double AutotestTgo = double.NaN;
        public double AutotestPadDh = double.NaN;
        public bool AutotestBurnStarted = false;
        public string AutotestBurnAxis = "";
        public double AutotestZemMag = double.NaN;
        public double AutotestZevMag = double.NaN;
        public double AutotestDesiredAccelMag = double.NaN;
        public double AutotestThrottle = double.NaN;
        public double AutotestAPlan = double.NaN;
        public double AutotestAClip = double.NaN;

        // Shared braking budget for ignition, t_go, and throttle clip.
        private double _aPlan = 0;
        private double _aClip = 0;
        private double _aMaxBudget = 0;

        /// <summary>
        /// Constructor - receives MechJeb core reference.
        /// </summary>
        /// <param name="Core">MechJeb core instance</param>
        public LandingBurn(MechJebCore Core) : base(Core)
        {
            targetOffset = Math.Max(1, Core.Landing.TargetOffset); // Must be non-zero at start to prevent premature burn cutoff, but can be reduced to 0 for final touchdown precision
            //zevCounter = 0;
            minDownRange = 1500.0f;
        }

        /// <summary>
        /// Main drive loop - called every frame.
        /// Runs guidance if vessel is not landed/splashed.
        /// </summary>
        /// <param name="s">Current flight control state</param>
        /// <returns>This step (remains active)</returns>
        public override AutopilotStep Drive(FlightCtrlState s)
        {
            if (!Vessel.LandedOrSplashed)
            {
                if (true == UpdateGuidanceAndControl(s))
                {
                    return new MoveToTarget(Core);
                }
                return this;
            }
            else
            {
                Core.Landing.StopLanding();
                return null;
            }
        }

        /// <summary>
        /// Fixed-update hook - handles time-warp decisions.
        /// </summary>
        /// <returns>This step (remains active)</returns>
        public override AutopilotStep OnFixedUpdate()
        {
            if (warpOn == false)
                return this;

            // Altitude above terrain (what you usually want for guidance "y")
            double alt = VesselState.altitudeTrue - targetOffset;

            // True radius from body center to bottom of vessel
            double r_bottom = Vessel.mainBody.Radius
                            + Vessel.mainBody.TerrainAltitude(Vessel.latitude, Vessel.longitude)
                            + alt;

            // Current position vector from body center
            Vector3d pos = (VesselState.CoM - Vessel.mainBody.position).normalized * (r_bottom);

            // Target position vector on surface
            Vector3d targetPos = Core.Target.GetPositionTargetPosition();

            // Vector from current position to target
            Vector3d posError = targetPos - pos;

            // Surface-relative velocity vector
            Vector3d vel = VesselState.surfaceVelocity;

            // Check if burn should be active
            ShouldStartBurn(ref posError, ref vel);

            return this;
        }


        /// <summary>
        /// One braking budget for ignition distance, t_go, and throttle clip.
        /// aPlan ~ min(8g, 0.7 aMax) but at least g+0.05 (Vesta ~2 m/s2; low-TWR keeps margin).
        /// aClip = min(aMax, 3 aPlan) soft ceiling; near-pad suicide may still use full aMax.
        /// </summary>
        private void ComputeBrakeBudget()
        {
            double aMax = Math.Max(VesselState.limitedMaxThrustAccel, 0.1);
            double gMag = Math.Max(VesselState.localg, 0.05);
            // Prefer 8g when TWR is high; on low-TWR bodies 0.7 aMax leaves headroom.
            double aPlan = Math.Min(8.0 * gMag, 0.7 * aMax);
            aPlan = Math.Max(aPlan, gMag + 0.05);
            aPlan = Math.Min(aPlan, aMax);
            double aClip = Math.Min(aMax, 3.0 * aPlan);
            _aMaxBudget = aMax;
            _aPlan = aPlan;
            _aClip = aClip;
        }


        private void CancelWarpOnBurnLatch()
        {
            // Any ignition latch must drop time warp (regular or physics) immediately.
            if (warpOn || warpOnP) Core.Landing.NoteWarp("LB exit:burn-latch");
            try { Core.Warp.MinimumWarp(true); } catch { }
            checkWarp = false;
            warpOn = false;
            warpOnP = false;
        }
        private bool ShouldStartBurn(ref Vector3d posError, ref Vector3d vel)
        {
            if (shouldBurnStarted)
            {
                // Re-assert warp cancel if latch came from any path / FixedUpdate race.
                if (checkWarp || warpOn || warpOnP)
                    CancelWarpOnBurnLatch();
                return true;
            }

            if (vel.magnitude < 1.0) return false;

            ComputeBrakeBudget();

            bool     rc;
            Vector3d up = Vessel.up;
            Vector3d retro = -vel.normalized;
            double sin_theta = Math.Abs(Vector3d.Dot(retro, -up));   // sin of angle from horizontal
            double cos_theta = Math.Sqrt(1.0 - sin_theta * sin_theta);
            double Vy = Vector3d.Dot(vel, -up);             // downward component
            double g = VesselState.localg;
            double a = _aPlan;                             // planned brake accel (not full aMax)

            double current_h = -Vector3d.Dot(posError, up);
            Vector3d posError_horizontal = posError - Vector3d.Dot(posError, up) * up;
            double current_d = posError_horizontal.magnitude;
            double current_agl = VesselState.altitudeTrue;

            // Terrain / AGL suicide: always evaluated so a flat coast cannot fly into the ground
            // while sin_theta keeps h_vert at 0. Uses aPlan vertical stop + two-tier AGL trigger
            // (Ricardo Nov 14, t310u/t311u/t314u): LANDING_BURN_IGN_PROX_AGL_M (1.5 km) inside the
            // no-warp zone (current_d <= 1.85 * d_horiz, just before ignition), LANDING_BURN_AGL_FAR_M
            // (2 km) beyond it. h_vert / d_horiz keep the LANDING_BURN_AGL_FLOOR_M (3 km) minimum.
            const double distFloor = MechJebModuleLandingAutopilot.LANDING_BURN_AGL_FLOOR_M;
            double vDown = Math.Max(Vy, 0.0);
            double ayNet = Math.Max(a - g, 0.1);
            double h_suicide = (1.05 + Core.Landing.BurnMarginPerc / 100.0) * (vDown * vDown) / (2.0 * ayNet);

            double Vx = vel.magnitude * cos_theta;
            // Effective accelerations along retrograde direction at aPlan
            double ay = a * sin_theta - g;                  // vertical (downward positive)
            double ax = a * cos_theta;                      // horizontal
            double dStopRaw = (1.00 + Core.Landing.BurnMarginPerc / 100.0) * (Vx * Vx) / (2.0 * Math.Max(ax, 0.1));
            double dIgnZone = 1.85 * Math.Max(distFloor, dStopRaw);
            double aglTrig = current_d <= dIgnZone
                ? MechJebModuleLandingAutopilot.LANDING_BURN_IGN_PROX_AGL_M
                : MechJebModuleLandingAutopilot.LANDING_BURN_AGL_FAR_M;
            if (current_agl <= Math.Max(aglTrig, h_suicide))
            {
                shouldBurnStarted = true;
                CancelWarpOnBurnLatch();
                densityAtBurnStart = FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(current_h / 5.0, MainBody), FlightGlobals.getExternalTemperature(current_h / 5.0, MainBody));
                Debug.Log($"[ShouldStartBurn2:agl] agl={current_agl:F0}, aglTrig={aglTrig:F0}, h_suicide={h_suicide:F0}, Vy={Vy:F3}, current_d={current_d:F0}, zone={dIgnZone:F0}, aPlan={a:F2}");
                return true;
            }

            // Ricardo Nov 14 (t310u): the old "Vy < 1 -> never ignite" gate is replaced by a narrow
            // guard. While not sinking, h_vert is ignored (a steep climb must not light via h_vert and
            // point retrograde at the ground); distance ignition stays allowed only for a genuinely fast
            // horizontal approach (unclamped stop distance > floor), e.g. a climbing k>1 trip coast.
            // Slow / short-hop climbs keep the old behaviour (no ignition). Warp logic below still runs.
            bool vertOn = Vy >= 1.0;
            if (!vertOn && dStopRaw <= distFloor)
            {
                Debug.Log($"[ShouldStartBurn2] climb-guard sin_theta={sin_theta:F3}, cos_theta={cos_theta:F3}, Vy={Vy:F3}, dStop={dStopRaw:F0}");
                rc = false;
            }
            else
            {
                // Baseline 3 km floor (no TWR-scaled expansion — that lit at ~8 km / 33 km AGL on Vesta).
                double h_vert  = Math.Max(distFloor, (1.05 + Core.Landing.BurnMarginPerc / 100.0) * (Vy * Vy) / (2.0 * Math.Max(ay, 0.1)));
                double d_horiz = Math.Max(distFloor, dStopRaw);

                if (sin_theta < 0.25 || !vertOn)
                {
                    h_vert = 0; // Nearly horizontal (or not sinking): horizontal range owns ignition; AGL suicide above covers terrain
                }

                rc = (vertOn && current_h <= (h_vert)) || (current_d <= (d_horiz));
                if (rc)
                {
                    shouldBurnStarted = true;
                    CancelWarpOnBurnLatch();
                    densityAtBurnStart = FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(current_h / 5.0, MainBody), FlightGlobals.getExternalTemperature(current_h / 5.0, MainBody));
                }
                else
                {
                    rc = (vertOn && current_h <= (1.85 * h_vert)) || (current_d <= (1.85 * d_horiz));

                    // Perform warp if allowed and requested
                    if (checkWarp && (rc == false) && Core.Node.Autowarp && !Vessel.LandedOrSplashed)
                    {
                        h_vert *= 1.85;
                        d_horiz *= 1.85;
                        Debug.Log($"[ShouldStartBurn2:w] current_h={current_h:F0}, h_vert={h_vert:F0}, current_d={current_d:F0}, d_horiz={d_horiz:F0}, aPlan={a:F2}");
                    }
                    else
                    {
                        Debug.Log($"[ShouldStartBurn2] current_h={current_h:F0}, h_vert={h_vert:F0}, current_d={current_d:F0}, d_horiz={d_horiz:F0}, aPlan={a:F2}");
                    }
                }
            }

            // Perform warp if allowed and requested
            if (checkWarp && (rc == false) && Core.Node.Autowarp && !Vessel.LandedOrSplashed)
            {
                if (VesselState.altitudeASL < Vessel.mainBody.atmosphereDepth)
                {
                    if (warpOnP == false)
                    {
                        //too low to use any regular warp rates. Use physics warp at a max of x2:
                        Core.Landing.NoteWarp("LB coast(physics)");
                        Core.Warp.WarpPhysicsAtRate(2);
                        warpOnP = true;
                    }
                }
                else
                {
                    // Conservative velocity estimate for 10-second impact buffer
                    double velocityGuess = Math.Max(Math.Abs(VesselState.speedVertical), VesselState.localg * 5);
                    // Safe warp rate: limit based on altitude and orbital period
                    float warpRate = (float)Math.Min(VesselState.altitudeTrue / (5 * velocityGuess), (Orbit.period / 6));

                    if (!warpOn) Core.Landing.NoteWarp("LB coast");
                    Core.Warp.WarpRegularAtRate(warpRate);
                    warpOnP = false;
                    warpOn = true;
                }
            }
            else if (warpOn)
            {
                Core.Landing.NoteWarp(rc ? "LB exit:no-warp-zone(1.85*dIgn)"
                    : !Core.Node.Autowarp ? "LB exit:autowarp-off"
                    : Vessel.LandedOrSplashed ? "LB exit:landed" : "LB exit:checkWarp-off");
                Core.Warp.MinimumWarp(true);
                checkWarp = warpOn = warpOnP = false;
            }

            return rc;
        }

        /// <summary>
        /// Computes vector ZEM/ZEV guidance commands and sets throttle/attitude.
        /// Uses full vector math for position/velocity errors.
        /// Applies lateral damping to eliminate off-target velocity.
        /// </summary>
        private bool UpdateGuidanceAndControl(FlightCtrlState s)
        {
            bool recover = false;

            if (warpOn == true)
                return recover;

            // Altitude above terrain (what you usually want for guidance "y")
            double alt = VesselState.altitudeTrue - targetOffset;

            // True radius from body center to bottom of vessel
            double r_bottom = Vessel.mainBody.Radius
                            + Vessel.mainBody.TerrainAltitude(Vessel.latitude, Vessel.longitude)
                            + alt;

            // Current position vector from body center
            Vector3d pos = (VesselState.CoM - Vessel.mainBody.position).normalized * (r_bottom);

            // Target position vector on surface
            Vector3d targetPos = Core.Target.GetPositionTargetPosition();

            // Vector from current position to target
            Vector3d posError = targetPos - pos;

            // Slant range (distance to target)
            double slantRange = posError.magnitude;

            if (slantRange <= 0.1)
            {
                Core.Thrust.RequestActiveThrottle(0);
                return recover;
            }

            // Surface-relative velocity vector
            Vector3d vel = VesselState.surfaceVelocity;

            // Get range vectors for target direction
            Vector3d downrangeVec, crossrangeVec, horizontalVec;
            Core.Landing.GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, VesselState.time,
                out downrangeVec, out crossrangeVec, out horizontalVec);

            // Check if burn should be active
            ShouldStartBurn(ref posError, ref vel); 

            // Local gravity magnitude
            Vector3d g = VesselState.gravityForce;

            // Net upward braking acceleration available
            double a_brake_net = VesselState.limitedMaxThrustAccel - g.magnitude;

            // Vertical velocity component (positive up)
            double vy = Vector3d.Dot(vel, Vessel.up);

            // Downward velocity magnitude
            double v_down = Math.Max(0, -vy);

            // t_go: stopping time at shared aPlan (same budget as ShouldStartBurn).
            ComputeBrakeBudget();
            double a_max = _aMaxBudget;
            double gMag = Math.Max(g.magnitude, 0.05);
            double a_used = _aPlan;
            double a_clip = _aClip;
            double a_brake_f = Math.Max(a_used - gMag, 0.1);
            double rangeForTgo = Math.Max(alt, 1.0);
            double disc = v_down * v_down + 2.0 * a_brake_f * rangeForTgo;
            double t_vert = (v_down + Math.Sqrt(Math.Max(disc, 0.0))) / a_brake_f;
            double v_horiz = Vector3d.Exclude(Vessel.up, vel).magnitude;
            double t_horiz = v_horiz / a_used; // downrange stop time
            float downrange = (float)downrangeVec.magnitude;
            double t_go = Math.Max(1.0, Math.Max(t_vert, t_horiz));
            // Mild low-pass only (was Min(prev-dt/5, t_go) which held multi-minute t_go).
            if (prev_t_go > 0)
            {
                t_go = Math.Max(1.0, 0.75 * t_go + 0.25 * prev_t_go);
            }
            prev_t_go = t_go;

            // Correct ZEM and ZEV signs (desired - predicted)
            Vector3d predicted_pos = vel * t_go + 0.5 * VesselState.gravityForce * t_go * t_go;
            // Aim at a point LB_AIM_ABOVE_PAD_M above the pad (radial). Velocity target stays 0.
            Vector3d zem = posError + targetPos.normalized * LB_AIM_ABOVE_PAD_M - predicted_pos;
            Vector3d zem_x = Vector3d.Exclude(Vessel.up, zem);
            Vector3d zev_x = -VesselState.horizontalSurface;
            Vector3d zem_y = Vector3d.Exclude(zem_x, zem);
            Vector3d zev_y = -(vy*Vessel.up + VesselState.gravityForce * t_go); // = 0 - predicted_velocity

            // Vertical ZEV: soft TWR-scaled fade (no hard zero outside 1500 m). Hard gate left
            // ZevV=0 for the whole Vesta approach so the vertical channel never shaped descent.
            double ZevV = Core.Landing.ZevV;
            float zevFade = 1.0f;

            if (ZevV < 0)
            {
                double stopRef = (v_horiz * v_horiz) / Math.Max(2.0 * a_used, 0.1);
                float zevGate = (float)Math.Max(DOWNRANGE_THRESHOLD_CONSTANT,
                    Math.Min(80000.0, 3.0 * stopRef + 1500.0));
                minDownRange = Math.Min(minDownRange, downrange);
                float rangeForFade = Math.Min(minDownRange, downrange);
                zevFade = Mathf.Clamp01((zevGate - rangeForFade) / Math.Max(zevGate - ZEV_FULL_RANGE_M, 1.0f));
                ZevV *= zevFade;
            }

            // Commanded acceleration (ZEM/ZEV formula)
            Vector3d a_cmd = (Core.Landing.ZemH / (t_go * t_go)) * zem_x + (Core.Landing.ZevH / t_go) * zev_x +
                             (Core.Landing.ZemV / (t_go * t_go)) * zem_y + (ZevV / t_go) * zev_y;

            // below this altitude, switch to recovery mode if not on target (prevents hard landings from guidance errors or bad tuning)
            // Or, if the downrange to altitude ratio is close to minRatio, switch to recovery mode - The final landing is based on tracking vertical and horizontal velocity as we zero in to target.
            if (lastDesiredAcceleration < (0.95*MoveToTarget.LIMITED_MAX_THRUST_G_RATIO*VesselState.localg))
            {
                if (((downrange / alt) <= (1.1 * Core.Landing.minRatio)) || (alt < Core.Landing.LandWithMoverAlt) || (downrange < Core.Landing.LandWithMoverOffset))
                {
                    recover = true;
                }
            }

            // Apply terminal velocity bias (encourages stronger vertical braking near ground to hit target speed)
            // Final-phase boost (stronger control near ground)
            if ( (alt < 20.0) || (targetOffset==0) )
            {
                targetOffset = 0; // Remove target offset once reached to provide more safety margin for final touchdown
                lateralGain = 0.1;// 0.01; // Reduce lateral gain near ground to prevent overshoot from aggressive vertical braking

                a_cmd = a_cmd - Vector3d.Project(a_cmd, Vessel.up); // Remove vertical component to prevent interference with vertical braking

                // Terminal vertical velocity bias: add extra vertical braking if descending faster than target speed near ground
                a_cmd += (-Math.Abs(Core.Landing.TouchdownSpeed) - vy) * Vessel.up * (6.0 / t_go);
            }

            // Near pad / low AGL with large downward speed: bias a_cmd to suicide-burn vertical
            // before horizontal downrange/anti-downrange can bounce past the pad (baseline
            // crossed pad then thr~0.05 downrange while vSpd stayed -60).
            if (alt < 5000.0 && vy < -8.0)
            {
                double needAlt = (vy * vy) / (2.0 * Math.Max(a_brake_net, 0.1));
                if (alt < needAlt * 2.0)
                {
                    double blend = Mathf.Clamp01((float)((needAlt * 2.0 - alt) / Math.Max(needAlt, 1.0)));
                    Vector3d vertBrake = Vessel.up * VesselState.limitedMaxThrustAccel;
                    a_cmd = a_cmd * (1.0 - blend) + vertBrake * blend;
                }
            }

            // Gain scale: do not floor at 0.4 for long t_go (that kept thr~0.02 on Vesta).
            // With stopping-time t_go this stays near 1; still soft-caps only if t_go is large.
            double gain_scale = Math.Min(1.0, Math.Max(0.7, 45.0 / Math.Max(t_go, 1.0)));
            a_cmd *= gain_scale;

            // Desired acceleration vector
            Vector3d desired_accel = a_cmd;

            // Lateral damping: only horizontal plane perpendicular to target
            if ( lateralGain> 0 )
            {
                Vector3d parallelHorizontal = horizontalVec.normalized;
                Vector3d velParallelH = Vector3d.Project(VesselState.horizontalSurface, parallelHorizontal);
                Vector3d velLateralH = VesselState.horizontalSurface - velParallelH;
                desired_accel -= lateralGain * velLateralH;
            }

            // Clamp magnitude to aClip (3*aPlan); near-pad vertBrake may have used full aMax above.
            double desired_mag = desired_accel.magnitude;
            if (desired_mag > a_clip + 0.01)
            {
                desired_accel = desired_accel.normalized * a_clip;
                desired_mag = a_clip;
            }

            if ( shouldBurnStarted == true )
            {
                // Throttle fraction of full aMax so aPlan maps to ~aPlan/aMax
                float throttle = (float)(desired_mag / Math.Max(a_max, 0.1));

                if (Core.Attitude.attitudeAngleFromTarget() < 30)
                {
                    lastDesiredAcceleration = desired_mag;
                    Core.Thrust.RequestActiveThrottle(throttle);
                }
                else
                {
                    Core.Thrust.RequestActiveThrottle(0);
                }

                if (--debugCounter <= 0)
                {
                    // Diagnostic log for debugging burn trigger
                    Debug.Log($"[burn] t_go={t_go:F1} alt={alt:F1} vel={Vessel.horizontalSrfSpeed:F1},{vy:F1} t={throttle:F2} {densityAtBurnStart:F3} ");
                    debugCounter = 2;
                }

                MaybeLogGuidanceTick(t_go, zem, zev_x + zev_y, desired_accel, downrangeVec, alt, vy, throttle, zevFade);
            }
            else
            {
                MaybeLogGuidanceTick(t_go, zem, zev_x + zev_y, desired_accel, downrangeVec, alt, vy, 0f, zevFade);
            }

            // Set attitude to point thrust along desired_accel
            Quaternion targetRot = Quaternion.LookRotation(desired_accel.normalized, Vessel.up)
                                 * Quaternion.Euler(0, 0, (float)Core.Landing.vesselAngle);
            Core.Attitude.attitudeTo(targetRot, AttitudeReference.INERTIAL, this);

            return recover;
        }

        private void MaybeLogGuidanceTick(double t_go, Vector3d zem, Vector3d zev, Vector3d desiredAccel,
            Vector3d downrangeVec, double alt, double vy, float throttle, float zevFade)
        {
            double now = Planetarium.GetUniversalTime();
            double padDh = double.NaN;
            bool havePad = Core.Landing.TryPadOverflightDeltaHeight(out padDh, out _);
            string burnAxis = DescribeBurnAxis(desiredAccel, downrangeVec);

            AutotestTgo = t_go;
            AutotestPadDh = havePad ? padDh : double.NaN;
            AutotestBurnStarted = shouldBurnStarted;
            AutotestBurnAxis = burnAxis;
            AutotestZemMag = zem.magnitude;
            AutotestZevMag = zev.magnitude;
            AutotestDesiredAccelMag = desiredAccel.magnitude;
            AutotestThrottle = throttle;
            AutotestAPlan = _aPlan;
            AutotestAClip = _aClip;

            if (now - _lastGuidanceLogUt < 1.0)
                return;
            _lastGuidanceLogUt = now;

            double dist = double.NaN;
            try { dist = Core.Landing.getHDistanceToTarget(); } catch { /* ignore */ }

            Debug.Log("[LB-1Hz] burnStarted=" + shouldBurnStarted
                + " t_go=" + t_go.ToString("F1")
                + " padDh=" + (havePad ? padDh.ToString("F0") : "n/a")
                + " dist=" + (double.IsNaN(dist) ? "n/a" : dist.ToString("F0"))
                + " hSpd=" + VesselState.speedSurfaceHorizontal.ToString("F1")
                + " vSpd=" + vy.ToString("F1")
                + " alt=" + alt.ToString("F0")
                + " ApA=" + VesselState.orbitApA.ToString("F0")
                + " burn=" + burnAxis
                + " zem=" + zem.magnitude.ToString("F0")
                + " zev=" + zev.magnitude.ToString("F1")
                + " aCmd=" + desiredAccel.magnitude.ToString("F2")
                + " aPlan=" + _aPlan.ToString("F2")
                + " aClip=" + _aClip.ToString("F2")
                + " thr=" + throttle.ToString("F2")
                + " TgtAlt=" + Core.Landing.TgtAlt.ToString("F0")
                + " aimH=" + LB_AIM_ABOVE_PAD_M.ToString("F0")
                + " zevFade=" + zevFade.ToString("F2"));
        }

        private string DescribeBurnAxis(Vector3d desiredAccel, Vector3d downrange)
        {
            if (desiredAccel.sqrMagnitude < 1e-12) return "none";
            Vector3d a = desiredAccel.normalized;
            double upDot = Vector3d.Dot(a, Vessel.up);
            Vector3d ah = Vector3d.Exclude(Vessel.up, a);
            double drDot = 0;
            if (downrange.sqrMagnitude > 1e-12 && ah.sqrMagnitude > 1e-12)
                drDot = Vector3d.Dot(ah.normalized, downrange.normalized);
            Vector3d vel = VesselState.surfaceVelocity;
            double retroDot = 0;
            if (vel.sqrMagnitude > 1e-6)
                retroDot = Vector3d.Dot(a, -vel.normalized);
            if (retroDot > 0.7) return "retrograde-brake";
            if (drDot < -0.5) return "anti-downrange";
            if (drDot > 0.5) return "downrange";
            if (upDot > 0.7) return "radial-out";
            if (upDot < -0.7) return "radial-in";
            return "lateral-mix";
        }
    }
}
