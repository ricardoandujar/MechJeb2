using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using MuMech.Landing;
using MuMech.LandingAutopilot;

namespace MuMech
{
    /// <summary>
    /// Unattended vacuum far-hop test harness. Active only when
    /// GameData/MechJeb2/autotest.cfg exists. Dev-branch only.
    ///
    /// Load path mirrors stock Resume Game:
    /// MAINMENU (settle) → LoadGame → SPACECENTER (settle) →
    /// re-LoadGame(named save) → StartAndFocusVessel → Flight → engage.
    /// Re-load at SpaceCentre so we never inherit persistent.sfs from a prior hop.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class MechJebAutotestMenuLoader : MonoBehaviour
    {
        private const float MenuSettleSeconds = 5f;

        private void Start()
        {
            if (!MechJebAutotest.CfgExists())
            {
                Debug.Log("[MechJebAutotest] No autotest.cfg at " + MechJebAutotest.CfgPath + " - harness idle.");
                return;
            }

            if (!MechJebAutotestSession.TryClaimMenuLoad())
            {
                Debug.Log("[MechJebAutotest] MainMenu load already claimed this session - skipping.");
                return;
            }

            StartCoroutine(LoadThroughSpaceCenter());
        }

        private IEnumerator LoadThroughSpaceCenter()
        {
            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] MainMenu: settling {0:F1}s before stock load path", MenuSettleSeconds));
            yield return new WaitForSecondsRealtime(MenuSettleSeconds);

            if (!MechJebAutotest.TryReadCfg(out MechJebAutotestConfigData cfg))
            {
                Debug.LogError("[MechJebAutotest] Failed to parse autotest.cfg");
                yield break;
            }

            MechJebAutotestConfig.Apply(cfg);

            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] MainMenu: LoadGame('{0}', '{1}') → SPACECENTER",
                cfg.SaveName, cfg.SaveFolder));

            Game game;
            try
            {
                game = GamePersistence.LoadGame(cfg.SaveName, cfg.SaveFolder, true, false);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                Debug.LogError("[MechJebAutotest] LoadGame exception: " + e.Message);
                yield break;
            }

            if (game == null || game.flightState == null)
            {
                Debug.LogError("[MechJebAutotest] LoadGame returned null/empty flightState for " +
                               cfg.SaveFolder + "/" + cfg.SaveName);
                yield break;
            }

            HighLogic.CurrentGame = game;
            HighLogic.SaveFolder  = cfg.SaveFolder;
            game.startScene       = GameScenes.SPACECENTER;
            MechJebAutotestSession.PendingSpaceCenterToFlight = true;

            // Stock Resume path goes through the space center first; do not jump straight to FLIGHT.
            HighLogic.LoadScene(GameScenes.SPACECENTER);
        }
    }

    [KSPAddon(KSPAddon.Startup.SpaceCentre, false)]
    public class MechJebAutotestSpaceCentreLoader : MonoBehaviour
    {
        private const float SpaceCenterSettleSeconds = 5f;

        private void Start()
        {
            if (!MechJebAutotestSession.PendingSpaceCenterToFlight)
                return;
            if (!MechJebAutotest.CfgExists() && !MechJebAutotestConfig.Ready)
                return;

            StartCoroutine(StartFlightAfterSettle());
        }

        private IEnumerator StartFlightAfterSettle()
        {
            // Wait until the space-center scene has finished its own startup.
            yield return null;
            yield return new WaitForEndOfFrame();

            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] SpaceCentre: settling {0:F1}s before StartAndFocusVessel",
                SpaceCenterSettleSeconds));
            yield return new WaitForSecondsRealtime(SpaceCenterSettleSeconds);

            if (!MechJebAutotestSession.PendingSpaceCenterToFlight)
                yield break;

            // Re-load the named quicksave here. Entering SPACECENTER can leave
            // HighLogic.CurrentGame pointing at persistent.sfs from a prior hop.
            string saveFolder = MechJebAutotestConfig.SaveFolder;
            string saveName   = MechJebAutotestConfig.SaveName;
            if (string.IsNullOrEmpty(saveName))
            {
                Debug.LogError("[MechJebAutotest] SpaceCentre: saveName missing - cannot start flight.");
                MechJebAutotestSession.PendingSpaceCenterToFlight = false;
                yield break;
            }

            Game game;
            try
            {
                game = GamePersistence.LoadGame(saveName, saveFolder, true, false);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                Debug.LogError("[MechJebAutotest] SpaceCentre LoadGame exception: " + e.Message);
                MechJebAutotestSession.PendingSpaceCenterToFlight = false;
                yield break;
            }

            if (game == null || game.flightState == null)
            {
                Debug.LogError("[MechJebAutotest] SpaceCentre: LoadGame returned null/empty flightState.");
                MechJebAutotestSession.PendingSpaceCenterToFlight = false;
                yield break;
            }

            HighLogic.CurrentGame = game;
            HighLogic.SaveFolder  = saveFolder;

            int idx = MechJebAutotest.ResolveVesselIndex(game,
                MechJebAutotestConfig.VesselName,
                MechJebAutotestConfig.VesselIndex);
            if (idx < 0 || idx >= game.flightState.protoVessels.Count)
            {
                Debug.LogError(string.Format(CultureInfo.InvariantCulture,
                    "[MechJebAutotest] SpaceCentre: vessel index {0} out of range (count={1})",
                    idx, game.flightState.protoVessels.Count));
                MechJebAutotestSession.PendingSpaceCenterToFlight = false;
                yield break;
            }

            ProtoVessel pv = game.flightState.protoVessels[idx];
            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] SpaceCentre: focusing idx={0} name='{1}' lat={2:F5} lon={3:F5} alt={4:F1} sit={5} (activeVesselIdx={6})",
                idx,
                pv != null ? pv.vesselName : "?",
                pv != null ? pv.latitude : 0,
                pv != null ? pv.longitude : 0,
                pv != null ? pv.altitude : 0,
                pv != null ? pv.situation.ToString() : "?",
                game.flightState.activeVesselIdx));

            MechJebAutotestSession.PendingSpaceCenterToFlight = false;
            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] SpaceCentre: FlightDriver.StartAndFocusVessel(game, {0})", idx));

            try
            {
                FlightDriver.StartAndFocusVessel(game, idx);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                Debug.LogError("[MechJebAutotest] StartAndFocusVessel exception: " + e.Message);
            }
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class MechJebAutotest : MonoBehaviour
    {
        private const string CfgRelativePath = "GameData/MechJeb2/autotest.cfg";
        private const string LogDirRelative  = "GameData/MechJeb2/autotest";
        private const float  FlightSettleSeconds = 5f;
        private const double MinAirborneSeconds  = 10.0;
        private const double MinTerrainAltForLiftoff = 50.0;
        private const float  TimeoutExtendStepSec = 300f;
        private const float  TimeoutHardCapSec = 3600f;
        private const double TimeoutImpactHorizonSec = 400.0;
        private const double TimeoutClosePadM = 15000.0;

        private enum Phase
        {
            WaitReady,
            Engage,
            Running,
            Done
        }

        private Phase   _phase = Phase.WaitReady;
        private float   _timeoutSec = 1800f;
        private float   _timeoutBaseSec = 1800f;
        private int     _timeoutExtensions;
        private float   _startUt = -1f;
        private float   _nextLogUt;
        private double  _maxApa = double.NegativeInfinity;
        private string  _csvPath;
        private StreamWriter _csv;
        private string  _prevStepName;
        private bool    _optionalTarget;
        private double  _optLat, _optLon;
        private string  _optBody = "";
        private string  _outcome;
        private bool    _subscribed;
        private bool    _liftoffQualified;
        private double  _airborneStartUt = -1;
        private float   _readyRealtime = -1f;

        public static string CfgPath =>
            Path.Combine(KSPUtil.ApplicationRootPath, CfgRelativePath.Replace('/', Path.DirectorySeparatorChar));

        public static bool CfgExists() => File.Exists(CfgPath);

        internal static bool TryReadCfg(out MechJebAutotestConfigData cfg)
        {
            cfg = new MechJebAutotestConfigData();

            ConfigNode root = ConfigNode.Load(CfgPath);
            if (root == null)
                return false;

            ConfigNode node = root.GetNode("MechJebAutotest") ?? root;
            node.TryGetValue("saveFolder", ref cfg.SaveFolder);
            node.TryGetValue("saveName", ref cfg.SaveName);
            node.TryGetValue("timeoutSeconds", ref cfg.TimeoutSec);
            node.TryGetValue("vesselName", ref cfg.VesselName);

            int vesselIndex = -1;
            if (node.TryGetValue("vesselIndex", ref vesselIndex))
                cfg.VesselIndex = vesselIndex;

            if (node.HasValue("targetLatitude") && node.HasValue("targetLongitude"))
            {
                cfg.HasTarget = true;
                node.TryGetValue("targetLatitude", ref cfg.TargetLat);
                node.TryGetValue("targetLongitude", ref cfg.TargetLon);
                node.TryGetValue("targetBody", ref cfg.TargetBody);
            }

            return !string.IsNullOrEmpty(cfg.SaveName);
        }

        /// <summary>
        /// Resolve which protoVessel to focus:
        /// vesselIndex cfg override &gt; vesselName match &gt; flightState.activeVesselIdx.
        /// </summary>
        public static int ResolveVesselIndex(Game game, string vesselName, int vesselIndexOverride)
        {
            if (game == null || game.flightState == null || game.flightState.protoVessels == null)
                return -1;

            var list = game.flightState.protoVessels;

            if (vesselIndexOverride >= 0)
            {
                Debug.Log(string.Format(CultureInfo.InvariantCulture,
                    "[MechJebAutotest] vesselIndex override={0}", vesselIndexOverride));
                return vesselIndexOverride;
            }

            if (!string.IsNullOrEmpty(vesselName))
            {
                for (int i = 0; i < list.Count; i++)
                {
                    ProtoVessel pv = list[i];
                    if (pv == null || string.IsNullOrEmpty(pv.vesselName))
                        continue;
                    if (string.Equals(pv.vesselName, vesselName, StringComparison.OrdinalIgnoreCase))
                    {
                        Debug.Log(string.Format(CultureInfo.InvariantCulture,
                            "[MechJebAutotest] vesselName '{0}' matched idx={1}", vesselName, i));
                        return i;
                    }
                }

                Debug.LogWarning(string.Format(CultureInfo.InvariantCulture,
                    "[MechJebAutotest] vesselName '{0}' not found; falling back to activeVesselIdx",
                    vesselName));
            }

            return game.flightState.activeVesselIdx;
        }

        private void Start()
        {
            if (!CfgExists() && !MechJebAutotestConfig.Ready)
            {
                enabled = false;
                return;
            }

            if (!MechJebAutotestConfig.Ready)
            {
                // Flight started without going through our loader (user loaded save manually with cfg present).
                if (!TryReadCfg(out MechJebAutotestConfigData cfg))
                {
                    enabled = false;
                    return;
                }

                MechJebAutotestConfig.Apply(cfg);
            }

            _timeoutSec     = MechJebAutotestConfig.TimeoutSec;
            _timeoutBaseSec = _timeoutSec;
            _timeoutExtensions = 0;
            _optionalTarget = MechJebAutotestConfig.HasTarget;
            _optLat         = MechJebAutotestConfig.TargetLat;
            _optLon         = MechJebAutotestConfig.TargetLon;
            _optBody        = MechJebAutotestConfig.TargetBody;

            string logDir = Path.Combine(KSPUtil.ApplicationRootPath,
                LogDirRelative.Replace('/', Path.DirectorySeparatorChar));
            Directory.CreateDirectory(logDir);
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _csvPath = Path.Combine(logDir, stamp + ".csv");
            _csv = new StreamWriter(_csvPath, false, Encoding.UTF8);
            _csv.WriteLine(
                "time,autopilot_state,step,altitude,terrain_altitude,vertical_speed,horizontal_speed,distance_to_target,ApA,apoapsis_past_pad,frac,padDh,padDt,t_go,burn_axis,burn_started,zem,zev,a_cmd,handoff,throttle,pitch,a_plan,a_clip,hoop_center,pred_h,hoop_miss");
            _csv.Flush();

            GameEvents.onCrash.Add(OnCrash);
            GameEvents.onCollision.Add(OnCollision);
            GameEvents.onCrashSplashdown.Add(OnCrash);
            _subscribed = true;

            Debug.Log("[MechJebAutotest] Flight harness ready. CSV → " + _csvPath);
            _phase = Phase.WaitReady;
        }

        private void OnDestroy()
        {
            if (_subscribed)
            {
                GameEvents.onCrash.Remove(OnCrash);
                GameEvents.onCollision.Remove(OnCollision);
                GameEvents.onCrashSplashdown.Remove(OnCrash);
                _subscribed = false;
            }

            Finish("destroyed", force: true);
        }

        private void OnCrash(EventReport report)
        {
            Finish("crash");
        }

        private void OnCollision(EventReport report)
        {
            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null || v.state == Vessel.State.DEAD)
                Finish("crash");
        }

        private void Update()
        {
            if (_phase == Phase.Done)
                return;

            if (!FlightGlobals.ready || FlightGlobals.ActiveVessel == null)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (!vessel.loaded || vessel.packed)
                return;

            if (_readyRealtime < 0f)
            {
                _readyRealtime = Time.realtimeSinceStartup;
                Debug.Log(string.Format(CultureInfo.InvariantCulture,
                    "[MechJebAutotest] FlightGlobals.ready + vessel loaded/unpacked; settling {0:F1}s  name='{1}' lat={2:F5} lon={3:F5} alt={4:F1}",
                    FlightSettleSeconds,
                    vessel.vesselName,
                    vessel.latitude,
                    vessel.longitude,
                    vessel.altitude));
            }

            if (Time.realtimeSinceStartup - _readyRealtime < FlightSettleSeconds)
                return;

            MechJebCore core = vessel.GetMasterMechJeb();
            if (core == null)
                return;

            double ut = Planetarium.GetUniversalTime();
            if (_startUt < 0)
            {
                _startUt   = (float)ut;
                _nextLogUt = (float)ut;
            }

            if (_phase == Phase.WaitReady)
            {
                if (_optionalTarget)
                {
                    CelestialBody body = vessel.mainBody;
                    if (!string.IsNullOrEmpty(_optBody))
                    {
                        CelestialBody named = FlightGlobals.Bodies.Find(b =>
                            b != null &&
                            string.Equals(b.bodyName, _optBody, StringComparison.OrdinalIgnoreCase));
                        if (named != null)
                            body = named;
                    }

                    core.Target.SetPositionTarget(body, _optLat, _optLon);
                    if (vessel.LandedOrSplashed)
                        core.Landing.hop = true;
                    double dist = double.NaN;
                    try { dist = core.Landing.getHDistanceToTarget(); }
                    catch (Exception) { /* ignore */ }
                    Debug.Log(string.Format(CultureInfo.InvariantCulture,
                        "[MechJebAutotest] Set position target {0} lat={1} lon={2}  vessel='{3}' dist={4:F1}m",
                        body.bodyName, _optLat, _optLon, vessel.vesselName, dist));
                }
                else if (!core.Target.PositionTargetExists)
                {
                    Debug.LogWarning(
                        "[MechJebAutotest] Save has no position target and cfg has no target* - waiting.");
                    if (ut - _startUt > 30)
                        Finish("no_target");
                    return;
                }

                _phase = Phase.Engage;
                return;
            }

            if (_phase == Phase.Engage)
            {
                if (!core.Target.PositionTargetExists)
                {
                    if (ut - _startUt > 60)
                        Finish("no_target");
                    return;
                }

                if (vessel.LandedOrSplashed)
                    core.Landing.hop = true;

                // Ground Track Landing == LandingType 1 (see MechJebModuleLandingGuidance).
                core.Landing.LandingType.Val = 1;
                core.Landing.LandAtPositionTarget(this);
                Debug.Log("[MechJebAutotest] Engaged LandAtPositionTarget (LandingType=1 Ground Track).");
                _phase     = Phase.Running;
                _nextLogUt = (float)ut;
                return;
            }

            if (_phase == Phase.Running)
            {
                if (vessel.state == Vessel.State.DEAD)
                {
                    Finish("crash");
                    return;
                }

                UpdateLiftoffGate(vessel, core, ut);

                if (_liftoffQualified && vessel.LandedOrSplashed)
                {
                    Finish("landed");
                    return;
                }

                if ((float)ut - _startUt >= _timeoutSec)
                {
                    if (TryExtendTimeout(vessel, core, ut))
                        return;
                    Finish("timeout");
                    return;
                }

                if (ut >= _nextLogUt)
                {
                    LogSample(vessel, core, ut);
                    _nextLogUt = (float)ut + 1f;
                }
            }
        }

        /// <summary>
        /// Near-pad adaptive timeout: when the base (or prior) budget expires, keep flying
        /// if still airborne, descending, landing AP engaged, and impact/landing is soon
        /// (AGL/|vSpd| or pad-proximate). Extend by TimeoutExtendStepSec up to TimeoutHardCapSec.
        /// </summary>
        private bool TryExtendTimeout(Vessel vessel, MechJebCore core, double ut)
        {
            if (_timeoutSec >= TimeoutHardCapSec)
                return false;
            if (vessel == null || vessel.LandedOrSplashed)
                return false;
            if (core == null || core.Landing == null || !core.Landing.Enabled)
                return false;

            double agl = 0;
            double vSpd = vessel.verticalSpeed;
            try
            {
                if (core.VesselState != null)
                {
                    agl = core.VesselState.altitudeTrue;
                    vSpd = core.VesselState.speedVertical;
                }
                else
                    agl = vessel.altitude - vessel.terrainAltitude;
            }
            catch (Exception)
            {
                agl = Math.Max(0, vessel.altitude - vessel.terrainAltitude);
            }

            // Must be descending toward the surface.
            if (!(vSpd < -0.5))
                return false;

            double timeToImpact = agl / Math.Max(-vSpd, 0.1);
            double dist = double.PositiveInfinity;
            try { dist = core.Landing.getHDistanceToTarget(); }
            catch (Exception) { dist = double.PositiveInfinity; }

            bool impactSoon = timeToImpact <= TimeoutImpactHorizonSec;
            bool closeToPad = dist <= TimeoutClosePadM && timeToImpact <= TimeoutImpactHorizonSec * 1.5;
            if (!impactSoon && !closeToPad)
                return false;

            float prev = _timeoutSec;
            float next = Math.Min(TimeoutHardCapSec, _timeoutSec + TimeoutExtendStepSec);
            if (next <= prev)
                return false;

            _timeoutSec = next;
            _timeoutExtensions++;
            string msg = string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] Timeout extended #{0}: {1:F0}s -> {2:F0}s (base={3:F0} cap={4:F0}) elapsed={5:F1}s agl={6:F1}m vSpd={7:F2} tImpact={8:F1}s padDist={9:F1}m",
                _timeoutExtensions, prev, next, _timeoutBaseSec, TimeoutHardCapSec,
                (float)ut - _startUt, agl, vSpd, timeToImpact,
                double.IsInfinity(dist) ? -1.0 : dist);
            Debug.Log(msg);
            if (_csv != null)
            {
                try { _csv.WriteLine(msg); _csv.Flush(); }
                catch (Exception) { /* ignore csv flush errors */ }
            }
            return true;
        }

        private void UpdateLiftoffGate(Vessel vessel, MechJebCore core, double ut)
        {
            if (_liftoffQualified)
                return;

            double terrainAlt = 0;
            try
            {
                if (core != null && core.VesselState != null)
                    terrainAlt = core.VesselState.altitudeTrue;
                else
                    terrainAlt = vessel.altitude - vessel.terrainAltitude;
            }
            catch (Exception)
            {
                terrainAlt = 0;
            }

            bool airborne = !vessel.LandedOrSplashed;
            if (airborne)
            {
                if (_airborneStartUt < 0)
                    _airborneStartUt = ut;

                if ((ut - _airborneStartUt) >= MinAirborneSeconds || terrainAlt >= MinTerrainAltForLiftoff)
                {
                    _liftoffQualified = true;
                    Debug.Log(string.Format(CultureInfo.InvariantCulture,
                        "[MechJebAutotest] Liftoff gate armed (airborne={0:F1}s terrainAlt={1:F1}m)",
                        ut - _airborneStartUt, terrainAlt));
                }
            }
            else
            {
                // Still on the pad / brief unpack flicker — keep waiting.
                _airborneStartUt = -1;
            }
        }

        private void LogSample(Vessel vessel, MechJebCore core, double ut)
        {
            VesselState vs = core.VesselState;
            double alt     = vs.altitudeASL;
            double terrain = vs.altitudeTrue;
            double vSpeed  = vs.speedVertical;
            double hSpeed  = vs.speedSurfaceHorizontal;
            double apa     = vs.orbitApA;
            if (apa > _maxApa)
                _maxApa = apa;

            double dist = 0;
            try
            {
                if (core.Target.PositionTargetExists)
                    dist = core.Landing.getHDistanceToTarget();
            }
            catch (Exception)
            {
                dist = double.NaN;
            }

            string state = "Off";
            string stepName = "";
            string apPast = "";
            string frac = "";
            string padDhS = "";
            string padDtS = "";
            string tGoS = "";
            string burnAxis = "";
            string burnStarted = "";
            string zemS = "";
            string zevS = "";
            string aCmdS = "";
            string aPlanS = "";
            string aClipS = "";
            string handoff = "";
            string hoopCenterS = "";
            string predHS = "";
            string hoopMissS = "";

            if (core.Landing != null)
            {
                if (core.Landing.CurrentStep != null)
                    state = core.Landing.CurrentStep.GetType().Name + ":" + (core.Landing.Status ?? "");
                else if (core.Landing.Enabled)
                    state = "Enabled";

                AutopilotStep cur = core.Landing.CurrentStep;
                if (cur is TargetSubOrbit tsoStep)
                {
                    stepName = tsoStep.AutotestStepName ?? "";
                    if (!double.IsNaN(tsoStep.AutotestPadDh))
                        padDhS = tsoStep.AutotestPadDh.ToString("F1", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(tsoStep.AutotestPadDt))
                        padDtS = tsoStep.AutotestPadDt.ToString("F1", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(tsoStep.AutotestApFrac))
                        frac = tsoStep.AutotestApFrac.ToString("F3", CultureInfo.InvariantCulture);
                    apPast = tsoStep.AutotestApPast ? "1" : "0";
                    burnAxis = tsoStep.AutotestBurnAxis ?? "";
                    handoff = tsoStep.AutotestHandoff ?? "";
                    if (!double.IsNaN(tsoStep.AutotestHoopCenter))
                        hoopCenterS = tsoStep.AutotestHoopCenter.ToString("F1", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(tsoStep.AutotestPredH))
                        predHS = tsoStep.AutotestPredH.ToString("F1", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(tsoStep.AutotestHoopMiss))
                        hoopMissS = tsoStep.AutotestHoopMiss.ToString("F1", CultureInfo.InvariantCulture);
                }
                else if (cur is LandingBurn lbStep)
                {
                    stepName = "LandingBurn";
                    if (!double.IsNaN(lbStep.AutotestPadDh))
                        padDhS = lbStep.AutotestPadDh.ToString("F1", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(lbStep.AutotestTgo))
                        tGoS = lbStep.AutotestTgo.ToString("F2", CultureInfo.InvariantCulture);
                    burnAxis = lbStep.AutotestBurnAxis ?? "";
                    burnStarted = lbStep.AutotestBurnStarted ? "1" : "0";
                    if (!double.IsNaN(lbStep.AutotestZemMag))
                        zemS = lbStep.AutotestZemMag.ToString("F1", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(lbStep.AutotestZevMag))
                        zevS = lbStep.AutotestZevMag.ToString("F1", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(lbStep.AutotestDesiredAccelMag))
                        aCmdS = lbStep.AutotestDesiredAccelMag.ToString("F2", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(lbStep.AutotestAPlan))
                        aPlanS = lbStep.AutotestAPlan.ToString("F2", CultureInfo.InvariantCulture);
                    if (!double.IsNaN(lbStep.AutotestAClip))
                        aClipS = lbStep.AutotestAClip.ToString("F2", CultureInfo.InvariantCulture);
                }
                else if (cur != null)
                {
                    stepName = cur.GetType().Name;
                }

                if (string.IsNullOrEmpty(padDhS) && core.Landing.TryPadOverflightDeltaHeight(out double padDh, out double padDt))
                {
                    padDhS = padDh.ToString("F1", CultureInfo.InvariantCulture);
                    padDtS = padDt.ToString("F1", CultureInfo.InvariantCulture);
                }

                string stepKey = cur != null ? cur.GetType().Name + ":" + stepName : "Off";
                if (!string.IsNullOrEmpty(_prevStepName) && _prevStepName != stepKey)
                {
                    if (string.IsNullOrEmpty(handoff))
                        handoff = _prevStepName + "->" + stepKey;
                    Debug.Log("[Autotest-handoff] " + handoff
                        + " t=" + (ut - _startUt).ToString("F1", CultureInfo.InvariantCulture)
                        + " dist=" + dist.ToString("F0", CultureInfo.InvariantCulture)
                        + " padDh=" + (string.IsNullOrEmpty(padDhS) ? "n/a" : padDhS));
                }
                _prevStepName = stepKey;
            }

            float throttle = vessel.ctrlState != null ? vessel.ctrlState.mainThrottle : 0f;
            double pitch   = vs.vesselPitch;

            string line = string.Format(CultureInfo.InvariantCulture,
                "{0:F3},{1},{2},{3:F3},{4:F3},{5:F3},{6:F3},{7:F3},{8:F3},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20:F3},{21:F3},{22},{23},{24},{25},{26}",
                ut - _startUt,
                CsvEscape(state),
                CsvEscape(stepName),
                alt,
                terrain,
                vSpeed,
                hSpeed,
                dist,
                apa,
                apPast,
                frac,
                padDhS,
                padDtS,
                tGoS,
                CsvEscape(burnAxis),
                burnStarted,
                zemS,
                zevS,
                aCmdS,
                CsvEscape(handoff),
                throttle,
                pitch,
                aPlanS,
                aClipS,
                hoopCenterS,
                predHS,
                hoopMissS);

            _csv.WriteLine(line);
            _csv.Flush();
        }

        private void Finish(string outcome, bool force = false)
        {
            if (_phase == Phase.Done && !force)
                return;
            if (_phase == Phase.Done && _outcome != null && !force)
                return;

            _phase   = Phase.Done;
            _outcome = outcome;

            Vessel vessel = FlightGlobals.ActiveVessel;
            MechJebCore core = vessel != null ? vessel.GetMasterMechJeb() : null;
            double finalDist = double.NaN;
            try
            {
                if (core != null && core.Target != null && core.Target.PositionTargetExists)
                    finalDist = core.Landing.getHDistanceToTarget();
            }
            catch (Exception)
            {
                // ignore
            }

            double elapsed = _startUt >= 0 ? Planetarium.GetUniversalTime() - _startUt : 0;
            double maxApa  = double.IsNegativeInfinity(_maxApa) ? double.NaN : _maxApa;

            string result = string.Format(CultureInfo.InvariantCulture,
                "RESULT outcome={0} final_distance_m={1:F3} max_ApA_m={2:F3} time_s={3:F1} save={4}/{5}",
                outcome, finalDist, maxApa, elapsed,
                MechJebAutotestConfig.SaveFolder, MechJebAutotestConfig.SaveName);

            Debug.Log("[MechJebAutotest] " + result);

            if (_csv != null)
            {
                _csv.WriteLine(result);
                _csv.Flush();
                _csv.Close();
                _csv = null;
            }

            try
            {
                if (core != null && core.Landing != null && core.Landing.Enabled)
                    core.Landing.StopLanding();
            }
            catch (Exception)
            {
                // ignore
            }
        }

        private static string CsvEscape(string s)
        {
            if (string.IsNullOrEmpty(s))
                return "";
            // Flatten newlines/tabs so one sample stays on one CSV row.
            s = s.Replace("\r\n", " ").Replace("\n", " ").Replace("\r", " ").Replace("\t", " ");
            if (s.IndexOfAny(new[] { ',', '"' }) >= 0)
                return "\"" + s.Replace("\"", "\"\"") + "\"";
            return s;
        }
    }

    /// <summary>
    /// Process-lifetime guards and handoff flags for the stock load path.
    /// Static fields survive scene loads within one KSP session.
    /// </summary>
    internal static class MechJebAutotestSession
    {
        private static bool _menuLoadClaimed;

        public static bool PendingSpaceCenterToFlight;

        public static bool TryClaimMenuLoad()
        {
            if (_menuLoadClaimed)
                return false;
            _menuLoadClaimed = true;
            return true;
        }
    }

    /// <summary>Mutable values parsed from autotest.cfg for one run.</summary>
    internal sealed class MechJebAutotestConfigData
    {
        public string SaveFolder  = "ksp-mechjeb";
        public string SaveName    = "";
        public float  TimeoutSec  = 1800f;
        public bool   HasTarget;
        public double TargetLat;
        public double TargetLon;
        public string TargetBody  = "";
        public string VesselName  = "";
        public int    VesselIndex = -1;
    }

    internal static class MechJebAutotestConfig
    {
        public static bool   Ready;
        public static float  TimeoutSec = 1800f;
        public static bool   HasTarget;
        public static double TargetLat;
        public static double TargetLon;
        public static string TargetBody = "";
        public static string SaveFolder = "";
        public static string SaveName   = "";
        public static string VesselName = "";
        public static int    VesselIndex = -1;

        public static void Apply(MechJebAutotestConfigData cfg)
        {
            TimeoutSec  = cfg.TimeoutSec;
            HasTarget   = cfg.HasTarget;
            TargetLat   = cfg.TargetLat;
            TargetLon   = cfg.TargetLon;
            TargetBody  = cfg.TargetBody ?? "";
            SaveFolder  = cfg.SaveFolder ?? "";
            SaveName    = cfg.SaveName ?? "";
            VesselName  = cfg.VesselName ?? "";
            VesselIndex = cfg.VesselIndex;
            Ready       = true;
        }
    }
}
