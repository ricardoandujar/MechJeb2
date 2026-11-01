using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Unattended vacuum far-hop test harness. Active only when
    /// GameData/MechJeb2/autotest.cfg exists. Dev-branch only.
    ///
    /// Load path mirrors stock Resume Game:
    /// MAINMENU (settle) → LoadGame → SPACECENTER (settle) → StartAndFocusVessel → Flight (ready + settle) → engage.
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

            if (!MechJebAutotest.TryReadCfg(out string saveFolder, out string saveName, out float timeout,
                    out bool hasTarget, out double lat, out double lon, out string body))
            {
                Debug.LogError("[MechJebAutotest] Failed to parse autotest.cfg");
                yield break;
            }

            MechJebAutotestConfig.TimeoutSec = timeout;
            MechJebAutotestConfig.HasTarget  = hasTarget;
            MechJebAutotestConfig.TargetLat  = lat;
            MechJebAutotestConfig.TargetLon  = lon;
            MechJebAutotestConfig.TargetBody = body ?? "";
            MechJebAutotestConfig.SaveFolder = saveFolder;
            MechJebAutotestConfig.SaveName   = saveName;
            MechJebAutotestConfig.Ready      = true;

            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] MainMenu: LoadGame('{0}', '{1}') → SPACECENTER",
                saveName, saveFolder));

            Game game;
            try
            {
                game = GamePersistence.LoadGame(saveName, saveFolder, true, false);
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
                               saveFolder + "/" + saveName);
                yield break;
            }

            HighLogic.CurrentGame = game;
            HighLogic.SaveFolder  = saveFolder;
            game.startScene       = GameScenes.SPACECENTER;
            MechJebAutotestSession.PendingSpaceCenterToFlight = true;
            MechJebAutotestSession.ActiveVesselIdx = game.flightState.activeVesselIdx;

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

            Game game = HighLogic.CurrentGame;
            if (game == null || game.flightState == null)
            {
                Debug.LogError("[MechJebAutotest] SpaceCentre: CurrentGame/flightState missing - cannot start flight.");
                MechJebAutotestSession.PendingSpaceCenterToFlight = false;
                yield break;
            }

            int idx = MechJebAutotestSession.ActiveVesselIdx;
            if (idx < 0)
                idx = game.flightState.activeVesselIdx;

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

        private enum Phase
        {
            WaitReady,
            Engage,
            Running,
            Done
        }

        private Phase   _phase = Phase.WaitReady;
        private float   _timeoutSec = 1800f;
        private float   _startUt = -1f;
        private float   _nextLogUt;
        private double  _maxApa = double.NegativeInfinity;
        private string  _csvPath;
        private StreamWriter _csv;
        private bool    _optionalTarget;
        private double  _optLat, _optLon;
        private string  _optBody = "";
        private string  _outcome;
        private bool    _subscribed;
        private bool    _wasAirborne;
        private float   _readyRealtime = -1f;

        public static string CfgPath =>
            Path.Combine(KSPUtil.ApplicationRootPath, CfgRelativePath.Replace('/', Path.DirectorySeparatorChar));

        public static bool CfgExists() => File.Exists(CfgPath);

        public static bool TryReadCfg(out string saveFolder, out string saveName, out float timeout,
            out bool hasTarget, out double lat, out double lon, out string body)
        {
            saveFolder = "ksp-mechjeb";
            saveName   = "";
            timeout    = 1800f;
            hasTarget  = false;
            lat        = 0;
            lon        = 0;
            body       = "";

            ConfigNode root = ConfigNode.Load(CfgPath);
            if (root == null)
                return false;

            ConfigNode node = root.GetNode("MechJebAutotest") ?? root;
            node.TryGetValue("saveFolder", ref saveFolder);
            node.TryGetValue("saveName", ref saveName);
            node.TryGetValue("timeoutSeconds", ref timeout);

            if (node.HasValue("targetLatitude") && node.HasValue("targetLongitude"))
            {
                hasTarget = true;
                node.TryGetValue("targetLatitude", ref lat);
                node.TryGetValue("targetLongitude", ref lon);
                node.TryGetValue("targetBody", ref body);
            }

            return !string.IsNullOrEmpty(saveName);
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
                if (!TryReadCfg(out string saveFolder, out string saveName, out float timeout,
                        out bool hasTarget, out double lat, out double lon, out string body))
                {
                    enabled = false;
                    return;
                }

                MechJebAutotestConfig.TimeoutSec = timeout;
                MechJebAutotestConfig.HasTarget  = hasTarget;
                MechJebAutotestConfig.TargetLat  = lat;
                MechJebAutotestConfig.TargetLon  = lon;
                MechJebAutotestConfig.TargetBody = body ?? "";
                MechJebAutotestConfig.SaveFolder = saveFolder;
                MechJebAutotestConfig.SaveName   = saveName;
                MechJebAutotestConfig.Ready      = true;
            }

            _timeoutSec     = MechJebAutotestConfig.TimeoutSec;
            _optionalTarget = MechJebAutotestConfig.HasTarget;
            _optLat         = MechJebAutotestConfig.TargetLat;
            _optLon         = MechJebAutotestConfig.TargetLon;
            _optBody        = MechJebAutotestConfig.TargetBody;

            string logDir = Path.Combine(KSPUtil.ApplicationRootPath,
                LogDirRelative.Replace('/', Path.DirectorySeparatorChar));
            Directory.CreateDirectory(logDir);
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            _csvPath = Path.Combine(logDir, stamp + ".csv");
            _csv     = new StreamWriter(_csvPath, false, Encoding.UTF8);
            _csv.WriteLine(
                "time,autopilot_state,altitude,terrain_altitude,vertical_speed,horizontal_speed,distance_to_target,ApA,apoapsis_past_pad,frac,throttle,pitch");
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
                    "[MechJebAutotest] FlightGlobals.ready + vessel loaded/unpacked; settling {0:F1}s",
                    FlightSettleSeconds));
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
                    Debug.Log(string.Format(CultureInfo.InvariantCulture,
                        "[MechJebAutotest] Set position target {0} lat={1} lon={2}",
                        body.bodyName, _optLat, _optLon));
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

                if (!vessel.LandedOrSplashed)
                    _wasAirborne = true;

                if (_wasAirborne && vessel.LandedOrSplashed)
                {
                    Finish("landed");
                    return;
                }

                if ((float)ut - _startUt >= _timeoutSec)
                {
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
            if (core.Landing != null)
            {
                if (core.Landing.CurrentStep != null)
                    state = core.Landing.CurrentStep.GetType().Name + ":" + (core.Landing.Status ?? "");
                else if (core.Landing.Enabled)
                    state = "Enabled";
            }

            string apPast = "";
            string frac   = "";

            float throttle = vessel.ctrlState != null ? vessel.ctrlState.mainThrottle : 0f;
            double pitch   = vs.vesselPitch;

            string line = string.Format(CultureInfo.InvariantCulture,
                "{0:F3},{1},{2:F3},{3:F3},{4:F3},{5:F3},{6:F3},{7:F3},{8},{9},{10:F3},{11:F3}",
                ut - _startUt,
                CsvEscape(state),
                alt,
                terrain,
                vSpeed,
                hSpeed,
                dist,
                apa,
                apPast,
                frac,
                throttle,
                pitch);

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
            if (s.IndexOfAny(new[] { ',', '"', '\n', '\r' }) >= 0)
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
        public static int  ActiveVesselIdx = -1;

        public static bool TryClaimMenuLoad()
        {
            if (_menuLoadClaimed)
                return false;
            _menuLoadClaimed = true;
            return true;
        }
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
    }
}
