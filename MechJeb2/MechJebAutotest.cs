using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Unattended vacuum far-hop test harness. Active only when
    /// GameData/MechJeb2/autotest.cfg exists. Dev-branch only.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class MechJebAutotestMenuLoader : MonoBehaviour
    {
        private void Start()
        {
            MechJebAutotest.TryLoadFromMenuOrSpaceCenter("MainMenu");
        }
    }

    [KSPAddon(KSPAddon.Startup.SpaceCentre, false)]
    public class MechJebAutotestSpaceCentreLoader : MonoBehaviour
    {
        private void Start()
        {
            MechJebAutotest.TryLoadFromMenuOrSpaceCenter("SpaceCentre");
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class MechJebAutotest : MonoBehaviour
    {
        private const string CfgRelativePath = "GameData/MechJeb2/autotest.cfg";
        private const string LogDirRelative  = "GameData/MechJeb2/autotest";

        private enum Phase
        {
            WaitReady,
            Engage,
            Running,
            Done
        }

        private static bool _loadAttempted;

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

        public static string CfgPath =>
            Path.Combine(KSPUtil.ApplicationRootPath, CfgRelativePath.Replace('/', Path.DirectorySeparatorChar));

        public static bool CfgExists() => File.Exists(CfgPath);

        public static void TryLoadFromMenuOrSpaceCenter(string fromScene)
        {
            if (_loadAttempted)
                return;
            if (!CfgExists())
            {
                Debug.Log("[MechJebAutotest] No autotest.cfg at " + CfgPath + " — harness idle.");
                return;
            }

            _loadAttempted = true;
            if (!TryReadCfg(out string saveFolder, out string saveName, out float timeout,
                    out bool hasTarget, out double lat, out double lon, out string body))
            {
                Debug.LogError("[MechJebAutotest] Failed to parse autotest.cfg");
                return;
            }

            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[MechJebAutotest] {0}: loading save '{1}' from folder '{2}' (timeout={3}s)",
                fromScene, saveName, saveFolder, timeout));

            try
            {
                Game game = GamePersistence.LoadGame(saveName, saveFolder, true, false);
                if (game == null || game.flightState == null)
                {
                    Debug.LogError("[MechJebAutotest] LoadGame returned null/empty flightState for " +
                                   saveFolder + "/" + saveName);
                    return;
                }

                HighLogic.CurrentGame = game;
                HighLogic.SaveFolder  = saveFolder;
                game.startScene       = GameScenes.FLIGHT;
                // Stash optional target for the flight addon via PlayerPrefs-like statics on a holder.
                MechJebAutotestConfig.TimeoutSec   = timeout;
                MechJebAutotestConfig.HasTarget    = hasTarget;
                MechJebAutotestConfig.TargetLat    = lat;
                MechJebAutotestConfig.TargetLon    = lon;
                MechJebAutotestConfig.TargetBody   = body ?? "";
                MechJebAutotestConfig.SaveFolder   = saveFolder;
                MechJebAutotestConfig.SaveName     = saveName;
                MechJebAutotestConfig.Ready        = true;

                FlightDriver.StartAndFocusVessel(game, game.flightState.activeVesselIdx);
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                Debug.LogError("[MechJebAutotest] Exception while loading save: " + e.Message);
            }
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
            // Only treat as terminal if vessel is actually dead / missing.
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
                // Optional target override from cfg when save has none.
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
                        "[MechJebAutotest] Save has no position target and cfg has no target* — waiting.");
                    // Keep waiting a bit; user may have map-target that restores late.
                    if (ut - _startUt > 30)
                    {
                        Finish("no_target");
                    }

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

                // Mirror guidance: hop when starting from the surface.
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

                // Far-hop saves start landed; only count touchdown after we have been airborne.
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
            double alt    = vs.altitudeASL;
            double terrain = vs.altitudeTrue;
            double vSpeed = vs.speedVertical;
            double hSpeed = vs.speedSurfaceHorizontal;
            double apa    = vs.orbitApA;
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

            // ApoapsisPastDestination / frac are private on TargetSubOrbit — not accessible; leave blank.
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

        private static bool TryReadCfg(out string saveFolder, out string saveName, out float timeout,
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


