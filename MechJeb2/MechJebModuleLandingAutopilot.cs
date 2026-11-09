extern alias JetBrainsAnnotations;
using System;
//using System.Collections.Specialized;
//using Contracts.Agents.Mentalities;
using KSP.Localization;
//using MechJebLib.Utils;
using ModuleWheels;
using MuMech.Landing;
//using MuMech.LandingAutopilot;
using UnityEngine;
//using UnityToolbag;
//using static alglib;
//using static FinePrint.ContractDefs;
//using static MuMech.MechJebModuleLandingPredictions;

namespace MuMech
{
    // Landing AP TODO / BUG list
    // - Add a parameter to define how steep the descent will be ( fraction of Orbit ? )
    // - Fix the auto warp stop start dance
    // - Replace the openGL code with a LineRenderer
    // -
    public class MechJebModuleLandingAutopilot : AutopilotModule
    {
        public struct TargetingResult
        {
            public readonly MechJebCore Core;

            public double tUT;
            public double distanceToTarget;
            public Vector3d closestPos;
            public Vector3d surfaceVel;
            public Vector3d targetPos;
            public Vector3d downrangeVec;
            public Vector3d crossrangeVec;
            public Vector3d horizontalVec;
            public double hTargetError;

            public TargetingResult(MechJebCore core)
            {
                Core = core;

                tUT = 0;
                distanceToTarget = 0;
                closestPos = Vector3d.zero;
                surfaceVel = Vector3d.zero;
                targetPos = Vector3d.zero;
                downrangeVec = Vector3d.zero;
                crossrangeVec = Vector3d.zero;
                horizontalVec = Vector3d.zero;
                hTargetError = 0;
            }

            public void update()
            {
                // Get Target position in BODY-FIXED frame
                Core.Landing.ClosestPointToSurfaceTarget(out closestPos, out tUT, out distanceToTarget, out surfaceVel);
                targetPos = Core.Target.GetPositionTargetPosition();
                Core.Landing.GetRangeVectorsToSurfaceTarget(Core.part.vessel, Core.VesselState, targetPos, tUT, out downrangeVec, out crossrangeVec, out horizontalVec);
                hTargetError = horizontalVec.magnitude;

                Core.Landing._predictor.debug1MarkerRadius = 10; // GREEN
                Core.Landing._predictor.debug1Lat = Core.vessel.mainBody.GetLatitude(closestPos);
                Core.Landing._predictor.debug1Lon = Core.vessel.mainBody.GetLongitude(closestPos);
                closestPos = closestPos - Core.vessel.mainBody.position; // Convert to body fixed frame for calculations
            }
        }

        public readonly double LOW_GRAVITY = 1.0;       // Any gravity lower than this is considered low gravity for landing purposes
        public readonly double EARTH_GRAVITY = 9.81;    // Standard Earth gravity in m/sec^2
        public readonly float BASE_STEEPNESS = 1400.0f;  // Base steepness value for the descent profile
        public static readonly float DEFAULT_BASE_SLOPE    = 0.6f;   // Default base slope for the descent profile
        public static readonly float DEFAULT_BASE_MINRATIO = 0.05f;  // Default base min ratio for the descent profile
        public static readonly float DEFAULT_BASE_MAXRATIO = 3.0f;   // Default base max ratio for the descent profile
        public static readonly float DEFAULT_DEORBIT_BURN_ANGLE = 90.0f; // Default angle of the deorbit burn
        public readonly float POST_TARGET_THRESHOLD = 10000.0f;  // 10 km beyond target we switch to move to target
        private const double MIN_TARGET_ALTITUDE = 3500; // Minimum target altitude to consider when calculating optimal deorbit altitude - this is to avoid very low gate altitudes that can cause issues with the landing guidance.
        private bool _deployedGears;  // Have we already deployed the landing gears?
        public  bool LandAtTarget;  // Are we landing at a position target?
        public bool UseOnlyMoveToTarget = false;  // If true we will not do a retro burn first, just go straight to move to target.
        public double g;  // Gravity of body at sea level
        public bool hop = false;  // Is this a hop landing? hop = true means we will be increasing vertical speed first.
        public bool increaseVertical = false;  // Are we currently increasing vertical speed?
        public double TargetAltitude = 0; // Set in constructor
        public TargetingResult targetingResult;

        // ClosestPointToSurfaceTarget is on the FixedUpdate path. A full 50+40+100
        // getPositionAtUT search plus Debug.Log every tick freezes the Unity mouse.
        private bool _closestPointCacheValid;
        private double _closestPointCacheNow;
        private double _closestPointCacheLat;
        private double _closestPointCacheLon;
        private Vector3d _closestPointCachePos;
        private double _closestPointCacheUT;
        private double _closestPointCacheDist;
        private Vector3d _closestPointCacheVel;
        private const double CLOSEST_POINT_REUSE_DT = 0.05;
        private const double CLOSEST_POINT_FULL_PERIOD = 1.0;
        private const bool LOG_CLOSEST_POINT = false;

        // Same-tick reuse for hop + CoastClear. Do not keep this across UT: a 0.05s
        // stale answer would make coast-clear lie about a rim still in the way.
        private bool _avoidCacheValid;
        private double _avoidCacheNow;
        private double _avoidCacheLookAhead;
        private double _avoidCacheMinAlt;
        private double _avoidCacheDV;
        private bool _avoidCacheCrash;

        // Same-UT reuse for intercept-path terrain samples (not per-tick hitch).
        private bool _pathClearCacheValid;
        private double _pathClearCacheNow;
        private double _pathClearCacheMinAlt;
        private bool _pathClearCacheResult;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public readonly EditableDouble TouchdownSpeed = 0.5;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public bool DeployGears = true;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public readonly EditableInt LimitGearsStage = 0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public bool DeployChutes = true;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public readonly EditableInt LimitChutesStage = 0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public bool RCSAdjustment = true;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public bool FlySafe = false;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableInt LandingType = 0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble steepness = 0.6;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble minRatio = DEFAULT_BASE_MINRATIO;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble maxRatio = DEFAULT_BASE_MAXRATIO;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble RadialPercent = 0.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble BurnMarginPerc = 0.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble TargetAltPercent = 0;
        public double TgtAlt;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble deorbitBurnAngle = 90.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble vesselAngle = 90.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble atmosSafeSpeed = 7000;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble TargetOffset = 5.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble LandWithMoverAlt = 100.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble LandWithMoverOffset = 20.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble ZemH = 8.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble ZevH = -1.2;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble ZemV = 5.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble ZevV = -1.4;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug1 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug2 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug3 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug4 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug5 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug6 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug7 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug8 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug91 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug92 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug93 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug94 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug95 = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug96 = 1.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug1h = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug2h = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug3h = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug4h = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug5h = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug6h = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug7h = 1.0;
        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug8h = 1.0;


        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug10 = 1.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug11 = 1.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug12 = 1.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug13 = 1.0;

        [Persistent(pass = (int)(Pass.LOCAL | Pass.TYPE | Pass.GLOBAL))]
        public EditableDouble debug14 = 1.0;

        // This is used to adjust the height at which the parachutes semi deploy as a means of
        // targeting the landing in an atmosphere where it is not possible to control atitude
        // to perform course correction burns.
        private ParachutePlan _parachutePlan;

        //Landing prediction data:
        internal MechJebModuleLandingPredictions _predictor;
        public  ReentrySimulation.Result        Prediction => _predictor.Result;

        private ReentrySimulation.Result _errorPrediction => _predictor.GetErrorResult();

        public bool PredictionReady //We shouldn't do any autopilot stuff until this is true
        {
            get
            {
                // Check that there is a prediction and that it is a landing prediction.
                if (Prediction == null)
                {
                    return false;
                }

                return Prediction.Outcome == ReentrySimulation.Outcome.LANDED;
            }
        }

        // We shouldn't try to use an ErrorPrediction until this is true.
        private bool _errorPredictionReady => _errorPrediction is { Outcome: ReentrySimulation.Outcome.LANDED };

        private double _landingAltitude // The altitude above sea level of the terrain at the landing site
        {
            get
            {
                if (!PredictionReady) return 0;

                // Although we know the landingASL as it is in the prediction, we suspect that
                // it might sometimes be incorrect. So to check we will calculate it again here,
                // and if the two differ log an error. It seems that this terrain ASL calls when
                // made from the simulatiuon thread are regularly incorrect, but are OK when made
                // from this thread. At the time of writting (KSP0.23) there seem to be several
                // other things going wrong with he terrain system, such as visual glitches as
                // we as the occasional exceptions being thrown when calls to the CelestialBody
                // object are made. I suspect a bug or some sort - for now this hack improves
                // the landing results.
                {
                    double checkASL = Prediction.Body.TerrainAltitude(Prediction.EndPosition.Latitude, Prediction.EndPosition.Longitude);
                    // ReSharper disable once CompareOfFloatsByEqualityOperator
                    // ReSharper disable once RedundantCheckBeforeAssignment
                    if (checkASL != Prediction.EndASL)
                    {
                        // I know that this check is not required as we might as well always make
                        // the asignment. However this allows for some debug monitoring of how often this is occuring.
                        Prediction.EndASL = checkASL;
                    }
                }

                return Prediction.EndASL;
            }
        }

        public Vector3d LandingSite =>
            MainBody.GetWorldSurfacePosition(Prediction.EndPosition.Latitude,
                Prediction.EndPosition.Longitude, _landingAltitude) - MainBody.position; // The current position of the landing site

        private Vector3d _rotatedLandingSite => Prediction.WorldEndPosition(); // The position where the landing site will be when we land at it

        public  IDescentSpeedPolicy DescentSpeedPolicy;
        private double              _vesselAverageDrag;

        public MechJebModuleLandingAutopilot(MechJebCore core)
            : base(core)
        {
            targetingResult = new TargetingResult(core);
        }

        public override void OnStart(PartModule.StartState state) => _predictor = Core.GetComputerModule<MechJebModuleLandingPredictions>();

        //public interface:
        public void LandAtPositionTarget(object controller)
        {
            LandAtTarget = true;
            setTargetAltitude();

            // Hop to new target activated - we will increase vertical speed for a while to get a higher landing angle and thus more room for course correction.
            if (hop == true)
            {
                hop = false; // Disable hop - user has to re-activate by selecting new location while landed, then press land at target
                increaseVertical = true;
                UseOnlyMoveToTarget = true;
            }
            else
            {
                increaseVertical = false;    // Default - do not increase vertical speed
                UseOnlyMoveToTarget = false; // Default - Use retroburm before transitioning to move to target
            }

            Users.Add(controller);

            _predictor.Users.Add(this);
            Vessel.RemoveAllManeuverNodes(); // For the benefit of the landing predictions module

            _deployedGears = false;

            g = MainBody.GeeASL * EARTH_GRAVITY; // in m/sec^2

            // For low gravity only use move to target - it appears to work much better than normal landing algos
            if ( (Core.Landing.LandingType == 1) && (g <= LOW_GRAVITY) )
            {
                UseOnlyMoveToTarget = true;
            }

            // Create a new parachute plan
            _parachutePlan = new ParachutePlan(this);
            _parachutePlan.StartPlanning();

            if (Orbit.PeA < 0)
            {
                if (UseOnlyMoveToTarget == true)
                {
                    SetStep(new MoveToTarget(Core));
                }
                else if (Core.Landing.LandingType == 1)
                {
                    SetStep(new TargetSubOrbit(Core));
                }
                else
                {
                    SetStep(new CourseCorrection(Core));
                }
            }
            else if (UseLowDeorbitStrategy())
            {
                SetStep(new PlaneChange(Core));
            }
            else
            {
                SetStep(new DeorbitBurn(Core));
            }
        }

        public void LandUntargeted(object controller)
        {
            hop = false;
            LandAtTarget = false;
            Users.Add(controller);

            _deployedGears = false;

            // Create a new parachute plan
            _parachutePlan = new ParachutePlan(this);
            _parachutePlan.StartPlanning();

            SetStep(new UntargetedDeorbit(Core));
        }

        public void StopLanding()
        {
            Users.Clear();
            Core.Thrust.ThrustOff();
            Core.Thrust.Users.Remove(this);
            if (Core.Landing.RCSAdjustment)
                Core.RCS.Enabled = false;
            SetStep(null);
        }

        public override void Drive(FlightCtrlState s)
        {
            if (!Active)
                return;

            DescentSpeedPolicy = PickDescentSpeedPolicy();

            _predictor.descentSpeedPolicy            = PickDescentSpeedPolicy(); //create a separate IDescentSpeedPolicy object for the simulation
            _predictor.decelEndAltitudeASL           = DecelerationEndAltitude();
            _predictor.parachuteSemiDeployMultiplier = _parachutePlan.Multiplier;

            // Consider lowering the langing gear
            {
                double minalt = Math.Min(VesselState.altitudeBottom, Math.Min(VesselState.altitudeASL, VesselState.altitudeTrue));
                if (DeployGears && !_deployedGears && minalt < 1000)
                    DeployLandingGears();
            }

            base.Drive(s);
        }

        public override void OnFixedUpdate()
        {
            _vesselAverageDrag = VesselAverageDrag();
            base.OnFixedUpdate();
            DeployParachutes();
        }

        protected override void OnModuleEnabled()
        {
            Core.Attitude.Users.Add(this);
            Core.Thrust.Users.Add(this);
        }

        protected override void OnModuleDisabled()
        {
            Core.Attitude.attitudeDeactivate();
            _predictor.Users.Remove(this);
            _predictor.descentSpeedPolicy = null;
            Core.Thrust.ThrustOff();
            Core.Thrust.Users.Remove(this);
            if (Core.Landing.RCSAdjustment)
                Core.RCS.Enabled = false;
            SetStep(null);
        }

        public Vector3d ComputeCourseCorrection(double heightASL = 0, double progradeFraction = 1.0, double radialFraction = 1.0, double lateralFraction = 1.0)
        {
            // Only use the original terrain-based solver when the specified height is equal or less than minimum height
            // AND the periapsis altitude is negative relative to body radius
            if ( (Core.Landing.PredictionReady==true) && (heightASL == 0) && (Orbit.PeA < MainBody.Radius) )
            {
                // 1. Try the original MechJeb landing solver that requires a landing prediction first.
                double landRadius = DecelerationEndAltitude();
                return ComputeCorrectionTerrain(landRadius, progradeFraction, radialFraction, lateralFraction, _rotatedLandingSite);
            }

            // 2. No predicted impact â†’ use the alternate correction method using a shell altitude using heightASL.
            //    This handles all cases where periapsis is above terrain or the solver fails.
            return ComputeCorrectionShellG(heightASL, progradeFraction, radialFraction, lateralFraction);
        }

        // Estimate the delta-V of the correction burn that would be required to put us on
        // course for the target
        public Vector3d ComputeCorrectionTerrain(double heightASL, double progradeFraction, double radialFraction, double lateralFraction, Vector3d rotatedLandingSite, double tUT = 0)
        {
            // actualLandingPosition is the predicted actual landing position
            Vector3d actualLandingPosition = rotatedLandingSite - MainBody.position;

            // orbitLandingPosition is the point where our current orbit intersects the planet
            heightASL = ((heightASL == 0) ? DecelerationEndAltitude() : heightASL);
            double endRadius = MainBody.Radius + heightASL;

            // Return zero vector if already landed.
            if (Vessel.LandedOrSplashed)
                return Vector3d.zero;

            Vector3d orbitLandingPosition;
            if (tUT == 0)
            {
                orbitLandingPosition = Orbit.WorldBCIPositionAtUT(
                    Orbit.PeR < endRadius ? Orbit.NextTimeOfRadius(VesselState.time, endRadius) : Orbit.NextPeriapsisTime(VesselState.time) );
                tUT = Prediction.EndUT;
            }
            else
            {
                orbitLandingPosition = Orbit.WorldBCIPositionAtUT(tUT);
            }

            // convertOrbitToActual is a rotation that rotates orbitLandingPosition on actualLandingPosition
            var convertOrbitToActual = Quaternion.FromToRotation(orbitLandingPosition, actualLandingPosition);

            // Consider the effect small changes in the velocity in each of these three directions
            Vector3d[] perturbationDirections =
            {
                VesselState.surfaceVelocity.normalized, VesselState.radialPlusSurface, VesselState.normalPlusSurface
            };
            double[] perturbationFractions =
            {
                progradeFraction,radialFraction,lateralFraction
            };

            // Compute the effect burns in these directions would
            // have on the landing position, where we approximate the landing position as the place
            // the perturbed orbit would intersect the planet.
            var deltas = new Vector3d[3];
            for (int i = 0; i < 3; i++)
            {
                if (perturbationFractions[i] == 0)
                {
                    deltas[i] = Vector3d.zero;
                    continue; // Skip it if fraction is zero
                }

                //warning: hard experience shows that setting this too low leads to bewildering bugs due to finite precision of Orbit functions
                const double PERTURBATION_DELTA_V = 1;

                Orbit perturbedOrbit =
                    Orbit.PerturbedOrbit(VesselState.time, PERTURBATION_DELTA_V * perturbationDirections[i]*perturbationFractions[i]); //compute the perturbed orbit

                double perturbedLandingTime = perturbedOrbit.PeR < endRadius
                    ? perturbedOrbit.NextTimeOfRadius(VesselState.time, endRadius)
                    : perturbedOrbit.NextPeriapsisTime(VesselState.time);

                Vector3d perturbedLandingPosition = perturbedOrbit.WorldBCIPositionAtUT(perturbedLandingTime); //find where it hits the planet

                //find the difference between that and the original orbit's intersection point
                Vector3d landingDelta = perturbedLandingPosition - orbitLandingPosition;

                //rotate that difference vector so that we can now think of it as starting at the actual landing position
                landingDelta = convertOrbitToActual * landingDelta;

                //project the difference vector onto the plane tangent to the actual landing position
                landingDelta = Vector3d.Exclude(actualLandingPosition, landingDelta);

                //normalize by the delta-V considered, so that deltas now has units of meters per (meter/second) [i.e., seconds]
                deltas[i] = landingDelta / PERTURBATION_DELTA_V;
            }

            // Now deltas stores the predicted offsets in landing position produced by each of the three perturbations.
            // We now figure out the offset we actually want

            // First we compute the target landing position. We have to convert the latitude and longitude of the target
            // into a position. We can't just get the current position of those coordinates, because the planet will
            // rotate during the descent, so we have to account for that.
            Vector3d desiredLandingPosition = MainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, heightASL) - MainBody.position;

            // Moved rotation to its own method so it can be reused else where.
            desiredLandingPosition = RotateRelativePosition(desiredLandingPosition, tUT - VesselState.time);

            Vector3d desiredDelta = desiredLandingPosition - actualLandingPosition;
            desiredDelta = Vector3d.Exclude(actualLandingPosition, desiredDelta);

            // Now desiredDelta gives the desired change in our actual landing position (projected onto a plane
            // tangent to the actual landing position).

            Vector3d downrangeDirection;
            Vector3d downrangeDelta;
            if (progradeFraction > 0)
            {
                // Construct the linear combination of the prograde and radial+ perturbations
                // that produces the largest effect on the landing position. The Math.Sign is to
                // detect and handle the case where radial+ burns actually bring the landing sign closer
                // (e.g. when we are traveling close to straight up)
                downrangeDirection = (deltas[0].magnitude * perturbationDirections[0]
                                      + Math.Sign(Vector3d.Dot(deltas[0], deltas[1])) * deltas[1].magnitude * perturbationDirections[1]).normalized;

                downrangeDelta = Vector3d.Dot(downrangeDirection, perturbationDirections[0]) * deltas[0]
                                 + Vector3d.Dot(downrangeDirection, perturbationDirections[1]) * deltas[1];
            }
            else
            {
                // If we aren't allowed to burn prograde, downrange component of the landing
                // position has to be controlled by radial+/- burns:
                downrangeDirection = perturbationDirections[1];
                downrangeDelta     = deltas[1];
            }

            // Now solve a 2x2 system of linear equations to determine the linear combination
            // of perturbationDirection01 and normal+ that will give the desired offset in the
            // predicted landing position.
            var a = new Matrix2X2(
                downrangeDelta.sqrMagnitude, Vector3d.Dot(downrangeDelta, deltas[2]),
                Vector3d.Dot(downrangeDelta, deltas[2]), deltas[2].sqrMagnitude
            );

            var b = new Vector2d(Vector3d.Dot(desiredDelta, downrangeDelta), Vector3d.Dot(desiredDelta, deltas[2]));

            Vector2d coeffs = a.Inverse() * b;
            Vector3d courseCorrection = coeffs.x * downrangeDirection * 0.707102 + coeffs.y * perturbationDirections[2] * 0.707102;

            // --- 2. Define local reference frame ---
            Vector3d prograde = Vector3d.Exclude(Vessel.up, Vessel.obt_velocity).normalized;
            Vector3d normal = Vector3d.Cross(prograde, Vessel.up).normalized;

            // --- 3. Decompose raw Î”v ---
            double dvRadial = Vector3d.Dot(courseCorrection, Vessel.up);
            double dvPrograde = Vector3d.Dot(courseCorrection, prograde);
            double dvNormal   = Vector3d.Dot(courseCorrection, normal);

            // Scale only radial & prograde
            Vector3d shapedDv =
                (((dvRadial * radialFraction)   * Vessel.up +
                (dvPrograde * progradeFraction) * prograde +
                (dvNormal   * lateralFraction)  * normal).normalized)*courseCorrection.magnitude;

            // Set Debug Vectors as horizontal and vertical components of desired thrust vector
            MechJebModuleDebugArrows.debugVector  = (((dvPrograde * progradeFraction) * prograde) + dvNormal*normal) / shapedDv.magnitude;
            MechJebModuleDebugArrows.debugVector2 = ((dvRadial * radialFraction) * Vessel.up)/ shapedDv.magnitude;

            return shapedDv;
        }

        private Vector3d ComputeCorrectionShellG(double targetHeightASL, double progradeFraction, double radialFraction, double lateralFraction)
        {
            ClosestPointToSurfaceTarget(out Vector3d closestPos, out double tUT, out double distance, out Vector3d surfaceVel);
            _predictor.debug1MarkerRadius = 10;
            _predictor.debug1Lat = MainBody.GetLatitude(closestPos);
            _predictor.debug1Lon = MainBody.GetLongitude(closestPos);

            closestPos = RotateRelativePosition(closestPos - MainBody.position, tUT - Planetarium.GetUniversalTime());
            //            closestPos = closestPos - MainBody.position;
            double height = closestPos.magnitude - MainBody.Radius;
            Vector3d dvSolution;
            if ( (targetHeightASL - height) < 0)
            {
                dvSolution = 100*Mathf.Clamp01((float)(targetHeightASL - height)/(40)) * VesselState.surfaceVelocity.normalized;
            }
            else
            {
                dvSolution = ComputeCorrectionTerrain(targetHeightASL, progradeFraction, radialFraction, lateralFraction, closestPos, tUT);
            }

            if ( Math.Abs(targetHeightASL - height) <= 10000 )
            {
                dvSolution = Vector3d.zero;
            }

            return dvSolution;
        }
        public double ArcDistance(Vector3d a, Vector3d b, double radius)
        {
            Vector3d na = a.normalized;
            Vector3d nb = b.normalized;
            double dot = Math.Max(-1.0, Math.Min(1.0, Vector3d.Dot(na, nb)));
            double angle = Math.Acos(dot);
            return angle * radius;
        }


        /// <summary>
        /// True when apoapsis is an elliptical number intercept math can use.
        /// Hyperbolic/parabolic ApA is Inf or negative; (0.5*Inf)/(0.01*Inf) is NaN.
        /// </summary>
        public static bool ApoapsisIsFinite(Orbit o)
        {
            if (o == null) return false;
            if (double.IsNaN(o.eccentricity) || o.eccentricity >= 1.0) return false;
            double apa = o.ApA;
            return !double.IsNaN(apa) && !double.IsInfinity(apa) && apa > 0;
        }

        public static bool IsFiniteVec(Vector3d p)
        {
            return !double.IsNaN(p.x) && !double.IsNaN(p.y) && !double.IsNaN(p.z)
                && !double.IsInfinity(p.x) && !double.IsInfinity(p.y) && !double.IsInfinity(p.z);
        }

        public void ClosestPointToSurfaceTarget(out Vector3d bestPos, out double bestUT, out double bestDist, out Vector3d surfaceVel, int coarseSamples = 50, double maxHorizon = 3600)
        {
            Orbit orbit = Vessel.orbit;
            CelestialBody body = orbit.referenceBody;
            double R = body.Radius;
            double now = Planetarium.GetUniversalTime();
            double lat = Core.Target.targetLatitude;
            double lon = Core.Target.targetLongitude;
            if (_closestPointCacheValid
                && lat == _closestPointCacheLat && lon == _closestPointCacheLon
                && (now - _closestPointCacheNow) >= 0
                && (now - _closestPointCacheNow) < CLOSEST_POINT_REUSE_DT)
            {
                bestPos = _closestPointCachePos;
                bestUT = _closestPointCacheUT;
                bestDist = _closestPointCacheDist;
                surfaceVel = _closestPointCacheVel;
                return;
            }
            const double tol = 0.05;
            double rNow = orbit.pos.magnitude;
            if (!IsFiniteVec(orbit.pos) || rNow < R) rNow = R;
            double soi = body.sphereOfInfluence;
            double rCap;
            if (!double.IsNaN(soi) && !double.IsInfinity(soi) && soi > R)
                rCap = Math.Min(soi, Math.Max(5.0 * R, 3.0 * rNow));
            else
                rCap = Math.Max(5.0 * R, 3.0 * rNow);
            // --- Helpers -------------------------------------------------------------
            Vector3d TargetPosAtUT(double ut)
            {
                Vector3d targetNow =
                    body.GetWorldSurfacePosition(Core.Target.targetLatitude,
                                                 Core.Target.targetLongitude,
                                                 0.0)
                  - body.position;
                return RotateRelativePosition(targetNow, ut - now);
            }
            double GetDist(double ut)
            {
                Vector3d srcPos = orbit.getPositionAtUT(ut) - body.position;
                if (!IsFiniteVec(srcPos) || srcPos.magnitude < R || srcPos.magnitude > rCap) return double.MaxValue;
                Vector3d tgtPos = TargetPosAtUT(ut);
                if (!IsFiniteVec(tgtPos)) return double.MaxValue;
                return ArcDistance(srcPos, tgtPos, R);
            }
            double workDist = double.MaxValue;
            double workUT = now;
            Vector3d workPos = Vector3d.zero;
            void Consider(double ut)
            {
                double dist = GetDist(ut);
                if (dist >= workDist) return;
                Vector3d srcPos = orbit.getPositionAtUT(ut) - body.position;
                if (!IsFiniteVec(srcPos)) return;
                workDist = dist;
                workUT = ut;
                workPos = srcPos;
            }
            bool needFullSearch = !_closestPointCacheValid
                || lat != _closestPointCacheLat || lon != _closestPointCacheLon
                || (now - _closestPointCacheNow) >= CLOSEST_POINT_FULL_PERIOD;
            if (!needFullSearch)
            {
                workDist = double.MaxValue;
                workUT = now;
                workPos = Vector3d.zero;
                double seedUT = _closestPointCacheUT;
                if (double.IsNaN(seedUT) || double.IsInfinity(seedUT) || seedUT < now)
                    seedUT = now;
                double window = 20.0;
                double aC = Math.Max(now, seedUT - window);
                double bC = seedUT + window;
                if (bC < aC) bC = aC + window;
                double phiC = (1.0 + Math.Sqrt(5.0)) / 2.0;
                double invPhiC = 1.0 / phiC;
                double invPhi2C = invPhiC * invPhiC;
                for (int iter = 0; iter < 25 && (bC - aC) > tol; iter++)
                {
                    double x1 = aC + invPhi2C * (bC - aC);
                    double x2 = aC + invPhiC * (bC - aC);
                    if (GetDist(x1) < GetDist(x2))
                        bC = x2;
                    else
                        aC = x1;
                }
                Consider((aC + bC) / 2.0);
                Consider(seedUT);
                Consider(now);
                if (!IsFiniteVec(workPos) || workDist == double.MaxValue)
                {
                    workUT = now;
                    workPos = orbit.getPositionAtUT(now) - body.position;
                    if (!IsFiniteVec(workPos)) workPos = orbit.pos;
                    workDist = GetDist(now);
                    if (double.IsNaN(workDist) || double.IsInfinity(workDist)) workDist = 0;
                }
                bestDist = workDist;
                bestUT = workUT;
                bestPos = workPos;
                Vector3d velCheap = Vessel.orbit.getOrbitalVelocityAtUT(bestUT);
                Vector3d rotCheap = Vector3d.Cross(MainBody.angularVelocity, bestPos.normalized);
                surfaceVel = velCheap - rotCheap;
                if (LOG_CLOSEST_POINT)
                    Debug.Log("bestUT:" + (bestUT - now).ToString("F1") + " dist:" + bestDist.ToString("F1") + " alt:" + (bestPos.magnitude - R).ToString("F1") + " cheap");
                bestPos += body.position;
                _closestPointCacheValid = true;
                _closestPointCacheNow = now;
                _closestPointCacheLat = lat;
                _closestPointCacheLon = lon;
                _closestPointCachePos = bestPos;
                _closestPointCacheUT = bestUT;
                _closestPointCacheDist = bestDist;
                _closestPointCacheVel = surfaceVel;
                return;
            }
            // --- 1. Binary-search horizon detection ---------------------------------
            double FindImpactUT(double nowUT, double horizon)
            {
                double lo = nowUT;
                double hi = nowUT + horizon;
                Vector3d pHi = orbit.getPositionAtUT(hi) - body.position;
                if (!IsFiniteVec(pHi) || pHi.magnitude > R)
                    return hi;
                for (int i = 0; i < 40; i++)
                {
                    double mid = 0.5 * (lo + hi);
                    Vector3d p = orbit.getPositionAtUT(mid) - body.position;
                    if (!IsFiniteVec(p) || p.magnitude < R)
                        hi = mid;
                    else
                        lo = mid;
                }
                return hi;
            }
            double endUT = FindImpactUT(now, maxHorizon);
            // --- 2. Coarse global search --------------------------------------------
            workDist = double.MaxValue;
            workUT = now;
            workPos = Vector3d.zero;
            double step = (endUT - now) / coarseSamples;
            if (step < 0 || double.IsNaN(step) || double.IsInfinity(step)) step = 0;
            for (int i = 0; i <= coarseSamples; i++)
            {
                double ut = now + i * step;
                Vector3d srcPos = orbit.getPositionAtUT(ut) - body.position;
                if (!IsFiniteVec(srcPos) || srcPos.magnitude > rCap)
                    continue;
                if (srcPos.magnitude < R)
                    break;
                Consider(ut);
            }
            // Hyperbola has no apoapsis; periapsis is the natural closest-to-body time.
            if (orbit.eccentricity >= 1.0)
            {
                double tPe = orbit.timeToPe;
                if (!double.IsNaN(tPe) && !double.IsInfinity(tPe) && tPe >= 0 && now + tPe <= endUT)
                    Consider(now + tPe);
            }
            if (!IsFiniteVec(workPos) || workDist == double.MaxValue)
            {
                workUT = now;
                workPos = orbit.getPositionAtUT(now) - body.position;
                if (!IsFiniteVec(workPos)) workPos = orbit.pos;
                workDist = GetDist(now);
                if (double.IsNaN(workDist) || double.IsInfinity(workDist)) workDist = 0;
            }
            // --- 3. Fine refinement with golden section search ----------------------
            double refineWindow = step * 5.0;
            if (double.IsNaN(refineWindow) || double.IsInfinity(refineWindow) || refineWindow <= 0) refineWindow = 10.0;
            double refineStart = Math.Max(now, workUT - refineWindow);
            double refineEnd = Math.Min(endUT, workUT + refineWindow);
            double phi = (1.0 + Math.Sqrt(5.0)) / 2.0;
            double invPhi = 1.0 / phi;
            double invPhi2 = invPhi * invPhi;
            double a = refineStart;
            double b = refineEnd;
            int maxIter = 100;
            for (int iter = 0; iter < maxIter && (b - a) > tol; iter++)
            {
                double x1 = a + invPhi2 * (b - a);
                double x2 = a + invPhi * (b - a);
                double f1 = GetDist(x1);
                double f2 = GetDist(x2);
                if (f1 < f2)
                {
                    b = x2;
                }
                else
                {
                    a = x1;
                }
            }
            double fineUT = (a + b) / 2.0;
            double fineDist = GetDist(fineUT);
            if (fineDist < workDist)
            {
                workDist = fineDist;
                workUT = fineUT;
                workPos = orbit.getPositionAtUT(workUT) - body.position;
            }
            if (!IsFiniteVec(workPos))
            {
                workUT = now;
                workPos = orbit.pos;
            }
            bestDist = workDist;
            bestUT = workUT;
            bestPos = workPos;
            Vector3d velFuture = Vessel.orbit.getOrbitalVelocityAtUT(bestUT); // Future inertial velocity ---
            Vector3d rotVelFuture = Vector3d.Cross(MainBody.angularVelocity, bestPos.normalized); // Body rotation velocity at that future point ---
            surfaceVel = velFuture - rotVelFuture;// Future surface-relative velocity ---
            if (LOG_CLOSEST_POINT)
                Debug.Log("bestUT:" + (bestUT - now).ToString("F1") + " dist:" + bestDist.ToString("F1") + " alt:" + (bestPos.magnitude - R).ToString("F1"));
            bestPos += body.position;
            _closestPointCacheValid = true;
            _closestPointCacheNow = now;
            _closestPointCacheLat = lat;
            _closestPointCacheLon = lon;
            _closestPointCachePos = bestPos;
            _closestPointCacheUT = bestUT;
            _closestPointCacheDist = bestDist;
            _closestPointCacheVel = surfaceVel;
        }
        // Rotate Body relative position based on it's rotation over input elapsed seconds from now.
        public Vector3d RotateRelativePosition(Vector3d posBodyCentered, double timeDeltaSeconds)
        {
            // Rotates position that is relative to Body forward/backward in time to where it will be after timeDeltaSeconds
            // (positive angle for eastward planet rotation).
            // Input: posBodyCentered = relative to MainBody.position (body-centered)
            //        timeDeltaSeconds = time delta in seconds (positive = forward in time) (negative = backward in time)
            // Output: forward-rotated body-centered position
            double angleDeg = MainBody.angularV * timeDeltaSeconds * Mathf.Rad2Deg;
            QuaternionD rot = QuaternionD.AngleAxis(angleDeg, MainBody.angularVelocity.normalized);

            return rot * posBodyCentered;
        }

        public void ControlParachutes()
        {
            // Firstly - do we have a parachute plan? If not then we had better get one quick!
            _parachutePlan ??= new ParachutePlan(this);

            // Are there any deployable parachute? If not then there is no point in us being here. Let's switch to cruising to the deceleration burn instead.
            if (!ParachutesDeployable())
            {
                _predictor.runErrorSimulations = false;
                _parachutePlan.ClearData();
                return;
            }

            _predictor.runErrorSimulations = true;

            // Is there an error prediction available? If so add that into the mix
            if (_errorPredictionReady && !double.IsNaN(_errorPrediction.ParachuteMultiplier))
            {
                _parachutePlan.AddResult(_errorPrediction);
            }

            // Has the Landing prediction been updated? If so then we can use the result to refine our parachute plan.
            if (PredictionReady && !double.IsNaN(Prediction.ParachuteMultiplier))
            {
                _parachutePlan.AddResult(Prediction);
            }
        }

        private void DeployParachutes()
        {
            if (!(VesselState.mainBody.atmosphere && DeployChutes)) return;

            for (int i = 0; i < VesselState.parachutes.Count; i++)
            {
                ModuleParachute p = VesselState.parachutes[i];
                // what is the ASL at which we should deploy this parachute? It is the actual deployment height above the surface + the ASL of the predicted landing point.
                double landingSiteASL = _landingAltitude;
                double parachuteDeployAboveGroundAtLandingSite = p.deployAltitude * _parachutePlan.Multiplier;

                double aslDeployAltitude = parachuteDeployAboveGroundAtLandingSite + landingSiteASL;

                if (p.part.inverseStage >= LimitChutesStage && p.deploymentState == ModuleParachute.deploymentStates.STOWED &&
                    aslDeployAltitude > VesselState.altitudeASL && p.deploymentSafeState == ModuleParachute.deploymentSafeStates.SAFE)
                {
                    p.Deploy();
                    //Debug.Log("Deploying parachute " + p.name + " at " + ASLDeployAltitude + ". (" + LandingSiteASL + " + " + ParachuteDeployAboveGroundAtLandingSite +")");
                }
            }
        }

        // This methods works out if there are any parachutes that are capable of being deployed
        public bool ParachutesDeployable()
        {
            if (!VesselState.mainBody.atmosphere) return false;
            if (!DeployChutes) return false;

            for (int i = 0; i < VesselState.parachutes.Count; i++)
            {
                ModuleParachute p = VesselState.parachutes[i];
                if (Math.Max(p.part.inverseStage, 0) >= LimitChutesStage && p.deploymentState == ModuleParachute.deploymentStates.STOWED)
                {
                    return true;
                }
            }

            return false;
        }

        // This methods works out if there are any parachutes that have already been deployed (or semi deployed)
        private bool ParachutesDeployed() => VesselState.parachuteDeployed;

        private void DeployLandingGears()
        {
            for (int i = 0; i < Vessel.parts.Count; i++)
            {
                Part p = Vessel.parts[i];
                if (p.HasModule<ModuleWheelDeployment>())
                {
                    // p.inverseStage is -1 for some configuration ?!?
                    if (Math.Max(p.inverseStage, 0) >= LimitGearsStage)
                    {
                        foreach (ModuleWheelDeployment wd in p.FindModulesImplementing<ModuleWheelDeployment>())
                        {
                            if (wd.fsm.CurrentState == wd.st_retracted || wd.fsm.CurrentState == wd.st_retracting)
                                wd.EventToggle();
                        }
                    }
                }
            }

            _deployedGears = true;
        }

        private IDescentSpeedPolicy PickDescentSpeedPolicy()
        {
            if ( Core.Landing.FlySafe == false)
            {
                return new GravityTurnDescentSpeedPolicy(MainBody.Radius + DecelerationEndAltitude(), MainBody.GeeASL * 9.81, VesselState.limitedMaxThrustAccel);
            }
            else
            {
                if (UseAtmosphereToBrake())
                {
                    return new PoweredCoastDescentSpeedPolicy(MainBody.Radius + DecelerationEndAltitude(), MainBody.GeeASL * 9.81,
                        VesselState.limitedMaxThrustAccel);
                }

                return new SafeDescentSpeedPolicy(MainBody.Radius + DecelerationEndAltitude(), MainBody.GeeASL * 9.81, VesselState.limitedMaxThrustAccel);
            }
        }

        public double DecelerationEndAltitude()
        {
            //if the atmosphere is thin, the deceleration burn should end
            //500 meters above the landing site to allow for a controlled final descent
            //MechJebCore.print("DecelerationEndAltitude Vacum " + (200 + LandingAltitude).ToString("F2"));
            if (!UseAtmosphereToBrake()) return 200 + _landingAltitude;

            // if the atmosphere is thick, deceleration (meaning freefall through the atmosphere)
            // should end a safe height above the landing site in order to allow braking from terminal velocity
            // FIXME: Drag Length is quite large now without parachutes, check this better
            double landingSiteDragLength = MainBody.DragLength(_landingAltitude, _vesselAverageDrag + ParachuteAddedDragCoef(), VesselState.mass);

            //MechJebCore.print("DecelerationEndAltitude Atmo " + (2 * landingSiteDragLength + LandingAltitude).ToString("F2"));
            return 1.1 * landingSiteDragLength + _landingAltitude;
        }

        //On planets with thick enough atmospheres, we shouldn't do a deceleration burn. Rather,
        //we should let the atmosphere decelerate us and only burn during the final descent to
        //ensure a safe touchdown speed. How do we tell if the atmosphere is thick enough? We check
        //to see if there is an altitude within the atmosphere for which the characteristic distance
        //over which drag slows the ship is smaller than the altitude above the terrain. If so, we can
        //expect to get slowed to near terminal velocity before impacting the ground.
        public bool UseAtmosphereToBrake()
        {
            double landingSiteDragLength = MainBody.DragLength(_landingAltitude, _vesselAverageDrag + ParachuteAddedDragCoef(), VesselState.mass);

            //if (mainBody.RealMaxAtmosphereAltitude() > 0 && (ParachutesDeployable() || ParachutesDeployed()))
            return MainBody.RealMaxAtmosphereAltitude() > 0 &&
                   landingSiteDragLength < 0.7 * MainBody.RealMaxAtmosphereAltitude(); // the ratio is totally arbitrary until I get something better
        }

        // Get an average drag for the whole vessel. Far from precise but fast.
        private double VesselAverageDrag()
        {
            float dragCoef = 0;
            for (int i = 0; i < Vessel.parts.Count; i++)
            {
                Part p = Vessel.parts[i];
                if (p.DragCubes.None || p.ShieldedFromAirstream)
                {
                    continue;
                }
                //part.DragCubes.SetDrag( -part.partTransform.InverseTransformDirection( vessel.ReferenceTransform.up ) , 1 );

                float partAreaDrag = 0;
                for (int f = 0; f < 6; f++)
                {
                    partAreaDrag = p.DragCubes.WeightedDrag[f] *
                                   p.DragCubes.AreaOccluded[f]; // * PhysicsGlobals.DragCurveValue(0.5, machNumber) but I ll assume it is 1 for now
                }

                dragCoef += partAreaDrag / 6;
            }

            return dragCoef * PhysicsGlobals.DragCubeMultiplier;
        }

        // This is not the exact number, but it's good enough for our use
        private double ParachuteAddedDragCoef()
        {
            double addedDragCoef = 0;
            if (!VesselState.mainBody.atmosphere || !DeployChutes) return addedDragCoef * PhysicsGlobals.DragCubeMultiplier;

            for (int i = 0; i < VesselState.parachutes.Count; i++)
            {
                ModuleParachute p = VesselState.parachutes[i];
                if (p.part.inverseStage < LimitChutesStage) continue;

                //addedDragMass += p.part.DragCubes.Cubes.Where(c => c.Name == "DEPLOYED").m

                float maxCoef = 0;
                for (int c = 0; c < p.part.DragCubes.Cubes.Count; c++)
                {
                    DragCube dragCube = p.part.DragCubes.Cubes[c];
                    if (dragCube.Name != "DEPLOYED")
                        continue;

                    for (int f = 0; f < 6; f++)
                    {
                        // we only want the additional coef from going fully deployed
                        maxCoef = Mathf.Max(maxCoef, p.part.DragCubes.WeightedDrag[f] - dragCube.Weight * dragCube.Drag[f]);
                    }
                }

                addedDragCoef += maxCoef;
            }

            return addedDragCoef * PhysicsGlobals.DragCubeMultiplier;
        }

        private bool UseLowDeorbitStrategy()
        {
            // For planets with atmosphere and the current position is within the atmosphere dont use low deorbit strategy
            // or if starting altitude is closer to atmosphere
            if (MainBody.atmosphere && (VesselState.altitudeASL < 4*MainBody.atmosphereDepth) )
            {
                return false;
            }

            // If Move to Target activated just use low deorbit strategy
            else if (Core.Landing.UseOnlyMoveToTarget)
            {
                return true;
            }

            double periapsisSpeed = Orbit.WorldOrbitalVelocityAtUT(Orbit.NextPeriapsisTime(VesselState.time)).magnitude;
            double stoppingDistance = Math.Pow(periapsisSpeed, 2) / (2 * VesselState.limitedMaxThrustAccel);

            return Orbit.PeA < 2 * stoppingDistance + MainBody.Radius / 4;
        }

        public double MaxAllowedSpeed(double terrainRadius = 0) => DescentSpeedPolicy.MaxAllowedSpeed(VesselState.CoM - MainBody.position, VesselState.surfaceVelocity, terrainRadius);

        public double MaxAllowedSpeedAfterDt(double dt) =>
            DescentSpeedPolicy.MaxAllowedSpeed(VesselState.CoM + VesselState.orbitalVelocity * dt - MainBody.position,
                VesselState.surfaceVelocity + dt * VesselState.gravityForce);

        [ValueInfoItem("#MechJeb_ParachuteControlInfo", InfoItem.Category.Misc, showInEditor = false)] //ParachuteControlInfo
        public string ParachuteControlInfo()
        {
            if (!ParachutesDeployable()) return "N/A";

            string retVal = Localizer.Format("#MechJeb_ChuteMultiplier", _parachutePlan.Multiplier.ToString("F7")); //"'Chute Multiplier: " +
            retVal += Localizer.Format("#MechJeb_MultiplierQuality",
                _parachutePlan.MultiplierQuality.ToString("F1"));                                         //"\nMultiplier Quality: " +  + "%"
            retVal += Localizer.Format("#MechJeb_Usingpredictions", _parachutePlan.MultiplierDataAmount); //"\nUsing " +  + " predictions"

            return retVal;
        }

        public void SetTargetKSC(MechJebCore controller)
        {
            Users.Add(controller);
            Core.Target.SetPositionTarget(MainBody, MechJebModuleLandingGuidance.LandingSites[0].Latitude,
                MechJebModuleLandingGuidance.LandingSites[0].Longitude);
        }


        /// Get the horizontal distance to target without any body rotation adjustments
        public double getHDistanceToTarget()
        {
            // Use target altitude - We only want the horizontal component
            double asl = Core.vessel.mainBody.TerrainAltitude(Core.Target.targetLatitude, Core.Target.targetLongitude);

            // Get the relative surface positions
            Vector3d vesselRelPos = Core.vessel.mainBody.GetRelSurfacePosition(Core.vessel.latitude, Core.vessel.longitude, asl);
            Vector3d targetRelPos = Core.vessel.mainBody.GetRelSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, asl);
            double angle = Vector3d.Angle(vesselRelPos, targetRelPos) * Math.PI / 180.0;
            return (angle * (Core.vessel.mainBody.Radius + asl));
        }

        /// Get the horizontal 3d vector that points to the target without any body rotation adjustments
        public Vector3d getHDirectionToTarget()
        {
            // Use target altitude - We only want the horizontal component
            double asl = Core.vessel.mainBody.TerrainAltitude(Core.Target.targetLatitude, Core.Target.targetLongitude);

            // Get the horizontal direction vector to target
            return Vector3d.Exclude(VesselState.up, Core.vessel.mainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, asl)
                - Core.vessel.mainBody.GetWorldSurfacePosition(Core.vessel.latitude, Core.vessel.longitude, asl));
        }

        /// <summary>
        /// Computes the circular orbit speed at a given altitude above a celestial body.
        /// </summary>
        /// <param name="altitude">The altitude above the body's surface in meters.</param>
        /// <param name="body">The celestial body around which the orbit occurs.</param>
        /// <returns>The circular orbit speed in meters per second.</returns>
        /// <remarks>
        /// This calculation is based on the vis-viva equation simplified for circular orbits: v = sqrt(Î¼ / r),
        /// where Î¼ is the gravitational parameter (GM) and r is the orbital radius (body radius + altitude).
        /// </remarks>
        public double GetCircularOrbitSpeed(double altitude, CelestialBody body)
        {
            if (body == null)
            {
                throw new ArgumentNullException(nameof(body));
            }
            if (altitude < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(altitude), "Altitude must be non-negative.");
            }

            double mu = body.gravParameter;
            double radius = body.Radius;
            double r = radius + altitude;
            return Math.Sqrt(mu / r);
        }

        /// <summary>
        /// Computes the downrange and crossrange vectors from the vessel to a fixed surface target on the vessel's main body,
        /// based on shortest arc angular distances.
        /// </summary>
        /// <param name="vessel">The current vessel in orbit.</param>
        /// <param name="targetLatitude">The target latitude in degrees.</param>
        /// <param name="targetLongitude">The target longitude in degrees.</param>
        /// <param name="targetAltitude">The target altitude above sea level in meters. Defaults to 0 (surface level).</param>
        /// <returns>A tuple containing the downrange vector and crossrange vector as Vector3d.</returns>
        /// <remarks>
        /// Vectors are computed using angular decompositions in the orbital frame:
        /// - Downrange: r * phi_down * t_hat, where phi_down is the signed in-plane angular separation.
        /// - Crossrange: r * phi_cross * c_hat, where phi_cross is the signed out-of-plane angular separation.
        /// The magnitudes represent arc distances along the orbit at radius r.
        /// Positive downrange indicates the target is ahead along the prograde direction.
        /// Assumes the vessel is in a valid orbit with non-zero angular momentum.
        /// </remarks>
        /// <exception cref="ArgumentNullException">Thrown if the vessel is null.</exception>
        /// <exception cref="InvalidOperationException">Thrown if the vessel has no orbit, main body, or insufficient angular momentum.</exception>
        public void GetRangeVectorsToSurfaceTarget(Vessel vessel, VesselState vstate, Vector3d targetPos, double tUT, out Vector3d downrangeVec, out Vector3d crossrangeVec, out Vector3d horizontalVec)
        {
            //targetPos = Core.Landing.RotateRelativePosition(targetPos, (tUT - Planetarium.GetUniversalTime()));
            //Vector3d vesselPos = Core.Landing.RotateRelativePosition(vessel.mainBody.GetWorldSurfacePosition(vessel.latitude, vessel.longitude, vessel.terrainAltitude) - vessel.mainBody.position, (tUT - Planetarium.GetUniversalTime()));
            Vector3d vesselPos = vessel.mainBody.GetWorldSurfacePosition(vessel.latitude, vessel.longitude, vessel.terrainAltitude) - vessel.mainBody.position;
            Vector3d rel = targetPos - vesselPos;
            double angle = Vector3d.Angle(vesselPos, targetPos) * Math.PI / 180.0;
            horizontalVec = (Vector3d.Exclude(vstate.up, rel)).normalized * angle * (vessel.mainBody.Radius + vessel.terrainAltitude);
            Vector3d hvel = Vector3d.Exclude(vstate.up, vessel.srf_velocity).normalized;
            Vector3d track_dir = hvel;
            Vector3d cross_dir = Vector3d.Cross(vstate.up, hvel).normalized;
            downrangeVec  = track_dir * Vector3d.Dot(horizontalVec, track_dir);
            crossrangeVec = cross_dir * Vector3d.Dot(horizontalVec, cross_dir);
        }

        /// <summary>
        /// Gravity-scaled pad-overflight / deorbit gate (baseline all-body formula).
        /// hGate = max(MIN_TARGET_ALTITUDE, vCirc^2/(2*aNet)*S). Low-g / high-TWR bodies
        /// floor at MIN_TARGET_ALTITUDE (3500); higher-g / lower-TWR bodies rise with hVertical*S
        /// (Earth higher than Vesta). Design TSO->LB overhead constraint - not a fixed meter value.
        /// </summary>
        public double ComputeOptimalDeorbitAltitude()
        {
            CelestialBody body = Vessel.mainBody;
            Vector3d targetPos = Core.Target.GetPositionTargetPosition(); // Target position in body fixed frame

            // --- Current orbital radius ---
            Vector3d vesselPos = VesselState.CoM - body.position;
            double r = vesselPos.magnitude;

            // --- Circular orbital speed (shallow deorbit assumption) ---
            double mu = body.gravParameter;
            double vCirc = Math.Sqrt(mu / r);

            // --- Local gravity at target ---
            double g = mu / (targetPos.sqrMagnitude);

            // --- Max thrust acceleration (fixed at 20 m/s^2) ---
            double aMax = VesselState.limitedMaxThrustAccel;
            double aNet = aMax - g;
            if (aNet <= 0.0)
                return double.PositiveInfinity;

            // --- Suicide-burn altitude (vertical-only model) ---
            double hVertical = (vCirc * vCirc) / (2.0 * aNet);

            // --- Horizontal scaling factor ---
            double S = 0.035; // 0.035 <= 0.05

            // --- Final optimal gate altitude ---
            double hGate = Math.Max(MIN_TARGET_ALTITUDE, hVertical * S);

            return hGate;
        }

        /// <summary>
        /// Minimum safe coast height over the pad for LandingBurn ZEM-ZEV braking room.
        /// Same all-body gravity-scaled gate as ComputeOptimalDeorbitAltitude (not a fixed 2400).
        /// Low-g floors at MIN_TARGET_ALTITUDE; higher-g / lower-TWR bodies rise with hVertical*S.
        /// Vesta evaluates to the 3500 floor, covering ~2400 m low-g LB headroom after band.
        /// </summary>
        public double MinSafeLandingBurnOverheadM()
        {
            double h = ComputeOptimalDeorbitAltitude();
            if (double.IsNaN(h) || double.IsInfinity(h) || h <= 0)
                return MIN_TARGET_ALTITUDE;
            return h;
        }

        /// <summary>
        /// Never let TgtAlt sit below the gravity-scaled LB overhead gate (crush / bump-cap / band).
        /// </summary>
        public void EnsureTgtAltMeetsLbOverheadFloor()
        {
            double floor = MinSafeLandingBurnOverheadM();
            if (TgtAlt < floor)
                TgtAlt = floor;
        }

        void setTargetAltitude()
        {
            if (MainBody.atmosphere == true)
            {
                TargetAltitude = 0.5 * MainBody.atmosphereDepth;
            }
            else
            {
                TargetAltitude = Math.Max(5000.0, 0.023 * MainBody.Radius);
            }

            // Set target Altitude directly overhead to target
            if (Core.Landing.TargetAltPercent <= 0)
            {
                Core.Landing.TgtAlt = ComputeOptimalDeorbitAltitude();
            }
            else
            {
                Core.Landing.TgtAlt = Core.Landing.TargetAltPercent * TargetAltitude / 100.0;
            }
        }

        //
        // This is used to calculate a positive deltav to avoid collision with terrain in the future
        // Will probably work best when attempting to move up but there is a steep hill ahead, but may
        // be needed on the way down as long as we are not about to land.
        //
        public (double deltaV, bool willCrash) PredictAvoidCollisionDeltaV(double lookAheadSec, double minAltitude)
        {
            // Hop + CoastClear both call this every Drive with (15, 1500). Reuse one
            // evaluation per UT so the heading samples do not run 3x, but never answer
            // from a previous tick (that is how a cache makes coast-clear lie).
            double now = Planetarium.GetUniversalTime();
            if (_avoidCacheValid
                && now == _avoidCacheNow
                && lookAheadSec == _avoidCacheLookAhead
                && minAltitude == _avoidCacheMinAlt)
                return (_avoidCacheDV, _avoidCacheCrash);

            CelestialBody body = Vessel.mainBody;
            double vv = Vessel.verticalSpeed;
            double aNet = VesselState.limitedMaxThrustAccel - VesselState.localg;
            if (!(aNet > 0.05)) aNet = 0.05;
            double vesselASL = VesselState.altitudeASL;
            Vector3d vesselWorld = VesselState.CoM;
            Vector3d up = VesselState.up;
            double worstDeficit = 0;
            bool willCrash = false;

            void ConsiderTrajectoryAgl(double agl)
            {
                if (double.IsNaN(agl) || double.IsInfinity(agl)) return;
                double deficit = minAltitude - agl;
                if (deficit > worstDeficit) worstDeficit = deficit;
                double stopDist = (vv < 0) ? (vv * vv) / (2.0 * aNet) : 0;
                if ((agl - stopDist) < 0) willCrash = true;
            }

            // Keplerian samples along the current trajectory. One 15s point over the
            // crater floor reports AGL vs the floor and misses a rim a few km away.
            if (lookAheadSec > 0 && Vessel.orbit != null)
            {
                const int nTime = 6;
                for (int i = 1; i <= nTime; i++)
                {
                    double dt = lookAheadSec * i / nTime;
                    Vector3d pos = Vessel.orbit.getPositionAtUT(now + dt);
                    if (!IsFiniteVec(pos)) continue;
                    double posASL = body.GetAltitude(pos);
                    double terrainASL = body.TerrainAltitude(pos);
                    ConsiderTrajectoryAgl(posASL - terrainASL);
                }
            }

            // Heading samples: compare OUR ASL to terrain ASL ahead. A wall higher
            // than we are is a hop even if t+15s is still over the hole with AGL
            // above minAltitude. Do not mark willCrash here: a distant rim is climbable.
            // LOCAL terrain only: never scale/cap lookRange with hTargetError (distance
            // to the landing site). Climb-out from the departure crater must see the
            // current rim from the floor; coupling to a far pad collapsed or skewed
            // that look and broke the first climb when the target was hundreds of km out.
            double lookRange = Math.Max(4000.0, 8.0 * Math.Max(50.0, VesselState.altitudeTrue));
            lookRange = Math.Max(lookRange, VesselState.speedSurfaceHorizontal * Math.Max(lookAheadSec, 15.0));
            // Was max(20 km, 0.15*R) capped at 80 km + callers' minAltitude 1500:
            // cleared the floor but held vertical-first / over-hopped past the rim.
            // Dialed: max(10 km, 0.08*R) capped at 40 km. Still local-only (no
            // hTargetError). Callers use a lower minAltitude so H returns sooner.
            double localRimLook = Math.Min(40000.0, Math.Max(10000.0, body.Radius * 0.08));
            lookRange = Math.Max(lookRange, localRimLook);
            lookRange = Math.Min(lookRange, Math.Max(25000.0, localRimLook));

            void SampleHeading(Vector3d dir)
            {
                if (dir.sqrMagnitude < 1e-6) return;
                dir.Normalize();
                // Geometric steps from 250 m; allow enough doublings to reach localRimLook
                // (8 steps tops out ~32 km).
                double d = 250.0;
                int maxSamples = lookRange > 40000.0 ? 12 : 8;
                for (int i = 0; i < maxSamples && d <= lookRange + 1.0; i++, d *= 2.0)
                {
                    Vector3d sampleWorld = vesselWorld + dir * d;
                    if (!IsFiniteVec(sampleWorld)) continue;
                    double terrainASL = body.TerrainAltitude(sampleWorld);
                    if (double.IsNaN(terrainASL) || double.IsInfinity(terrainASL)) continue;
                    double clearance = vesselASL - terrainASL;
                    double deficit = minAltitude - clearance;
                    if (deficit > worstDeficit) worstDeficit = deficit;
                }
            }

            SampleHeading(targetingResult.horizontalVec);
            Vector3d hVel = Vector3d.Exclude(up, VesselState.horizontalSurface);
            if (hVel.sqrMagnitude > 1.0)
            {
                Vector3d toTarget = targetingResult.horizontalVec;
                double align = 1.0;
                if (toTarget.sqrMagnitude > 1e-6)
                    align = Vector3d.Dot(hVel.normalized, toTarget.normalized);
                if (align < 0.95)
                    SampleHeading(hVel);
            }

            double reqV = 0;
            if (worstDeficit > 0)
            {
                reqV = Math.Sqrt(2.0 * aNet * worstDeficit) - vv;
                if (reqV < 0) reqV = 0;
            }

            _avoidCacheValid = true;
            _avoidCacheNow = now;
            _avoidCacheLookAhead = lookAheadSec;
            _avoidCacheMinAlt = minAltitude;
            _avoidCacheDV = reqV;
            _avoidCacheCrash = willCrash;
            return (reqV, willCrash);
        }
        // Pad false-fire: same 5 km band as MoveToTarget CoastClear / hop suppress.
        private const double LANDING_BURN_PAD_FALSE_FIRE_M = 5000.0;
        // Match MoveToTarget AVOID_MIN_ALTITUDE for suborbit path terrain.
        private const double SUBORBIT_PATH_MIN_ALTITUDE = 900.0;

        /// <summary>
        /// True when the coast from now to ClosestPoint does not clip terrain before the
        /// pad. Local PredictAvoid is unchanged (no hTargetError in lookRange). Samples
        /// within 5 km of the pad are ignored. Near-pad overall is treated clear.
        /// Cached once per UT.
        /// </summary>
        public bool SubOrbitPathClearToTarget(double minAltitude = -1)
        {
            if (minAltitude < 0) minAltitude = SUBORBIT_PATH_MIN_ALTITUDE;
            return SubOrbitPathClearToTargetUncached(minAltitude);
        }

        private bool SubOrbitPathClearToTargetUncached(double minAltitude)
        {
            ClosestPointToSurfaceTarget(out Vector3d closestPos, out double tUT, out double distanceToTarget, out Vector3d surfaceVel);
            Vector3d targetPos = Core.Target.GetPositionTargetPosition();
            GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, tUT, out Vector3d downrangeVec, out Vector3d crossrangeVec, out Vector3d horizontalVec);
            if (horizontalVec.magnitude <= LANDING_BURN_PAD_FALSE_FIRE_M)
                return true;

            (double deltaV, bool willCrash) = PredictAvoidCollisionDeltaV(15, minAltitude);
            if (willCrash || deltaV > 0)
                return false;

            return FarInterceptPathClear(tUT, targetPos, minAltitude);
        }

        // Keplerian samples along the intercept only (PredictAvoid already did local).
        private bool FarInterceptPathClear(double tUT, Vector3d targetPos, double minAltitude)
        {
            double now = Planetarium.GetUniversalTime();
            if (_pathClearCacheValid
                && now == _pathClearCacheNow
                && minAltitude == _pathClearCacheMinAlt)
                return _pathClearCacheResult;

            bool result = FarInterceptPathClearUncached(tUT, targetPos, minAltitude, now);
            _pathClearCacheValid = true;
            _pathClearCacheNow = now;
            _pathClearCacheMinAlt = minAltitude;
            _pathClearCacheResult = result;
            return result;
        }

        private bool FarInterceptPathClearUncached(double tUT, Vector3d targetPos, double minAltitude, double now)
        {
            if (double.IsNaN(tUT) || double.IsInfinity(tUT) || tUT <= now + 1.0)
                return true;

            CelestialBody body = Vessel.mainBody;
            Orbit orbit = Vessel.orbit;
            if (orbit == null) return true;

            double endUT = Math.Min(tUT, now + 900.0);
            const int nSamples = 10;
            double span = endUT - now;
            if (span < 1.0) return true;

            for (int i = 1; i <= nSamples; i++)
            {
                double ut = now + span * i / nSamples;
                Vector3d pos = orbit.getPositionAtUT(ut);
                if (!IsFiniteVec(pos)) continue;

                Vector3d posBody = pos - body.position;
                Vector3d tgtNow = targetPos - body.position;
                Vector3d tgtAtUt = RotateRelativePosition(tgtNow, ut - now);
                if (!IsFiniteVec(posBody) || !IsFiniteVec(tgtAtUt)) continue;

                double arcToPad = ArcDistance(posBody, tgtAtUt, body.Radius);
                if (arcToPad <= LANDING_BURN_PAD_FALSE_FIRE_M)
                    continue;

                double posASL = body.GetAltitude(pos);
                double terrainASL = body.TerrainAltitude(pos);
                if (double.IsNaN(posASL) || double.IsInfinity(posASL)
                    || double.IsNaN(terrainASL) || double.IsInfinity(terrainASL))
                    continue;

                if ((posASL - terrainASL) < minAltitude)
                    return false;
            }

            return true;
        }

        /// <summary>
        /// LandingBurn ZEM/ZEV assumes descending into the pad (v_down = Max(0,-vy)).
        /// Allow entry only when descending and ClosestPoint is still ahead of the ship
        /// (not an overhead pass with Ap / intercept past the destination).
        /// localTerrainBlocked: near-term PredictAvoid wants a hop (MoveToTarget).
        /// pathTerrainBlocked: intercept samples clip before the pad (stay, raise path).
        /// </summary>
        // Match TargetSubOrbit DELTA_TARGET_ALT_THRESHOLD: low-side TgtAlt margin for ZEM-ZEV.
        private const double LANDING_BURN_TGTALT_LOW_MARGIN_M = 600.0;

        /// <summary>
        /// Estimated coast height delta at pad overflight: padDh = TgtAlt - heightOverPad.
        /// Same orbit-normal along-track approach as TargetSubOrbit.TryPadOverflightGate.
        /// </summary>
        public bool TryPadOverflightDeltaHeight(out double padDh, out double padDt)
        {
            padDh = double.NaN;
            padDt = double.NaN;

            Vector3d targetPosBody = Core.Target.GetPositionTargetPosition();
            if (!IsFiniteVec(targetPosBody))
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

            Vector3d a = Vector3d.Exclude(n, pos);
            Vector3d b = Vector3d.Exclude(n, targetPosBody);
            if (a.sqrMagnitude < 1e-12 || b.sqrMagnitude < 1e-12)
                return false;
            a.Normalize();
            b.Normalize();
            double sin = Vector3d.Dot(n, Vector3d.Cross(a, b));
            double cos = Vector3d.Dot(a, b);
            double angPad = Math.Atan2(sin, cos);
            double alongTrack = angPad * r;
            if (alongTrack <= 0.0)
                return false;

            double dt = alongTrack / hOrbSpeed;
            if (dt < 5.0)
                dt = 5.0;
            if (dt > 600.0)
                return false;

            double nowUt = Planetarium.GetUniversalTime();
            Vector3d atPad = Vessel.orbit.getPositionAtUT(nowUt + dt) - Vessel.mainBody.position;
            if (!IsFiniteVec(atPad))
                return false;

            double heightOverPad = atPad.magnitude - targetPosBody.magnitude;
            if (double.IsNaN(heightOverPad) || double.IsInfinity(heightOverPad))
                return false;

            padDh = TgtAlt - heightOverPad;
            padDt = dt;
            return !double.IsNaN(padDh) && !double.IsInfinity(padDh);
        }

        public bool LandingBurnEntryAllowed(out bool localTerrainBlocked, out bool pathTerrainBlocked)
        {
            return LandingBurnEntryAllowedCore(out localTerrainBlocked, out pathTerrainBlocked, out _, enforceRangeGate: true);
        }

        /// <summary>
        /// TargetSubOrbit completed pad@TgtAlt with Ap before pad: hand LandingBurn without the
        /// far-range brake gate (that gate blocked legitimate TSO finish at 40-90 km on Vesta).
        /// Still requires descent, intercept ahead, low-side TgtAlt margin, and terrain-clear path.
        /// </summary>
        public bool LandingBurnEntryAllowedFromTargetSubOrbit(
            out bool localTerrainBlocked, out bool pathTerrainBlocked, out string detail)
        {
            return LandingBurnEntryAllowedCore(out localTerrainBlocked, out pathTerrainBlocked, out detail, enforceRangeGate: false);
        }

        private bool LandingBurnEntryAllowedCore(
            out bool localTerrainBlocked, out bool pathTerrainBlocked, out string detail, bool enforceRangeGate)
        {
            localTerrainBlocked = false;
            pathTerrainBlocked = false;
            detail = "";

            // Do not teach LandingBurn to burn while climbing hard into the pad.
            // Strict v<0 when enforceRangeGate (non-TSO path).
            // TSO direct handoff: gravity-scaled crest â€” allow small ascent if ballistic
            // apex gain h_gain=v^2/(2g) stays inside min(100 m, band/6) and remaining
            // high-side padDh room (band = +/- LANDING_BURN_TGTALT_LOW_MARGIN_M).
            // Vesta g~0.25: 100 m allows ~7 m/s (covers the +1.1 crest that blocked 004036);
            // Kerbin g~9.8: 100 m allows ~44 m/s (not a multi-second free climb).
            if (enforceRangeGate)
            {
                if (!(VesselState.speedVertical < 0))
                {
                    detail = "ascending";
                    return false;
                }
            }
            else
            {
                double vSpdCrest = VesselState.speedVertical;
                double gCrest = Math.Max(VesselState.localg, 0.05);
                double hGain = 0.0;
                double tApCoast = 0.0;
                if (vSpdCrest > 0.0)
                {
                    hGain = (vSpdCrest * vSpdCrest) / (2.0 * gCrest);
                    tApCoast = vSpdCrest / gCrest;
                    const double crestGainCapM = 100.0;
                    double hGainAllow = Math.Min(crestGainCapM, LANDING_BURN_TGTALT_LOW_MARGIN_M / 6.0);
                    if (TryPadOverflightDeltaHeight(out double padDhCrest, out _))
                    {
                        // High side of +/-600 band is padDh >= -600; climb reduces padDh.
                        double roomHigh = LANDING_BURN_TGTALT_LOW_MARGIN_M + padDhCrest;
                        if (roomHigh > 0.0)
                            hGainAllow = Math.Min(hGainAllow, roomHigh);
                    }
                    if (hGain > hGainAllow)
                    {
                        detail = "ascending-hard v=" + vSpdCrest.ToString("F1")
                            + " hGain=" + hGain.ToString("F1")
                            + " tAp=" + tApCoast.ToString("F1")
                            + " allow=" + hGainAllow.ToString("F1");
                        return false;
                    }
                }
                // Stash crest metrics for the ok detail line below.
                detail = "crest v=" + vSpdCrest.ToString("F1")
                    + " hGain=" + hGain.ToString("F1")
                    + " tAp=" + tApCoast.ToString("F1");
            }

            ClosestPointToSurfaceTarget(out Vector3d closestPos, out double tUT, out double distanceToTarget, out Vector3d surfaceVel);
            double now = Planetarium.GetUniversalTime();

            // Overhead pass: closest approach already behind us.
            if (!double.IsNaN(tUT) && !double.IsInfinity(tUT) && tUT < now - 1.0)
            {
                detail = "intercept-behind";
                return false;
            }

            // Finite Ap still ahead of the intercept: ApA is after the destination.
            if (ApoapsisIsFinite(Vessel.orbit))
            {
                double tAp = VesselState.orbitTimeToAp;
                if (!double.IsNaN(tAp) && !double.IsInfinity(tAp) && tAp > 0 && tAp < 600
                    && !double.IsNaN(tUT) && !double.IsInfinity(tUT)
                    && tUT < now + tAp)
                {
                    detail = "ap-before-intercept tAp=" + tAp.ToString("F0");
                    return false;
                }
            }

            // Low-side TgtAlt margin: do not enter while heightOverPad < TgtAlt-600 (padDh > 600).
            // ZEM-ZEV needs room as LandingBurn reduces height over pad.
            // Absolute floor: +/-600 band must not accept heightOverPad below the gravity-scaled
            // ComputeOptimal gate (Vesta ~3500 floor covers ~2400 m low-g LB braking headroom).
            if (TryPadOverflightDeltaHeight(out double padDhEntry, out _))
            {
                if (padDhEntry > LANDING_BURN_TGTALT_LOW_MARGIN_M)
                {
                    detail = "padDh-too-low padDh=" + padDhEntry.ToString("F0");
                    return false;
                }
                double heightOverPadEntry = TgtAlt - padDhEntry;
                double minOverhead = MinSafeLandingBurnOverheadM();
                if (heightOverPadEntry < minOverhead)
                {
                    detail = "overhead-too-low h=" + heightOverPadEntry.ToString("F0")
                        + " min=" + minOverhead.ToString("F0")
                        + " padDh=" + padDhEntry.ToString("F0");
                    return false;
                }
            }

            Vector3d targetPos = Core.Target.GetPositionTargetPosition();
            GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, tUT, out Vector3d downrangeVec, out Vector3d crossrangeVec, out Vector3d horizontalVec);

            // Gravity/TWR-aware range gate: blocks premature LB from incomplete far TSO.
            // Skipped for LandingBurnEntryAllowedFromTargetSubOrbit after TSO finished pad@TgtAlt.
            double aMaxEntry = Math.Max(VesselState.limitedMaxThrustAccel, 0.1);
            double gEntry = Math.Max(VesselState.localg, 0.05);
            double twrEntry = aMaxEntry / gEntry;
            double fEntry = 0.35 + 0.35 * Math.Min(1.0, 2.0 / Math.Sqrt(Math.Max(twrEntry, 1.0)));
            double vHEntry = Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity).magnitude;
            double stopDist = (vHEntry * vHEntry) / Math.Max(2.0 * fEntry * aMaxEntry, 0.1);
            double rangeMargin = Math.Max(3000.0, Math.Min(40000.0, 3000.0 * Math.Sqrt(Math.Max(twrEntry, 1.0) / 2.0)));
            const double kStop = 2.5;
            double rangeLimit = kStop * stopDist + rangeMargin;
            if (enforceRangeGate
                && horizontalVec.magnitude > LANDING_BURN_PAD_FALSE_FIRE_M
                && horizontalVec.magnitude > rangeLimit)
            {
                detail = "range-gate dist=" + horizontalVec.magnitude.ToString("F0")
                    + " limit=" + rangeLimit.ToString("F0");
                return false;
            }

            if (horizontalVec.magnitude <= LANDING_BURN_PAD_FALSE_FIRE_M)
            {
                detail = "near-pad dist=" + horizontalVec.magnitude.ToString("F0");
                return true;
            }

            (double deltaV, bool willCrash) = PredictAvoidCollisionDeltaV(15, SUBORBIT_PATH_MIN_ALTITUDE);
            if (willCrash || deltaV > 0)
            {
                localTerrainBlocked = true;
                detail = "local-terrain dV=" + deltaV.ToString("F0");
                return false;
            }

                        if (!FarInterceptPathClear(tUT, targetPos, SUBORBIT_PATH_MIN_ALTITUDE))
            {
                pathTerrainBlocked = true;
                detail = "path-terrain";
                return false;
            }

            string crestInfo = detail.StartsWith("crest ", StringComparison.Ordinal) ? (" " + detail) : "";
            detail = "ok dist=" + horizontalVec.magnitude.ToString("F0")
                + " rangeGate=" + (enforceRangeGate ? "on" : "off")
                + crestInfo;
            return true;
        }
    }


    //A descent speed policy that gives the max safe speed if our entire velocity were straight down
    internal class SafeDescentSpeedPolicy : IDescentSpeedPolicy
    {
        private double _terrainRadius;
        private readonly double _g;
        private readonly double _thrust;

        public SafeDescentSpeedPolicy(double terrainRadius, double g, double thrust)
        {
            _terrainRadius = terrainRadius;
            _g             = g;
            _thrust        = thrust;
        }

        public double MaxAllowedSpeed(Vector3d pos, Vector3d vel, double terrainRadius = 0)
        {
            if (terrainRadius > 0) _terrainRadius = terrainRadius;
            double altitude = pos.magnitude - _terrainRadius;
            return 0.9 * Math.Sqrt(2 * (_thrust - _g) * altitude);
        }
    }

    internal class PoweredCoastDescentSpeedPolicy : IDescentSpeedPolicy
    {
        private float _terrainRadius;
        private readonly float _g;
        private readonly float _thrust;

        public PoweredCoastDescentSpeedPolicy(double terrainRadius, double g, double thrust)
        {
            _terrainRadius = (float)terrainRadius;
            _g             = (float)g;
            _thrust        = (float)thrust;
        }

        public double MaxAllowedSpeed(Vector3d pos, Vector3d vel, double terrainRadius = 0)
        {
            if (terrainRadius > 0) _terrainRadius = (float)terrainRadius;
            if (_terrainRadius < pos.magnitude)
                return double.MaxValue;

            double vSpeed = Vector3d.Dot(vel, pos.normalized);
            double toF = (vSpeed + Math.Sqrt(vSpeed * vSpeed + 2 * _g * (pos.magnitude - _terrainRadius))) / _g;

            //MechJebCore.print("ToF = " + ToF.ToString("F2"));
            return 0.8 * (_thrust - _g) * toF;
        }
    }

    internal class GravityTurnDescentSpeedPolicy : IDescentSpeedPolicy
    {
        private double _terrainRadius;
        private readonly double _g;
        private readonly double _thrust;

        public GravityTurnDescentSpeedPolicy(double terrainRadius, double g, double thrust)
        {
            _terrainRadius = terrainRadius;
            _g             = g;
            _thrust        = thrust;
        }

        public double MaxAllowedSpeed(Vector3d pos, Vector3d vel, double terrainRadius = 0)
        {
            if (terrainRadius > 0) _terrainRadius = terrainRadius;
            //do a binary search for the max speed that avoids death
            double maxFallDistance = pos.magnitude - _terrainRadius;

            double lowerBound = 0;
            double upperBound = 2.0 * vel.magnitude; // 2 <= 1.1

            while (upperBound - lowerBound > 0.01) // 0.01 <= 0.05 <= 0.1
            {
                double test = (upperBound + lowerBound) / 2;
                if (GravityTurnFallDistance(pos, test * vel.normalized) < maxFallDistance) lowerBound = test;
                else upperBound                                                                       = test;
            }

            return 0.96 * ((upperBound + lowerBound) / 2);
        }

        private double GravityTurnFallDistance(Vector3d x, Vector3d v)
        {
            double startRadius = x.magnitude;

            const int STEPS = 20;
            for (int i = 0; i < STEPS; i++)
            {
                Vector3d gVec = -_g * x.normalized;
                Vector3d thrustVec = -_thrust * v.normalized;
                double dt = 1.0 / (STEPS - i) * (v.magnitude / _thrust);
                Vector3d newV = v + dt * (thrustVec + gVec);
                x += dt * (v + newV) / 2;
                v =  newV;
            }

            double endRadius = x.magnitude;

            endRadius -= v.sqrMagnitude / (2 * (_thrust - _g));

            return startRadius - endRadius;
        }
    }

    // Class to hold all the information about the planned deployment of parachutes.
    internal class ParachutePlan
    {
        // We use a linear regression to calculate the best place to deploy based on previous predictions
        private LinearRegression _regression;

        //  store the last result so that we can check if any new result is actually a new one, or the same one again.
        private ReentrySimulation.Result _lastResult;

        //  store the last error result so that we can check if any new error result is actually a new one, or the same one again.
        private ReentrySimulation.Result _lastErrorResult;

        private readonly CelestialBody                 _body;
        private readonly MechJebModuleLandingAutopilot _autoPilot;
        private          bool                          _parachutePresent;
        private          double                        _maxSemiDeployHeight;
        private          double                        _minSemiDeployHeight;
        private          double                        _maxMultiplier;

        // This is the correlation coefficient of the dataset, and is used to tell if the data set is providing helpful information or not.
        // It is exposed outside the class as a "quality percentage" where -1 -> 100% and 0 or more -> 0%
        private double _correlation;

        private const int DATA_SET_SIZE = 40;

        public double Multiplier { get; private set; }

        public int MultiplierDataAmount => _regression.DataSetSize;

        public double MultiplierQuality
        {
            get
            {
                double corr = _correlation;
                if (double.IsNaN(corr)) return 0;
                return Math.Max(0, corr * -100);
            }
        }

        // Incorporates a new simulation result into the simulation data set and calculate a new semi deployment multiplier.
        // If the data set has a poor correlation, then it might just leave the mutiplier.
        // If the correlation becomes positive then it will clear the dataset and start again.
        public void AddResult(ReentrySimulation.Result newResult)
        {
            // if this result is the same as the old result, then it is not new!
            if (newResult.MultiplierHasError)
            {
                if (_lastErrorResult != null)
                {
                    if (newResult.ID == _lastErrorResult.ID) { return; }
                }

                _lastErrorResult = newResult;
            }
            else
            {
                if (_lastResult != null)
                {
                    if (newResult.ID == _lastResult.ID) { return; }
                }

                _lastResult = newResult;
            }

            // What was the overshoot for this new result?
            double overshoot = newResult.GetOvershoot(_autoPilot.Core.Target.targetLatitude, _autoPilot.Core.Target.targetLongitude);

            //Debug.Log("overshoot: " + overshoot.ToString("F2") + " multiplier: " + newResult.parachuteMultiplier.ToString("F4") + " hasError:" + newResult.multiplierHasError);

            // Add the new result to the linear regression
            _regression.Add(overshoot, newResult.ParachuteMultiplier);

            // What is the correlation coefficent of the data. If it is weak a correlation then we will dismiss the dataset and use it to change the current multiplier
            _correlation = _regression.CorrelationCoefficient;
            if (_correlation > -0.2) // TODO this is the best value to test for non-correlation?
            {
                // If the correlation is less that 0 then we will give up controlling the parachutes and throw away the dataset. Also check that we have got several bits of data, just in case we get two datapoints that are badly correlated.
                if (_correlation > 0 && _regression.DataSetSize > 5)
                {
                    ClearData();
                    // Debug.Log("Giving up control of the parachutes as the data does not correlate: " + correlation);
                }
                // Debug.Log("Ignoring the simulation dataset because the correlation is not significant enough: " + correlation);
            }
            else
            {
                // How much data is there? If just one datapoint then we need to slightly vary the multiplier to avoid doing exactly the same multiplier again and getting a divide by zero!. If there is just two then we will not update the multiplier as we can't conclude much from two points of data!
                int dataSetSize = _regression.DataSetSize;
                switch (dataSetSize)
                {
                    case 1:
                        Multiplier *= 0.99999;
                        break;
                    case 2:
                        // Doing nothing
                        break;
                    default:
                        // Use the linear regression to give us a new prediction for when to open the parachutes
                        try
                        {
                            Multiplier = _regression.YIntercept;
                        }
                        catch (Exception)
                        {
                            // If there is not enough data then we expect an exception. However we need to vary the multiplier everso slightly so that we get different data in order to start generating data. This should never happen as we have already checked the size of the dataset.
                            Multiplier *= 0.99999;
                        }

                        break;
                }

                // Impose sensible limits on the multiplier
                if (Multiplier < 1 || double.IsNaN(Multiplier)) { Multiplier = 1; }

                if (Multiplier > _maxMultiplier) { Multiplier = _maxMultiplier; }
            }
        }

        public ParachutePlan(MechJebModuleLandingAutopilot autopliot)
        {
            // Create the linear regression for storing previous prediction results
            _regression = new LinearRegression(DATA_SET_SIZE); // Store the previous however many predictions

            // Take a reference to the landing autopilot module that we are working for.
            _autoPilot = autopliot;

            // Take a note of which body this parachute plan is for. If we go to a different body, we will need a new plan!
            _body = autopliot.Vessel.orbit.referenceBody;
        }

        // Throw away any old data, and create a new empty dataset
        public void ClearData() =>
            // Create the linear regression for storing previous prediction results
            _regression = new LinearRegression(DATA_SET_SIZE); // Stored the previous 20 predictions

        public void StartPlanning()
        {
            // what is the highest point at which we could semi deploy? - look at all the parachutes in the craft, and consider the lowest semi deployment pressure.
            float minSemiDeployPressure = 0;
            float maxFullDeployHeight = 0;
            _parachutePresent = false; // First assume that there are no parachutes.

            // TODO should we check if each of these parachutes is withing the staging limit?
            for (int i = 0; i < _autoPilot.VesselState.parachutes.Count; i++)
            {
                ModuleParachute p = _autoPilot.VesselState.parachutes[i];
                if (p.minAirPressureToOpen > minSemiDeployPressure)
                    // Although this is called "minSemiDeployPressure" we want to find the largest value for each of our parachutes. This can be used to calculate the corresponding height, and hence a height at which we can be guarenteed that all our parachutes will deploy if asked to.
                {
                    minSemiDeployPressure = p.minAirPressureToOpen;
                }

                if (p.deployAltitude > maxFullDeployHeight)
                {
                    maxFullDeployHeight = p.deployAltitude;
                }

                _parachutePresent = true;
            }

            // If parachutes are present on the craft then work out the max / min semideployment heights and the starting value.
            if (_parachutePresent)
            {
                // TODO is there benefit in running an initial simulation to calculate the height at which the ratio between vertical and horizontal velocity would be the best for being able to deply the chutes to control the landing site?

                // At what ASL height does the reference body have this pressure?
                _maxSemiDeployHeight = _body.AltitudeForPressure(minSemiDeployPressure);

                // We have to have semi deployed by the time we fully deploy.
                _minSemiDeployHeight = maxFullDeployHeight;

                _maxMultiplier = _maxSemiDeployHeight / _minSemiDeployHeight;

                // Set the inital multiplier to be the mid point.
                Multiplier = _maxMultiplier / 2;
            }
        }
    }

    // A class to hold a set of x,y data and perform linear regression analysis on it
    internal class LinearRegression
    {
        private readonly double[] _x;
        private readonly double[] _y;
        private readonly int      _maxDataPoints;
        private          int      _currentDataPoint;
        private          double   _sumX;
        private          double   _sumY;
        private          double   _sumXx;
        private          double   _sumXY;
        private          double   _sumYy;

        public LinearRegression(int maxDataPoints)
        {
            _maxDataPoints    = maxDataPoints;
            DataSetSize       = 0;
            _currentDataPoint = -1;

            _x = new double[maxDataPoints];
            _y = new double[maxDataPoints];
        }

        // Add a new data point. It might be that this will replace an old data point, or it might be that this is the first or second datapoints
        public void Add(double x, double y)
        {
            _currentDataPoint++;
            DataSetSize++;

            // wrap back to the beginning if we have got the end of the array
            if (_currentDataPoint >= _maxDataPoints)
            {
                _currentDataPoint = 0;
            }

            // If we have maxed out the number of data points then we need to remove the old values from the running totals
            if (DataSetSize > _maxDataPoints)
            {
                DataSetSize = _maxDataPoints;
            }

            _x[_currentDataPoint] = x;
            _y[_currentDataPoint] = y;

            // Calculate the new totals
            _sumX  = 0;
            _sumY  = 0;
            _sumXx = 0;
            _sumXY = 0;
            _sumYy = 0;

            for (int i = 0; i < DataSetSize; i++)
            {
                double thisx = _x[i];
                double thisy = _y[i];

                _sumX  += thisx;
                _sumXx += thisx * thisx;
                _sumY  += thisy;
                _sumYy += thisy * thisy;
                _sumXY += thisx * thisy;
            }
        }

        private double _slope
        {
            get
            {
                // Require at least half the datapoints in the dataset
                if (DataSetSize < 2)
                {
                    throw new Exception("Not enough data to calculate trend line");
                }

                double result = (_sumXY - _sumX * _sumY / DataSetSize) / (_sumXx - _sumX * _sumX / DataSetSize);

                return result;
            }
        }

        public double YIntercept
        {
            get
            {
                double result = _sumY / DataSetSize - _slope * (_sumX / DataSetSize);

                return result;
            }
        }

        public int DataSetSize { get; private set; }

        // Calculation the Pearson product-moment correlation coefficient
        public double CorrelationCoefficient
        {
            get
            {
                double stdX = Math.Sqrt(_sumXx / DataSetSize - _sumX * _sumX / DataSetSize / DataSetSize);
                double stdY = Math.Sqrt(_sumYy / DataSetSize - _sumY * _sumY / DataSetSize / DataSetSize);
                double covariance = _sumXY / DataSetSize - _sumX * _sumY / DataSetSize / DataSetSize;

                return covariance / stdX / stdY;
            }
        }
    }
}
