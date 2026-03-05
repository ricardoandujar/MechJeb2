using Experience.Effects;
using KSP.Localization;
using UnityEngine;
using static alglib;

namespace MuMech
{
    namespace Landing
    {
        public class CourseCorrection : AutopilotStep
        {
            private const float MAX_ERROR_DEFAULT        = 150;
            private const float MAX_LARGE_DISTANCE       = 10000; //  20000; // 40000; // 80000;
            private const float  FAST_SURFACE_SPEED      = 6500;
            private const float DEF_TARGET_PERIAPSIS_PERC= 0.99f; //0.99 <= 0.98f
            const double TIME_CONSTANT = 12.0;// (12 ??)  <=  (try 8.0 no) <= 2.0(too much);// 10.0; //2.0
            const int THRUST_COUNTER = 150;

            private bool _courseCorrectionBurning = false;
            private bool increasePeriapsis = false;
            private float maxError = MAX_ERROR_DEFAULT;
            private int predictionCount = 1000;
            private double timeConstant = TIME_CONSTANT;
            private int thrustCounter = THRUST_COUNTER;
            private Vector3d lastDesiredThrustVector;
            private float targetPeriapsis;
            private int timeCounter = 10000;

            public CourseCorrection(MechJebCore core) : base(core)
            {
                lastDesiredThrustVector = Vector3d.zero;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if ( predictionCount <=0 )
                {
                    float temp = 2;
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    temp.ToString("F1")); //"Performing course correction of about " +  + " m/s"

                    Core.Thrust.ThrustOff();
                    return new DecelerationBurn(Core);
                }
                else if (!Core.Landing.PredictionReady)
                {
                    float temp = 1;
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    temp.ToString("F1")); //"Performing course correction of about " +  + " m/s"

                    predictionCount--; // This limits the wait in case its never ready
                    Core.Thrust.ThrustOff();
                    return this;
                }
                else
                {
                    predictionCount = 1000;
                    float temp = 3;
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    temp.ToString("F1")); //"Performing course correction of about " +  + " m/s"
                }

                // If the atomospheric drag is at least 100mm/s2 then start trying to target the overshoot using the parachutes
                if (Core.Landing.DeployChutes)
                {
                    if (Core.Landing.ParachutesDeployable())
                    {
                        Core.Landing.ControlParachutes();
                    }
                }

                double currentError = Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite);
                maxError = MAX_ERROR_DEFAULT + (MAX_LARGE_DISTANCE - MAX_ERROR_DEFAULT) * Mathf.Clamp01((float)VesselState.speedSurface / FAST_SURFACE_SPEED);
                if (increasePeriapsis == true)
                {
                    if ( VesselState.orbitPeA >= targetPeriapsis)
                    {
                        Core.Thrust.ThrustOff();
                        return new CoastToDeceleration(Core, false);
                    }
                }
                else if ( (currentError < maxError) )
                {
                    //Find the current vector from the planet center to the target landing site
                    Vector3d currentTargetRadialVector =
                        MainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0) - MainBody.position;
                    //Imagine we are going to deorbit now. Find the burn that would lower our periapsis to -10% of the planet's radius
                    Vector3d horizontalDV = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(Orbit, VesselState.time, 0.9 * MainBody.Radius);
                    //Compute the orbit that would put us on
                    Orbit forwardDeorbitTrajectory = Orbit.PerturbedOrbit(VesselState.time, horizontalDV);
                    //Find how long that orbit would take to impact the ground
                    double freefallTime =
                        forwardDeorbitTrajectory.NextTimeOfRadius(VesselState.time, MainBody.Radius) - VesselState.time;
                    //Find how many degrees the planet will rotate during that time
                    double planetRotationDuringFreefall =
                        360 * freefallTime / MainBody.rotationPeriod;
                    //Construct a quaternion representing the rotation of the planet found above
                    var freefallPlanetRotation =
                        Quaternion.AngleAxis((float)planetRotationDuringFreefall, MainBody.angularVelocity);
                    //Use this quaternion to find what the vector from the planet center to the target will be when we hit the ground
                    Vector3d freefallEndTargetRadialVector =
                        freefallPlanetRotation *  currentTargetRadialVector;
                    //Compute the angle between the location of the target at the end of freefall and the normal to our orbit:
                    Vector3d currentRadialVector = VesselState.CoM - MainBody.position;
                    double targetAheadAngle =
                        Vector3d.Angle(currentRadialVector, freefallEndTargetRadialVector); //How far ahead the target is, in degrees
                    if (targetAheadAngle > 60 && Vessel.mainBody.atmDensityASL > 100 && VesselState.drag < 0.1)
                    {
                        increasePeriapsis = true;
                        targetPeriapsis = DEF_TARGET_PERIAPSIS_PERC*(float)VesselState.orbitPeA;
                    }
                    else
                    {
                        Core.Thrust.ThrustOff();
                        if (Core.Landing.RCSAdjustment)
                            Core.RCS.Enabled = true;
                        return new CoastToDeceleration(Core, false);
                    }
                }

                // If we're off course, but already too low, skip the course correction
                if (VesselState.altitudeASL < Core.Landing.DecelerationEndAltitude() + 5)
                {
                    Core.Thrust.ThrustOff();
                    return new DecelerationBurn(Core);
                }

                // If a parachute has already been deployed then we will not be able to control attitude anyway, so move back to the coast to deceleration step.
                if (VesselState.parachuteDeployed)
                {
                    Core.Thrust.ThrustOff();
                    return new CoastToDeceleration(Core, false);
                }

                // We are not in .90 anymore. Turning while under drag is a bad idea
                if (VesselState.drag > 0.1)
                {
                    Core.Thrust.ThrustOff();
                    return new CoastToDeceleration(Core, false);
                }

                if (increasePeriapsis == false)
                {
                    Vector3d deltaV = Core.Landing.ComputeCourseCorrection();

                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                        deltaV.magnitude.ToString("F1")); //"Performing course correction of about " +  + " m/s"
                    Vector3d courseCorrection = Core.Landing.getHDirectionToTarget();
                    // 460k
                    Vector3d desiredThrustVector;
                    if (timeCounter > 0) timeCounter--;
                    if (timeCounter>0)
                        desiredThrustVector = 0.7 * courseCorrection.normalized + 0.3 * deltaV.normalized;
                    else
                        desiredThrustVector = 0.3 * courseCorrection.normalized + 0.7 * deltaV.normalized;
                    // 160k                   Vector3d desiredThrustVector = (0.35 * courseCorrection.normalized + 0.65 * deltaV.normalized).normalized;
                    // Vector3d desiredThrustVector = (0.31 * courseCorrection.normalized + 0.69 * deltaV.normalized).normalized;
                    //                    desiredThrustVector = (0.7*lastDesiredThrustVector + 0.3*desiredThrustVector).normalized;
                    //desiredThrustVector = (0.3 * lastDesiredThrustVector + 0.7 * desiredThrustVector).normalized;
                    Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, Core.Landing);
                    lastDesiredThrustVector = desiredThrustVector;
                    // TODO - Does this matter? Try to remove the attitude angle check and simply rely on accum.
                    // This has been here since day 1 but my change in DecelerationBurn does the same thing and does not use it at all.
                    // On Earth the prediction is way off since the altitude can be much lower - another thing would be to increase the angles
                    // orig = 2/30  - try 5/15  10/50   20/60 and compare results - might also try to increase time constant.
                    //20/60 - still stops and goes for a bit
                    //45/90 - does not seem to be working with time constant 10.0
                    //43/88 - shallow?
                    if (Core.Attitude.attitudeAngleFromTarget() < 45)
                        _courseCorrectionBurning = true;
                    else if (Core.Attitude.attitudeAngleFromTarget() > 90)
                    {
                        _courseCorrectionBurning = false;
                    }

                    if (_courseCorrectionBurning)
                    {
                        Core.Thrust.ThrustForDV(deltaV.magnitude, timeConstant);
                        // Scale down the throttle if we are close to the target
                        if (currentError < 2000000)
                        {
                            Core.Thrust.TargetThrottle *= 0.25f;
                        }

                        // Once the Target Throttle is very small, reduce the time constant to allow finer control.
                        if (Core.Thrust.TargetThrottle < 0.0005f)
                        {
                            // Thrust is too small - keep reducing the time constant
                            // while the the counter is going down set the thrust to
                            // the minimum amount
                            Core.Thrust.TargetThrottle = 0.002f;
                            thrustCounter--;
                            if (thrustCounter <= 0)
                            {
                                timeConstant /= 5.0;
                                thrustCounter = THRUST_COUNTER;
                            }
                        }
                    }
                    else
                    {
                        Core.Thrust.RequestActiveThrottle(0);
                        thrustCounter = THRUST_COUNTER;
                    }
                }

                // Set attitude and thrust to increase periapsis
                else
                {
                    Core.Attitude.attitudeTo(VesselState.orbitalVelocity.normalized, AttitudeReference.INERTIAL, Core.Landing);
                    if (Core.Attitude.attitudeAngleFromTarget() < 5)
                    {
                        float factor = 0.01f+0.5f * Mathf.Clamp01((float)((VesselState.orbitPeA-targetPeriapsis) / (-200000-targetPeriapsis)));
                        Core.Thrust.RequestActiveThrottle(factor * (float)(Core.VesselState.limitedMaxThrustAccel/Core.VesselState.maxThrustAccel) );
                    }
                    else
                    {
                        Core.Thrust.RequestActiveThrottle(0);
                    }
                }

                return this;
            }
        }
    }
}
