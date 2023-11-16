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
            private const float MAX_LARGE_DISTANCE       = 80000;
            private const float  FAST_SURFACE_SPEED      = 6500;
            const double TIME_CONSTANT = 10.0;// 2.0(too much);// 10.0; //2.0
            const int THRUST_COUNTER = 150;

            private bool _courseCorrectionBurning = false;
            private float maxError = MAX_ERROR_DEFAULT;
            private int predictionCount = 1000;
            private double timeConstant = TIME_CONSTANT;
            private int thrustCounter = THRUST_COUNTER;

            public CourseCorrection(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if ( predictionCount <=0 )
                {
                    float temp = 2;
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    temp.ToString("F1")); //"Performing course correction of about " +  + " m/s"

                    Core.Thrust.TargetThrottle = 0;
                    return new DecelerationBurn(Core);
                }
                else if (!Core.Landing.PredictionReady)
                {
                    float temp = 1;
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    temp.ToString("F1")); //"Performing course correction of about " +  + " m/s"

                    predictionCount--; // This limits the wait in case its never ready
                    Core.Thrust.TargetThrottle = 0;
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

                maxError = Mathf.Clamp(MAX_ERROR_DEFAULT, MAX_LARGE_DISTANCE, (float)VesselState.speedSurface / FAST_SURFACE_SPEED);
                if ( (currentError < maxError) )
                {
                    Core.Thrust.TargetThrottle = 0;
                    if (Core.Landing.RCSAdjustment)
                        Core.RCS.Enabled = true;
                    return new CoastToDeceleration(Core, false);
                }

                // If we're off course, but already too low, skip the course correction
                if (VesselState.altitudeASL < Core.Landing.DecelerationEndAltitude() + 5)
                {
                    return new DecelerationBurn(Core);
                }

                // If a parachute has already been deployed then we will not be able to control attitude anyway, so move back to the coast to deceleration step.
                if (VesselState.parachuteDeployed)
                {
                    Core.Thrust.TargetThrottle = 0;
                    return new CoastToDeceleration(Core, false);
                }

                // We are not in .90 anymore. Turning while under drag is a bad idea
                if (VesselState.drag > 0.1)
                {
                    return new CoastToDeceleration(Core, false);
                }

                {
                    double perturb = Mathf.Max(1.0f,Mathf.Clamp01((float)MainBody.Radius/6371000.0f) * 250);
                    Vector3d deltaV = Core.Landing.ComputeCourseCorrection(true, perturb);

                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                        deltaV.magnitude.ToString("F1")); //"Performing course correction of about " +  + " m/s"

                    Core.Attitude.attitudeTo(deltaV.normalized, AttitudeReference.INERTIAL, Core.Landing);

                    // TODO - Does this matter? Try to remove the attitude angle check and simply rely on accum.
                    // This has been here since day 1 but my change in DecelerationBurn does the same thing and does not use it at all.
                    // On Earth the prediction is way off since the altitude can be much lower - another thing would be to increase the angles
                    // orig = 2/30  - try 5/15  10/50   20/60 and compare results - might also try to increase time constant.
                    //20/60 - still stops and gos for a bit
                    //45/90 - does not seem to be working with time constant 10.0
                    if (Core.Attitude.attitudeAngleFromTarget() < 45)
                        _courseCorrectionBurning = true;
                    else if (Core.Attitude.attitudeAngleFromTarget() > 90)
                    {
                        _courseCorrectionBurning = false;
                    }

                    if (_courseCorrectionBurning)
                    {
                        Core.Thrust.ThrustForDV(deltaV.magnitude, timeConstant);
                        if (Core.Thrust.TargetThrottle < 0.0005f)
                        {
                            // Thrust is too small - keep reducing the time constant
                            // while the the counter is going down set the thrust to
                            // the minimum amount
                            Core.Thrust.TargetThrottle = 0.001f;
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
                        Core.Thrust.TargetThrottle = 0;
                        thrustCounter = THRUST_COUNTER;
                    }
                }

                return this;
            }
        }
    }
}
