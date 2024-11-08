﻿using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class CoastToDeceleration : AutopilotStep
        {
            private const float MAX_ERROR_DEFAULT = 150;
            private const float MAX_LARGE_DISTANCE = 80000;
            private const float FAST_SURFACE_SPEED = 6500;
            private bool courseCorrect;

            public CoastToDeceleration(MechJebCore core, bool correct = true) : base(core)
            {
                courseCorrect = correct;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (!Core.Landing.PredictionReady)
                    return this;

                Vector3d deltaV = Core.Landing.ComputeCourseCorrection(true, 20.0);

                if (!Core.Landing.RCSAdjustment) return this;

                if (deltaV.magnitude > 3)
                    Core.RCS.Enabled = true;
                else if (deltaV.magnitude < 0.01)
                    Core.RCS.Enabled = false;

                if (Core.RCS.Enabled)
                    Core.RCS.SetWorldVelocityError(deltaV);

                return this;
            }

            private bool _warpReady = false;
            private bool warpOn = false;

            public override AutopilotStep OnFixedUpdate()
            {
                Core.Thrust.TargetThrottle = 0;

                // If the atmospheric drag is has started to act on the vessel then we are in a position to start considering when to deploy the parachutes.
                if (Core.Landing.DeployChutes)
                {
                    if (Core.Landing.ParachutesDeployable())
                    {
                        Core.Landing.ControlParachutes();
                    }
                }

                // Move to DecelerationBurn if:
                //    If SurfaceSpeed reaches 90% of Max allowable speed.
                //    If we're already at low altitude, skip directly to the Deceleration burn
                //    If within atmosphere going too fast without heat shields
                double maxAllowedSpeed = Core.Landing.MaxAllowedSpeed();
                if  ( (VesselState.speedSurface > 0.9 * maxAllowedSpeed) ||
                      (VesselState.altitudeASL < Core.Landing.DecelerationEndAltitude() + 5) ||
                      ((VesselState.altitudeASL < MainBody.RealMaxAtmosphereAltitude()) && 
                       (VesselState.speedSurface > Core.Landing.ATMOS_FAST_SPEED)) )
                {
                    Core.Warp.MinimumWarp();
                    if (Core.Landing.RCSAdjustment)
                        Core.RCS.Enabled = false;
                    return new DecelerationBurn(Core);
                }

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status1"); //"Coasting toward deceleration burn"

                if (Core.Landing.LandAtTarget)
                {
                    double currentError = Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite);
                    double maxError = Mathf.Clamp(MAX_ERROR_DEFAULT, MAX_LARGE_DISTANCE, (float)VesselState.speedSurface / FAST_SURFACE_SPEED);
                    if (courseCorrect && currentError > maxError)
                    {
                        if (!VesselState.parachuteDeployed &&
                            VesselState.drag <=
                            0.1) // However if there is already a parachute deployed or drag is high, then do not bother trying to correct the course as we will not have any attitude control anyway.
                        {
                            Core.Warp.MinimumWarp();
                            if (Core.Landing.RCSAdjustment)
                                Core.RCS.Enabled = false;
                            return new CourseCorrection(Core);
                        }
                    }
                    else
                    {
                        Vector3d deltaV = Core.Landing.ComputeCourseCorrection(true, 20.0);
                        Status += "\n" + Localizer.Format("#MechJeb_LandingGuidance_Status2",
                            deltaV.magnitude.ToString("F3")); //"Course correction DV: " +  + " m/s"
                    }
                }

                if ( (Vessel.angularVelocity.magnitude < 0.005f) &&
                     (Core.Attitude.attitudeAngleFromTarget() < 1) ) { _warpReady = true; } // less warp start warp stop jumping

                if (Core.Landing.PredictionReady)
                {
                    if (VesselState.drag < 0.01)
                    {
                        double decelerationStartTime = Core.Landing.Prediction.Trajectory.Any()
                            ? Core.Landing.Prediction.Trajectory.First().UT
                            : VesselState.time;
                        Vector3d decelerationStartAttitude = -Orbit.WorldOrbitalVelocityAtUT(decelerationStartTime);
                        decelerationStartAttitude += MainBody.getRFrmVel(Orbit.WorldPositionAtUT(decelerationStartTime));
                        decelerationStartAttitude =  decelerationStartAttitude.normalized;
                        Core.Attitude.attitudeTo(decelerationStartAttitude, AttitudeReference.INERTIAL, Core.Landing);
                    }
                    else
                    {
                        Core.Attitude.attitudeTo(Vector3.back, AttitudeReference.SURFACE_VELOCITY, Core.Landing);
                    }
                }

                //Warp at a rate no higher than the rate that would have us impacting the ground 10 seconds from now:
                if (_warpReady && Core.Node.Autowarp)
                {
                    // Make sure if we're hovering that we don't go straight into too fast of a warp
                    // (g * 5 is average velocity falling for 10 seconds from a hover)
                    double velocityGuess = Math.Max(Math.Abs(VesselState.speedVertical), VesselState.localg * 5);
                    Core.Warp.WarpRegularAtRate((float)(VesselState.altitudeASL / (10 * velocityGuess)));
                    warpOn = true;
                }
                else if ( warpOn == true )
                {
                    Core.Warp.MinimumWarp();
                    warpOn = false;
                }

                return this;
            }
        }
    }
}
