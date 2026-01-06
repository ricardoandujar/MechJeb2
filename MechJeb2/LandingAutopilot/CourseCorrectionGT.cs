using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using DDSHeaders;
using KSP.Localization;
using MuMech.Landing;
using UnityEngine;
using static alglib;

namespace MuMech.LandingAutopilot
{
    public class CourseCorrectionGT : AutopilotStep
    {
        private enum Step
        {
            STEP1_TIME_TO_BURN_START, // Calculate timeToBurnUT
            STEP2_TIME_TO_BURN,       // Warping to TIME TO BURN
            STEP3_BURN,               // Burn
            STEP4_CALC_NEXT_STEP      // Determine Next Step
        };

        private LandingEquations landEq;
        private Step step;
        private Vector3d attitude;

        public CourseCorrectionGT(MechJebCore core) : base(core)
        {
            landEq = new LandingEquations();
            step = Step.STEP1_TIME_TO_BURN_START;
            attitude = -Core.vessel.srf_velocity.normalized;
        }
        public override AutopilotStep Drive(FlightCtrlState s)
        {
            if (step == Step.STEP3_BURN && Core.Attitude.attitudeAngleFromTarget() < 5)
            {
                Core.Thrust.RequestActiveThrottle((float)VesselState.limitedMaxThrustAccel);
            }
            else
            {
                Core.Thrust.ThrustOff();
            }

            Core.Attitude.attitudeTo(attitude, AttitudeReference.INERTIAL, Core.Landing);
            return this;
        }

        public override AutopilotStep OnFixedUpdate()
        {
            double latErrorDeg = 0;
            double lonErrorDeg = 0;
            CelestialBody body = Core.vessel.mainBody;
            double peDesAlt = (body.atmosphere) ? body.atmosphereDepth / 2 : body.Radius * 0.0144873;
            double timeToBurn = 0;
            double timeToBurnUT = 0;
            double tAlt = body.TerrainAltitude(Core.Target.targetLatitude, Core.Target.targetLongitude);
            double h = Core.vessel.terrainAltitude - tAlt;
            landEq.StartSamplePeriod(h, VesselState.dragUp, VesselState.mass, VesselState.speedVertical, VesselState.limitedMaxThrustAccel, VesselState.localg);

            switch (step)
            {
                // Calculate time to burn
                case Step.STEP1_TIME_TO_BURN_START:
                    latErrorDeg = landEq.ComputeLatError(Core.vessel, Core.Target.targetLatitude);
                    timeToBurn = landEq.TimeToNodeForSurfacePoint(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude);
                    if (timeToBurn == double.NaN)
                    {
                        // Orbit is coplanar with target - go to next step
                        step = Step.STEP4_CALC_NEXT_STEP;
                    }
                    else
                    {
                        step = Step.STEP2_TIME_TO_BURN;
                    }
                    break;

                // Warp to burn location
                case Step.STEP2_TIME_TO_BURN:
                    bool warpReady = ((Vector3.Scale(Core.vessel.angularVelocity, new Vector3(1f, 0f, 1f)).magnitude < 0.001) && (Core.Attitude.attitudeAngleFromTarget() < 5));
                    timeToBurn = landEq.TimeToNodeForSurfacePoint(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude);
                    timeToBurnUT = VesselState.time + timeToBurn;
                    if (warpReady && Core.Node.Autowarp && (timeToBurn > 60))
                    {
                        Core.Warp.WarpToUT(timeToBurnUT - 60);
                    }
                    else
                    {
                        Core.Warp.MinimumWarp();
                        attitude = landEq.NodeBasedAlignmentAttitude2(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude);
                        if (timeToBurn < 1)
                        {
                            step = Step.STEP3_BURN;
                        }
                    }
                    break;

                case Step.STEP3_BURN:
                    timeToBurn = 0;
                    attitude = landEq.NodeBasedAlignmentAttitude2(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude, timeToBurnUT);
                    latErrorDeg = landEq.ComputeLatError(Core.vessel, Core.Target.targetLatitude);

                    if ((Math.Abs(latErrorDeg) < 0.1))
                    {
                        step = Step.STEP4_CALC_NEXT_STEP;
                    }
                    break;

                default:
                case Step.STEP4_CALC_NEXT_STEP:
                    return new DeorbitBurnGT(Core);
            }
                
            // LAT/LONG ERR(deg):<<1>>/<<2>>
            Status = Localizer.Format("#MechJeb_LandingGuidance_Status18",
                latErrorDeg.ToString("F1"), lonErrorDeg.ToString("F1"), timeToBurn.ToString("F1"));

            return this;
        }
    }
}
