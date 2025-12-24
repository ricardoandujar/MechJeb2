using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KSP.Localization;
using MuMech.LandingAutopilot;
using UnityEngine;


namespace MuMech.Landing
{
    public class DeorbitBurnGT : AutopilotStep
    {
        private enum Step
        {
            STEP1_FIRST_TIME_TO_BURN, // First time getting time to burn
            STEP2_TIME_TO_BURN,       // Warping to TIME TO BURN
            STEP3_FIRST_BURN,         // First time burning
            STEP4_BURN,               // Burn
            STEP5_CALC_NEXT_STEP      // Determine Next Step
        };

        private LandingEquations landEq;
        private Step step;
        private Vector3d attitude;

        public DeorbitBurnGT(MechJebCore core) : base(core)
        {
            step = Step.STEP1_FIRST_TIME_TO_BURN;
            attitude = -Core.vessel.srf_velocity.normalized;
        }

        public override AutopilotStep Drive(FlightCtrlState s)
        {
            if ( step == Step.STEP4_BURN && Core.Attitude.attitudeAngleFromTarget() < 5 )
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
            double latErrorDeg;
            double lonErrorDeg;
            CelestialBody body = Core.vessel.mainBody;
            double peDesAlt = (body.atmosphere) ? body.atmosphereDepth / 2 : body.Radius * 0.0144873;
            double timeToBurn = 0;
            double tAlt = body.TerrainAltitude(Core.Target.targetLatitude, Core.Target.targetLongitude);
            double h = Core.vessel.terrainAltitude - tAlt;
            landEq.StartSamplePeriod(h, VesselState.dragUp, VesselState.mass, VesselState.speedVertical, VesselState.limitedMaxThrustAccel, VesselState.localg);

            switch (step)
            {
                case Step.STEP1_FIRST_TIME_TO_BURN:

                    timeToBurn = landEq.TimeRemainingToAdjustmentBurnGT(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude, out latErrorDeg, out lonErrorDeg);
                    if ((timeToBurn != 1) && (latErrorDeg < 5) && (lonErrorDeg < 5) )
                    {
                        step = Step.STEP2_TIME_TO_BURN;
                    }
                    else
                    {
                        return new CourseCorrectionGT(Core);
                    }
                    break;

                // Warp to burn location, except the last 10 seconds.
                case Step.STEP2_TIME_TO_BURN:
                    timeToBurn = landEq.TimeRemainingToDeorbitBurnGT(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude, peDesAlt);
                    bool warpReady = ((Vessel.angularVelocity.magnitude < 0.005f) && (Core.Attitude.attitudeAngleFromTarget() < 5));
                    if (warpReady && Core.Node.Autowarp && (timeToBurn > 1))
                    {
                        Core.Warp.WarpToUT(timeToBurn + VesselState.time);
                    }
                    else
                    {
                        Core.Warp.MinimumWarp();
                        if ( timeToBurn <= 1 )
                        {
                            step = Step.STEP3_FIRST_BURN;
                        }
                    }

                    break;

                // Set attitude for the first time.
                case Step.STEP3_FIRST_BURN:
                    attitude = landEq.CombinedDeorbitAttitudeGT(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude, peDesAlt,
                            out latErrorDeg, out lonErrorDeg);
                    break;

                case Step.STEP4_BURN:
                    attitude = landEq.CombinedDeorbitAttitudeGT(Core.vessel, Core.Target.targetLatitude, Core.Target.targetLongitude, peDesAlt,
                            out latErrorDeg, out lonErrorDeg);

                    if ( (latErrorDeg < 0.5) && (lonErrorDeg < 0.5) )
                    {
                        step = Step.STEP5_CALC_NEXT_STEP;
                    }
                    break;

                default:
                case Step.STEP5_CALC_NEXT_STEP:
                    Core.Thrust.ThrustOff();
                    if (body.atmosphere && body.atmosphereDepth > 20000)
                    {
                        return new AtmosphericBurn(Core);
                    }
                    else
                    {
                        return new LandingBurn(Core);
                    }
                    break;
            }

            return this;
        }
    }
}
