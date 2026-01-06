using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MuMech.Landing;
using static alglib;

namespace MuMech.LandingAutopilot
{
    public class AtmosphericBurn : AutopilotStep
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

        public AtmosphericBurn(MechJebCore core) : base(core)
        {
            landEq = new LandingEquations();
            step = Step.STEP1_FIRST_TIME_TO_BURN;
            attitude = -Core.vessel.srf_velocity.normalized;
        }
    }
}
