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
        private LandingEquations landEq;
        private Vector3d attitude;

        public AtmosphericBurn(MechJebCore core) : base(core)
        {
            landEq = new LandingEquations();
            attitude = -Core.vessel.srf_velocity.normalized;
        }
    }
}
