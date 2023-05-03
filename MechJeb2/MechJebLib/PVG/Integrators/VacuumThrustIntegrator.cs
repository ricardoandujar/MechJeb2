/*
 * Copyright Lamont Granquist (lamont@scriptkiddie.org)
 * Dual licensed under the MIT (MIT-LICENSE) license
 * and GPLv2 (GPLv2-LICENSE) license or any later version.
 */

#nullable enable

using System;
using MechJebLib.Core.ODE;
using MechJebLib.Primitives;
using MechJebLib.Utils;

namespace MechJebLib.PVG.Integrators
{
    public class VacuumThrustIntegrator : IPVGIntegrator
    {
        private class VacuumThrustKernel
        {
            public static int N => ArrayWrapper.ARRAY_WRAPPER_LEN;

            public Phase Phase = null!;

            public void dydt(Vn yin, double x, Vn dyout)
            {
                Check.True(Phase.Normalized);

                using var y = ArrayWrapper.Rent(yin);
                using var dy = ArrayWrapper.Rent(dyout);

                double at = Phase.thrust / y.M;
                if (Phase.Infinite) at *= 2;

                double r2 = y.R.sqrMagnitude;
                double r = Math.Sqrt(r2);
                double r3 = r2 * r;
                double r5 = r3 * r2;

                V3 u = Phase.Unguided ? Phase.u0.normalized : y.PV.normalized;

                dy.R  = y.V;
                dy.V  = -y.R / r3 + at * u;
                dy.PV = -y.PR;
                dy.PR = y.PV / r3 - 3 / r5 * V3.Dot(y.R, y.PV) * y.R;
                dy.M  = Phase.thrust == 0 || Phase.Infinite ? 0 : -Phase.mdot;
                dy.Pm = 0;
                /*dy.Pm = _phase.FinalMassProblem && !_phase.Infinite
                    ? _phase.ThrustBar * V3.Dot(y.PV, u) / (y.M * y.M)
                    : 0; */
                dy.DV = at;
            }
        }

        private readonly VacuumThrustKernel _ode    = new VacuumThrustKernel();
        private readonly DormandPrince54      _solver = new DormandPrince54();

        public void Integrate(Vn y0, Vn yf, Phase phase, double t0, double tf)
        {
            _solver.ThrowOnMaxIter = true;
            _ode.Phase                     = phase;
            _solver.Solve(_ode.dydt, y0, yf, t0, tf);
        }

        public void Integrate(Vn y0, Vn yf, Phase phase, double t0, double tf, Solution solution)
        {
            _solver.ThrowOnMaxIter = true;
            _ode.Phase                     = phase;
            var interpolant = Hn.Get(VacuumThrustKernel.N);
            _solver.Solve(_ode.dydt, y0, yf, t0, tf, interpolant);
            solution.AddSegment(t0, tf, interpolant, phase);
        }
    }
}
