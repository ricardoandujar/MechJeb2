using System;

namespace MuMech
{
    /// <summary>
    /// Minimum-snap final horizontal approach profile, vector form (Ricardo, Nov 20 2026).
    ///
    /// Why: the vessel can only re-orient at a limited rate, and horizontal acceleration follows its tilt, so
    /// the rate of change of jerk (snap) is what the attitude controller has to keep up with. Among all curves
    /// that move the craft from its measured state to the pad with position, velocity, acceleration and jerk
    /// all zero, the 7th-order polynomial in time minimizes integrated |snap|^2 for a given duration T.
    /// Choosing the shortest T that respects the limits makes it also the quickest such approach. One 2D
    /// plan covers along- and cross-track together, so nothing drifts sideways and there is no axis split.
    ///
    /// Math (local horizontal plane; r = vessel position relative to the pad, t = time since plan start):
    ///   r(t) = sum_{k=0..7} c_k t^k, c_k vectors
    ///   start (t = 0): [r, r', r'', r'''] = measured / seeded [r0, v0, a0, j0]
    ///   end   (t = T): [r, r', r'', r'''] = 0
    /// c0..c3 follow directly from the start state. With b_k = c_k T^k (k = 4..7) the end conditions are a
    /// constant 4x4 matrix (independent of T), inverted once (END_INV) and applied to every vector component.
    /// T is the smallest horizon (coarse scan then fine scan) for which, at CHECK_SAMPLES points on [0, T]:
    ///   no overshoot:  r(t) . r0hat &gt;= -NO_OVERSHOOT_TOL_M  (never passes the plane through the pad normal to r0)
    ///   no reversal:   v(t) . r0hat &lt;= +NO_REVERSE_TOL_MPS  (closing along r0 never turns into opening)
    ///   |a(t)| &lt;= MaxAccel, |j(t)| &lt;= Jerk, |snap(t)| &lt;= Snap (vector magnitudes).
    ///
    /// How it is used: MoveToTarget (TryMinSnapCommand) latches a plan inside
    /// Core.Landing.FinalApproachActivationDistance and each tick commands
    /// v_plan(t) + a_plan(t)*MinSnapLeadTime + MinSnapPositionGain*(r_plan(t) - r_meas); it re-plans at
    /// MinSnapReplanHz from the measured r, v and the plan's own a, j (TryFit with the shrinking horizon, or a
    /// fresh TryCreate search when that breaks a limit). After T, Eval returns zero (hold over the pad).
    /// Vectors are Unity world vectors; callers project onto the current horizontal plane.
    /// </summary>
    public sealed class MinSnapProfile
    {
        /// <summary>Polynomial coefficients c0..c7 (7th order: 4 start + 4 end conditions).</summary>
        private const int COEFF_COUNT = 8;

        /// <summary>Boundary derivatives fixed at each end: position, velocity, acceleration, jerk.</summary>
        private const int BOUNDARY_DERIVATIVES = 4;

        /// <summary>Highest derivative evaluated (snap = 4th derivative of position).</summary>
        public const int SNAP_DERIVATIVE = 4;

        /// <summary>Derivative indices for Eval.</summary>
        public const int POSITION = 0, VELOCITY = 1, ACCELERATION = 2, JERK = 3;

        /// <summary>Shortest horizon searched (s); shorter plans are not physically followable (sim lower bound).</summary>
        public const double MIN_HORIZON_S = 4.0;

        /// <summary>Longest horizon searched (s); beyond this the approach is treated as infeasible.</summary>
        public const double MAX_HORIZON_S = 90.0;

        /// <summary>Coarse search step (s) for the first feasible horizon.</summary>
        private const double COARSE_STEP_S = 0.5;

        /// <summary>Fine search step (s) inside the last coarse interval (sim resolution).</summary>
        private const double FINE_STEP_S = 0.05;

        /// <summary>Samples per candidate when checking limits (sim used 400; 300 keeps CPU modest).</summary>
        private const int CHECK_SAMPLES = 300;

        /// <summary>Tolerance (m/s) on the no-reversal check so float noise at the end does not count.</summary>
        private const double NO_REVERSE_TOL_MPS = 1e-6;

        /// <summary>Tolerance (m) on the no-overshoot check.</summary>
        private const double NO_OVERSHOOT_TOL_M = 1e-3;

        /// <summary>Smallest start distance (m) a plan is built for; closer than this the hold law is used.</summary>
        private const double MIN_PLAN_DISTANCE_M = 1e-3;

        /// <summary>Smallest pivot accepted by the one-time 4x4 inversion.</summary>
        private const double PIVOT_EPS = 1e-12;

        /// <summary>Falling factorials k!/(k-d)! for k = 0..7, d = 0..4 (derivative coefficients).</summary>
        private static readonly double[,] FALL = BuildFallingFactorials();

        /// <summary>Inverse of the end-condition matrix M[d, k-4] = FALL[k, d] (depends on nothing but the order).</summary>
        private static readonly double[,] END_INV = InvertEndMatrix();

        private readonly Vector3d[] _c = new Vector3d[COEFF_COUNT];

        /// <summary>Plan horizon (s): r, v, a and j reach zero at t = T.</summary>
        public double T { get; private set; }

        public Vector3d R0 { get; private set; }  // position relative to the pad at plan start (m)
        public Vector3d V0 { get; private set; }  // velocity at plan start (m/s)
        public Vector3d A0 { get; private set; }  // acceleration seed (m/s^2), clamped to MaxAccel
        public Vector3d J0 { get; private set; }  // jerk seed (m/s^3), clamped to Jerk
        public double PeakAccel { get; private set; } // max |a| over the plan (m/s^2)
        public double PeakJerk { get; private set; }  // max |j| over the plan (m/s^3)
        public double PeakSnap { get; private set; }  // max |snap| over the plan (m/s^4)

        /// <summary>Limits a plan must respect (magnitudes): MaxAccel m/s^2, Jerk m/s^3, Snap m/s^4.</summary>
        public struct Limits
        {
            public double MaxAccel;
            public double Jerk;
            public double Snap;
        }

        /// <summary>
        /// Searches the shortest feasible horizon for the start state. Returns null when no horizon up to
        /// MAX_HORIZON_S is feasible (e.g. opening from the pad, or too fast for the distance).
        /// </summary>
        public static MinSnapProfile TryCreate(Vector3d r0, Vector3d v0, Vector3d a0, Vector3d j0, Limits lim)
        {
            double firstOk = double.NaN;
            for (double t = MIN_HORIZON_S; t <= MAX_HORIZON_S; t += COARSE_STEP_S)
            {
                if (TryFit(r0, v0, a0, j0, t, lim) != null) { firstOk = t; break; }
            }
            if (double.IsNaN(firstOk))
                return null;
            // Refine downward inside the last coarse interval to the sim resolution.
            double lo = Math.Max(MIN_HORIZON_S, firstOk - COARSE_STEP_S + FINE_STEP_S);
            for (double t = lo; t < firstOk; t += FINE_STEP_S)
            {
                MinSnapProfile p = TryFit(r0, v0, a0, j0, t, lim);
                if (p != null) return p;
            }
            return TryFit(r0, v0, a0, j0, firstOk, lim);
        }

        /// <summary>
        /// Fits the polynomial for a fixed horizon (re-plans with the shrinking horizon). Returns null if the
        /// curve breaks any limit. Seed a / j are clamped to the limits first so one noisy measurement
        /// cannot make every plan infeasible.
        /// </summary>
        public static MinSnapProfile TryFit(Vector3d r0, Vector3d v0, Vector3d a0, Vector3d j0, double horizon, Limits lim)
        {
            if (r0.magnitude < MIN_PLAN_DISTANCE_M || horizon <= 0)
                return null;
            var p = new MinSnapProfile
            {
                T = horizon, R0 = r0, V0 = v0,
                A0 = ClampLength(a0, lim.MaxAccel), J0 = ClampLength(j0, lim.Jerk)
            };
            Vector3d[] start = { p.R0, p.V0, p.A0, p.J0 };
            for (int d = 0; d < BOUNDARY_DERIVATIVES; d++)
                p._c[d] = start[d] * (1.0 / FALL[d, d]);
            p.SolveEndCoefficients();
            return p.CheckLimits(lim) ? p : null;
        }

        /// <summary>d-th time derivative of r at time t into the plan (d = 0..4); zero at and after T.</summary>
        public Vector3d Eval(double t, int d)
        {
            if (t >= T)
                return Vector3d.zero;
            t = Math.Max(0.0, t);
            Vector3d sum = Vector3d.zero;
            double pow = 1.0;
            for (int k = d; k < COEFF_COUNT; k++)
            {
                sum += _c[k] * (FALL[k, d] * pow);
                pow *= t;
            }
            return sum;
        }

        // End conditions: sum_k FALL[k,d] c_k T^(k-d) = 0 for d = 0..3. Multiply by T^d and set b_k = c_k T^k:
        // sum_{k>=4} FALL[k,d] b_k = -sum_{k<4} FALL[k,d] c_k T^k, so b = END_INV * rhs for every component.
        private void SolveEndCoefficients()
        {
            var rhs = new Vector3d[BOUNDARY_DERIVATIVES];
            for (int d = 0; d < BOUNDARY_DERIVATIVES; d++)
            {
                Vector3d s = Vector3d.zero;
                for (int k = 0; k < BOUNDARY_DERIVATIVES; k++)
                    s += _c[k] * (FALL[k, d] * Math.Pow(T, k));
                rhs[d] = s * -1.0;
            }
            for (int i = 0; i < BOUNDARY_DERIVATIVES; i++)
            {
                Vector3d b = Vector3d.zero;
                for (int d = 0; d < BOUNDARY_DERIVATIVES; d++)
                    b += rhs[d] * END_INV[i, d];
                _c[BOUNDARY_DERIVATIVES + i] = b * (1.0 / Math.Pow(T, BOUNDARY_DERIVATIVES + i));
            }
        }

        private bool CheckLimits(Limits lim)
        {
            Vector3d r0Hat = R0 * (1.0 / R0.magnitude);
            double pa = 0.0, pj = 0.0, ps = 0.0;
            for (int i = 0; i < CHECK_SAMPLES; i++) // t = T itself is exactly zero by construction
            {
                double t = T * i / CHECK_SAMPLES;
                if (Vector3d.Dot(Eval(t, POSITION), r0Hat) < -NO_OVERSHOOT_TOL_M
                    || Vector3d.Dot(Eval(t, VELOCITY), r0Hat) > NO_REVERSE_TOL_MPS)
                    return false;
                pa = Math.Max(pa, Eval(t, ACCELERATION).magnitude);
                pj = Math.Max(pj, Eval(t, JERK).magnitude);
                ps = Math.Max(ps, Eval(t, SNAP_DERIVATIVE).magnitude);
                if (pa > lim.MaxAccel || pj > lim.Jerk || ps > lim.Snap)
                    return false;
            }
            PeakAccel = pa; PeakJerk = pj; PeakSnap = ps;
            return true;
        }

        /// <summary>v shortened to length max when longer (direction kept); max &lt;= 0 gives zero.</summary>
        public static Vector3d ClampLength(Vector3d v, double max)
        {
            if (max <= 0) return Vector3d.zero;
            double m = v.magnitude;
            return m > max ? v * (max / m) : v;
        }

        private static double[,] BuildFallingFactorials()
        {
            var f = new double[COEFF_COUNT, SNAP_DERIVATIVE + 1];
            for (int k = 0; k < COEFF_COUNT; k++)
            {
                double v = 1.0;
                for (int d = 0; d <= SNAP_DERIVATIVE; d++)
                {
                    f[k, d] = d > k ? 0.0 : v;
                    v *= k - d;
                }
            }
            return f;
        }

        // Gauss-Jordan inverse of M[d, i] = FALL[4 + i, d] (4x4, well conditioned: entries 1..840).
        private static double[,] InvertEndMatrix()
        {
            const int n = BOUNDARY_DERIVATIVES;
            var m = new double[n, 2 * n];
            for (int d = 0; d < n; d++)
            {
                for (int i = 0; i < n; i++)
                    m[d, i] = FALL[BOUNDARY_DERIVATIVES + i, d];
                m[d, n + d] = 1.0;
            }
            for (int col = 0; col < n; col++)
            {
                int piv = col;
                for (int r = col + 1; r < n; r++)
                    if (Math.Abs(m[r, col]) > Math.Abs(m[piv, col])) piv = r;
                if (Math.Abs(m[piv, col]) < PIVOT_EPS)
                    throw new InvalidOperationException("MinSnapProfile: singular end-condition matrix");
                for (int k = 0; k < 2 * n; k++) { double tmp = m[col, k]; m[col, k] = m[piv, k]; m[piv, k] = tmp; }
                double inv = 1.0 / m[col, col];
                for (int k = 0; k < 2 * n; k++) m[col, k] *= inv;
                for (int r = 0; r < n; r++)
                {
                    if (r == col) continue;
                    double f = m[r, col];
                    for (int k = 0; k < 2 * n; k++) m[r, k] -= f * m[col, k];
                }
            }
            var result = new double[n, n];
            for (int r = 0; r < n; r++)
                for (int k = 0; k < n; k++)
                    result[r, k] = m[r, n + k];
            return result;
        }
    }
}
