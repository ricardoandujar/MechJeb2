using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        /// <summary>
        /// Unified 3D velocity PID in the landing frame.
        /// Frame axes: X = along-track (horizontal toward target), Y = vertical (up), Z = lateral (cross-track).
        ///
        /// Per-axis Kp/Ki/Kd and gain schedules stay independently adjustable (the existing v* / h*
        /// debug knobs, plus lateral via debug10/debug12). Coupling is NOT "run three loops and
        /// hypot the throttles" — integral/D state lives in one object, anti-windup is on the
        /// saturated 3D accel, and the caller turns one acceleration vector into attitude+throttle.
        ///
        /// This replaces the unfinished VectorPID skeleton. The live AutopilotStep is still
        /// MoveToTarget; the previous axis-independent controller is MoveToTargetAxisIndependent.
        /// </summary>
        public class LandingFramePID
        {
            public const double IntegralMax = 500.0;

            /// <summary>Integral state: x=along-track, y=vertical, z=lateral.</summary>
            public Vector3d Integral;

            /// <summary>Filtered d(measVel)/dt in the landing frame (D-on-measurement, not D-on-error).</summary>
            public Vector3d FilteredDeriv;

            // Previous samples in WORLD space so D can be re-projected into today's landing
            // frame. Storing frame-axis meas made soft (and hard) frame rotations look like
            // huge velocity changes and re-coupled H/L through the D term.
            private Vector3d _prevHSurface;
            private double   _prevVertSpeed;
            private bool     _hasPrev;
            private Vector3d _lastError;

            public void Reset()
            {
                Integral       = Vector3d.zero;
                FilteredDeriv  = Vector3d.zero;
                _prevHSurface  = Vector3d.zero;
                _prevVertSpeed = 0;
                _hasPrev       = false;
                _lastError     = Vector3d.zero;
            }

            /// <summary>
            /// Drop along-track / lateral I and D after a hard landing-frame swing.
            /// Keeps vertical I (up is stable) and clears the world sample so the next
            /// Accumulate does not invent a D spike from the old frame.
            /// </summary>
            public void InvalidateHorizontal()
            {
                Vector3d i = Integral;
                i.x = 0;
                i.z = 0;
                Integral = i;

                Vector3d d = FilteredDeriv;
                d.x = 0;
                d.z = 0;
                FilteredDeriv = d;

                _hasPrev = false;
            }

            /// <summary>
            /// Advance I and filtered measurement derivative.
            /// Vertical I matches the working axis PID (accumulates error with no dt).
            /// Along-track and lateral I use error*dt like the working horizontal PID.
            /// D is computed by projecting the previous and current world velocities into
            /// the CURRENT landing frame, so a rotating hDir/latDir does not fake dV/dt.
            /// </summary>
            public void Accumulate(Vector3d error, Vector3d hSurface, double vertSpeed,
                                   Vector3d hDir, Vector3d latDir, double dt, double alpha,
                                   double resetAbsH, double resetAbsV)
            {
                if (dt < 1e-6)
                    dt = 0.02;

                // Vertical (working pidVelocity_v): I += error, no dt.
                Integral.y += error.y;
                Integral.y =  Clamp(Integral.y, -IntegralMax, IntegralMax);
                if (Math.Abs(error.y) > resetAbsV)
                    Integral.y = 0;

                // Along-track (working pidVelocity_h): I += error*dt.
                Integral.x += error.x * dt;
                Integral.x =  Clamp(Integral.x, -IntegralMax, IntegralMax);
                if (Math.Abs(error.x) > resetAbsH)
                    Integral.x = 0;

                // Lateral: same formulation as along-track so it shares a time scale with H.
                Integral.z += error.z * dt;
                Integral.z =  Clamp(Integral.z, -IntegralMax, IntegralMax);
                if (Math.Abs(error.z) > resetAbsH)
                    Integral.z = 0;

                Vector3d meas = new Vector3d(
                    Vector3d.Dot(hSurface, hDir),
                    vertSpeed,
                    Vector3d.Dot(hSurface, latDir));

                if (_hasPrev)
                {
                    Vector3d prevMeas = new Vector3d(
                        Vector3d.Dot(_prevHSurface, hDir),
                        _prevVertSpeed,
                        Vector3d.Dot(_prevHSurface, latDir));
                    Vector3d rawDv = (meas - prevMeas) / dt;
                    FilteredDeriv = alpha * FilteredDeriv + (1.0 - alpha) * rawDv;
                }
                else
                {
                    FilteredDeriv = Vector3d.zero;
                    _hasPrev = true;
                }

                _prevHSurface  = hSurface;
                _prevVertSpeed = vertSpeed;
                _lastError     = error;
            }

            /// <summary>
            /// Coupled anti-windup: if the 3D accel was reduced (cone, saturation, or projection),
            /// bleed I on every axis rather than letting one axis wind up while another is clipped.
            /// </summary>
            public void AntiWindup(Vector3d rawAccel, Vector3d appliedAccel)
            {
                double rawMag      = rawAccel.magnitude;
                double appliedMag  = appliedAccel.magnitude;
                if (rawMag < 1e-9)
                    return;

                if (appliedMag + 0.05 < rawMag)
                {
                    // Scale I with the same factor we lost on the vector. Shared bleed is the coupling:
                    // a saturated horizontal command also eases the vertical integral, and vice versa.
                    double scale = Clamp(appliedMag / rawMag, 0.5, 0.98);
                    Integral *= scale;
                }
            }

            private static double Clamp(double v, double lo, double hi)
            {
                if (v < lo) return lo;
                if (v > hi) return hi;
                return v;
            }
        }
    }
}
