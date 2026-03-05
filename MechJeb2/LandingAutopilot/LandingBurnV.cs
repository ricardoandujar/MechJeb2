using System;
using MuMech.Landing;
using UnityEngine;
using static DishController;

namespace MuMech.LandingAutopilot
{
    /// <summary>
    /// MechJeb autopilot step for powered descent using vector-based ZEM/ZEV guidance.
    /// Controls thrust direction and throttle during final descent.
    /// </summary>
    public class LandingBurnV : AutopilotStep
    {
        // Used to print every 50 times
        private long debugCounter = 0;

        // Flag to indicate if burn has started (latches on once triggered)
        private bool shouldBurnStarted = false;

        // Flag: is time warp currently active?
        private bool warpOn = false;

        // Flag: is time warp allowed right now?
        private bool checkWarp = true;

        // Minimum altitude (meters) below which warp is disabled for safety
        private const double ALT_MIN_WARP_CONSTANT = 5000.0;

        private double targetOffset;     // Removed once reached.
        private double lateralGain = 0.4;// Lateral gain correction
        private double zemTerm;          // ZEM gain term (tunable)
        private double zevTerm;          // ZEV gain term (tunable)

        /// <summary>
        /// Constructor - receives MechJeb core reference.
        /// </summary>
        /// <param name="Core">MechJeb core instance</param>
        public LandingBurnV(MechJebCore Core) : base(Core)
        {
            targetOffset = Math.Max(1,Core.Landing.debug1); // Must be non-zero at start to prevent premature burn cutoff, but can be reduced to 0 for final touchdown precision
            zemTerm = Core.Landing.debug2;
            zevTerm = Core.Landing.debug3;
        }

        /// <summary>
        /// Main drive loop - called every frame.
        /// Runs guidance if vessel is not landed/splashed.
        /// </summary>
        /// <param name="s">Current flight control state</param>
        /// <returns>This step (remains active)</returns>
        public override AutopilotStep Drive(FlightCtrlState s)
        {
            if (!Vessel.LandedOrSplashed)
            {
                if (true == UpdateGuidanceAndControl(s))
                {
                    return new DecelerationBurn(Core);
                }
                return this;
            }
            else
            {
                Core.Landing.StopLanding();
                return null;
            }
        }

        /// <summary>
        /// Fixed-update hook - handles time-warp decisions.
        /// </summary>
        /// <returns>This step (remains active)</returns>
        public override AutopilotStep OnFixedUpdate()
        {
            if ( checkWarp == false )
            {
                return this;
            }

            // Angle between surface velocity and local up (90° = horizontal flight)
            double pitchAngle = 90.0 - Vector3d.Angle(VesselState.surfaceVelocity, VesselState.up);

            // Cosine and sine of pitch angle for thrust projection calculations
            double cosPitch = Math.Abs(Math.Cos(pitchAngle * UtilMath.Deg2Rad));
            double sinPitch = Math.Abs(Math.Sin(pitchAngle * UtilMath.Deg2Rad));

            // Horizontal distance required to stop at max thrust (projected)
            double hStoppingDistance = VesselState.speedSurfaceHorizontal * VesselState.speedSurfaceHorizontal /
                                       (2 * VesselState.limitedMaxThrustAccel * cosPitch);

            // Vertical distance required to stop if descending
            double vStoppingDistance = (VesselState.speedVertical < 0)
                ? VesselState.speedVertical * VesselState.speedVertical /
                  (2 * (VesselState.limitedMaxThrustAccel * sinPitch - VesselState.localg))
                : 0;

            // Horizontal distance to target landing site
            double hTargetError = Core.Landing.getHDistanceToTarget();

            // Ratio of horizontal error to altitude (used for warp safety)
            float ratio = (float)(hTargetError / VesselState.altitudeTrue);

            // Conditions that force warp to stop (safety near ground/target/unsafe speed)
            bool mustStopWarp =
                (hTargetError < 1.5 * hStoppingDistance) ||
                (VesselState.altitudeTrue < 1.5 * vStoppingDistance) ||
                (ratio < 1.5 * Core.Landing.maxRatio) ||
                ((VesselState.speedVertical < 0) &&
                 ((VesselState.altitudeTrue < ALT_MIN_WARP_CONSTANT) ||
                  ((VesselState.altitudeASL < Vessel.mainBody.RealMaxAtmosphereAltitude()) &&
                   (VesselState.speedSurface > Core.Landing.atmosSafeSpeed))));

            if (mustStopWarp) checkWarp = false;

            // Perform warp if allowed and requested
            if (checkWarp && Core.Node.Autowarp && !Vessel.LandedOrSplashed)
            {
                // Conservative velocity estimate for 10-second impact buffer
                double velocityGuess = Math.Max(Math.Abs(VesselState.speedVertical), VesselState.localg * 5);

                // Safe warp rate: limit based on altitude and orbital period
                float warpRate = (float)Math.Min(VesselState.altitudeASL / (6 * velocityGuess), (Orbit.period / 6));

                Core.Warp.WarpRegularAtRate(warpRate);
                warpOn = true;
            }
            else if (warpOn || !MuUtils.PhysicsRunning())
            {
                Core.Warp.MinimumWarp();
                warpOn = false;
            }

            return this;
        }

        /// <summary>
        /// Suicide-burn trigger using slant range and stopping capability.
        /// Returns true once triggered (latches on).
        /// Logs diagnostic info for debugging.
        /// </summary>
        private bool ShouldStartBurn(double y, double vy, Vector3d horizontalVec)
        {
            if (shouldBurnStarted) return true;
            bool rc = false;
            double hTargetError = horizontalVec.magnitude;
            double slantRange = Math.Sqrt(y * y + hTargetError * hTargetError);

            double v_down = Math.Max(0, -vy);
            double g_mag = VesselState.gravityForce.magnitude;
            double a_brake_net = VesselState.limitedMaxThrustAccel - g_mag;

            if (a_brake_net <= 0 || slantRange <= 0.1)
            {
                rc = true;
            }
            else
            {
                // Horizontal stopping distance
                double h_stop = VesselState.speedSurfaceHorizontal * VesselState.speedSurfaceHorizontal / (2 * VesselState.limitedMaxThrustAccel);

                // Vertical stopping distance
                double v_stop = (v_down * v_down) / (2 * a_brake_net);

                // Combined effective stopping range
                double stop_range = Math.Sqrt(h_stop * h_stop + v_stop * v_stop);

                rc = stop_range >= Core.Landing.debug4 * slantRange; // lower margin for shallow entry
            }

            if (rc) shouldBurnStarted = true;

            if (--debugCounter <= 0)
            {
                // Diagnostic log for debugging burn trigger
                Debug.Log($"[ShouldStartBurn] rc={rc}, y={y:F0}, vy={vy:F1}, hError={hTargetError:F0}, slant={slantRange:F0}");
                debugCounter = 50;
            }

            return rc;
        }

        /// <summary>
        /// Computes vector ZEM/ZEV guidance commands and sets throttle/attitude.
        /// Uses full vector math for position/velocity errors.
        /// Applies lateral damping to eliminate off-target velocity.
        /// </summary>
        private bool UpdateGuidanceAndControl(FlightCtrlState s)
        {
            bool recover = false;

            // Local up direction
            Vector3d up = Vessel.up;

            // Altitude above terrain (what you usually want for guidance "y")
            double alt = VesselState.altitudeTrue - targetOffset;

            // True radius from body center to bottom of vessel
            double r_bottom = Vessel.mainBody.Radius
                            + Vessel.mainBody.TerrainAltitude(Vessel.latitude, Vessel.longitude)
                            + alt;

            // Current position vector from body center
            Vector3d pos = (VesselState.CoM - Vessel.mainBody.position).normalized * (r_bottom);

            // Target position vector on surface
            Vector3d targetPos = Core.Target.GetPositionTargetPosition();

            // Vector from current position to target
            Vector3d posError = targetPos - pos;

            // Slant range (distance to target)
            double slantRange = posError.magnitude;

            if (slantRange <= 0.1)
            {
                Core.Thrust.RequestActiveThrottle(0);
                return recover;
            }

            // Surface-relative velocity vector
            Vector3d vel = VesselState.surfaceVelocity;

            // Vertical velocity component (positive up)
            double vy = Vector3d.Dot(vel, up);

            // Get range vectors for target direction
            Vector3d downrangeVec, crossrangeVec, horizontalVec;
            Core.Landing.GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, VesselState.time,
                out downrangeVec, out crossrangeVec, out horizontalVec);

            // Check if burn should be active
            if (!ShouldStartBurn(slantRange, vy, horizontalVec))
            {
                Core.Attitude.attitudeTo(-VesselState.surfaceVelocity.normalized, AttitudeReference.INERTIAL, this);
                return recover;
            }

            // Local gravity magnitude
            Vector3d g = VesselState.gravityForce;

            // Net upward braking acceleration available
            double a_brake_net = VesselState.limitedMaxThrustAccel - g.magnitude;

            // Downward velocity magnitude
            double v_down = Math.Max(0, -vy);

            // Discriminant for vertical suicide-burn time
            double disc = v_down * v_down + 2 * a_brake_net * slantRange;

            // Combined t_go (max of vertical and horizontal)
            double t_go = (disc >= 0) ? (v_down + Math.Sqrt(disc)) / a_brake_net : 9999;
            //double t_go = Math.Min(prev_t_go,(disc >= 0) ? (v_down + Math.Sqrt(disc)) / a_brake_net : 9999);
            //prev_t_go = t_go;

            // Correct ZEM and ZEV signs (desired - predicted)
            Vector3d predicted_pos = pos + vel * t_go + 0.5 * VesselState.gravityForce * t_go * t_go;
            Vector3d zem = posError - (predicted_pos - pos);   // = target - predicted_position

            Vector3d predicted_vel = vel + VesselState.gravityForce * t_go;
            Vector3d zev = -predicted_vel;                     // = 0 - predicted_velocity


            // Commanded acceleration (ZEM/ZEV formula)
            Vector3d a_cmd = (zemTerm / (t_go * t_go)) * zem + (zevTerm / t_go) * zev;

            // Apply terminal velocity bias (encourages stronger vertical braking near ground to hit target speed)
            // Final-phase boost (stronger control near ground)
            if ( (alt < Core.Landing.debug7) || (targetOffset==0) )
            {
                targetOffset = 0; // Remove target offset once reached to provide more safety margin for final touchdown
                lateralGain = 0.01; // Reduce lateral gain near ground to prevent overshoot from aggressive vertical braking
                zemTerm = Core.Landing.debug8;    // Reduce ZEM gain near ground to prevent overshoot from aggressive braking

                // Need to reduce horizontal velocity but this algorithm can't handle it - defer to MoveToTarget 
                if ((Math.Abs(Vessel.horizontalSrfSpeed) > 1.7 * Math.Abs(vy)) || Core.Landing.debug5==0)
                {
                    recover = true;
                }
                a_cmd = a_cmd - Vector3d.Project(a_cmd, up); // Remove vertical component to prevent interference with vertical braking

                // Terminal vertical velocity bias: add extra vertical braking if descending faster than target speed near ground
                a_cmd += (-Math.Abs(Core.Landing.TouchdownSpeed) - vy) * up * (Core.Landing.debug5 / t_go);
            }

            // Early-phase gain reduction (less aggressive when far away)
            double gain_scale = 0.4 + 0.6 * Mathf.Clamp((float)((60 - t_go) / 30), 0, 1);

            // Apply gain scale
            a_cmd *= gain_scale;

            // Desired acceleration vector
            Vector3d desired_accel = a_cmd;

            // Lateral damping: only horizontal plane perpendicular to target
            if ( lateralGain> 0 )
            {
                Vector3d parallelHorizontal = horizontalVec.normalized;
                Vector3d velParallelH = Vector3d.Project(VesselState.horizontalSurface, parallelHorizontal);
                Vector3d velLateralH = VesselState.horizontalSurface - velParallelH;
                desired_accel -= lateralGain * velLateralH;
            }

            // Clamp magnitude to max available thrust acceleration
            double desired_mag = desired_accel.magnitude;
            if (desired_mag > VesselState.limitedMaxThrustAccel + 0.01)
                desired_accel = desired_accel.normalized * VesselState.limitedMaxThrustAccel;

            // Set throttle (0–1)
            float throttle = (float)(desired_mag / VesselState.limitedMaxThrustAccel);
            if (Core.Attitude.attitudeAngleFromTarget() < 30)
            {
                Core.Thrust.RequestActiveThrottle(throttle);
            }
            else
            {
                Core.Thrust.RequestActiveThrottle(0);
            }

            // Set attitude to point thrust along desired_accel
            Core.Attitude.attitudeTo(desired_accel, AttitudeReference.INERTIAL, this);

            if (--debugCounter <= 0)
            {
                // Diagnostic log for debugging burn trigger
                Debug.Log($"[burn] t_go={t_go:F1} alt={alt:F1} vel={Vessel.horizontalSrfSpeed:F1},{vy:F1} t={throttle:F2} ");
                debugCounter = 1;
            }

            return recover;
        }
    }
}
