using System;
using MuMech;
using MuMech.Landing;
using UnityEngine;
using static alglib;
using static DishController;

namespace MuMech.LandingAutopilot
{
    /// <summary>
    /// MechJeb autopilot step that performs powered descent using ZEM/ZEV guidance
    /// with suicide-burn start logic, lateral velocity damping, and time-warp management.
    /// </summary>
    public class LandingBurn : AutopilotStep
    {
        private long debugCounter = 0;
        private bool shouldBurnStarted = false;
        private bool warpOn = false;                        // Tracks if warp is currently active
        private bool checkWarp = true;                      // Flag to allow warping when safe

        private const double ALT_MIN_WARP_CONSTANT = 5000.0; // Minimum altitude (m) below which warp is disallowed

        /// <summary>
        /// Constructor – receives MechJeb core reference.
        /// </summary>
        public LandingBurn(MechJebCore Core) : base(Core)
        {
            Core.Thrust.ThrustOff();
            Core.Attitude.attitudeTo(-VesselState.surfaceVelocity.normalized, AttitudeReference.INERTIAL, this);
        }

        /// <summary>
        /// Called every frame when autopilot is active.
        /// Runs guidance and control logic if not landed.
        /// </summary>
        public override AutopilotStep Drive(FlightCtrlState s)
        {
            if (!Vessel.LandedOrSplashed)
            {
                if ( true == UpdateGuidanceAndControl(s) )
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
        /// Fixed-update hook – handles time-warp decisions.
        /// </summary>
        public override AutopilotStep OnFixedUpdate()
        {
            if ( checkWarp == false )
            {
                return this;
            }

            // Compute pitch angle of current surface velocity relative to local up
            double pitchAngle = 90.0 - Vector3d.Angle(VesselState.surfaceVelocity, VesselState.up);
            double cosPitch = Math.Abs(Math.Cos(pitchAngle * UtilMath.Deg2Rad));
            double sinPitch = Math.Abs(Math.Sin(pitchAngle * UtilMath.Deg2Rad));

            // Horizontal stopping distance at max thrust
            double hStoppingDistance = Math.Pow(VesselState.speedSurfaceHorizontal, 2)/(2 * VesselState.limitedMaxThrustAccel * cosPitch);
            hStoppingDistance *= (1.25 + Core.Landing.HorizMargin / 100);

            // Vertical stopping distance (only if descending)
            double vStoppingDistance = (VesselState.speedVertical < 0) ? Math.Pow(VesselState.speedVertical, 2) /(2 * (VesselState.limitedMaxThrustAccel * sinPitch - VesselState.localg)):0;
            vStoppingDistance *= (1.25 + Core.Landing.VerticalMargin / 100);

            // Horizontal distance to target
            double hTargetError = Core.Landing.getHDistanceToTarget();

            // Conditions that force warp to stop (safety near ground/target)
            bool mustStopWarp =
                (hTargetError < 1.05 * hStoppingDistance) ||
                (VesselState.altitudeTrue < 1.05 * vStoppingDistance) ||
                ((VesselState.speedVertical < 0) &&
                 ((VesselState.altitudeTrue < ALT_MIN_WARP_CONSTANT) ||
                  ((MainBody.atmosphere==true) && (VesselState.altitudeASL < Vessel.mainBody.RealMaxAtmosphereAltitude()) &&
                   (VesselState.speedSurface > Core.Landing.atmosSafeSpeed))));

            if (mustStopWarp)
            {
                checkWarp = false;
            }

            // Perform warp only if safe and requested
            if (checkWarp && Core.Node.Autowarp && !Vessel.LandedOrSplashed)
            {
                double velocityGuess = Math.Max(Math.Abs(VesselState.speedVertical), VesselState.localg * 5);
                float warpRate = (float)Math.Min(
                    VesselState.altitudeASL / (6 * velocityGuess),
                    (Orbit.period / 6)
                );

                Core.Warp.WarpRegularAtRate(warpRate);
                warpOn = true;
            }
            else if (warpOn && !MuUtils.PhysicsRunning())
            {
                Core.Warp.MinimumWarp();
                warpOn = false;
            }

            return this;
        }

        /// <summary>
        /// Determines if thrusting should begin based on suicide-burn safety margin.
        /// Returns true if remaining altitude is within 90% of what is needed to stop.
        /// </summary>
        private bool ShouldStartBurn(double y, double vy, double hTargetError)
        {
            if (shouldBurnStarted) return true;

            // Compute pitch angle of current surface velocity relative to local up
            double pitchAngle = 90.0 - Vector3d.Angle(VesselState.surfaceVelocity, VesselState.up);
            double cosPitch = Math.Abs(Math.Cos(pitchAngle * UtilMath.Deg2Rad));
            double sinPitch = Math.Abs(Math.Sin(pitchAngle * UtilMath.Deg2Rad));

            // Horizontal stopping distance at max thrust
            double hStoppingDistance = (VesselState.speedSurfaceHorizontal* VesselState.speedSurfaceHorizontal)/(2 * VesselState.limitedMaxThrustAccel * cosPitch);
            hStoppingDistance *= (1.25 + Core.Landing.HorizMargin / 100);

            // Vertical stopping distance (only if descending)
            double vStoppingDistance = (VesselState.speedVertical < 0) ? (VesselState.speedVertical* VesselState.speedVertical) /(2 * (VesselState.limitedMaxThrustAccel * sinPitch - VesselState.localg)) : 0;
            vStoppingDistance *= (1.25 + Core.Landing.VerticalMargin / 100);

            // Effective stopping range - start burn if either vertical or horizontal distance is within stopping distance
            bool rc = (y <= vStoppingDistance) || (hTargetError <= hStoppingDistance);
            if (rc) shouldBurnStarted = true;

            if (--debugCounter <= 0)
            {
                Debug.Log($"[ShouldStartBurn] burn?={shouldBurnStarted}, y={y:F0} ystop={vStoppingDistance:F0}, x={hTargetError:F0} xstop={hStoppingDistance:F0}");
                debugCounter = 50;
            }
            return rc;
        }

        /// <summary>
        /// Main guidance loop: computes ZEM/ZEV commands, applies lateral damping,
        /// sets throttle, and directs attitude.
        /// </summary>
        private bool UpdateGuidanceAndControl(FlightCtrlState s)
        {
            bool recover = false;
            Vector3d up = (Vessel.CoM - Vessel.mainBody.position).normalized;
            Vector3d targetPos = Vessel.mainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0) - Vessel.mainBody.position;
            Vector3d downrangeVec, crossrangeVec, horizontalVec;
            Core.Landing.GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, VesselState.time, out downrangeVec, out crossrangeVec, out horizontalVec);

            double y = VesselState.altitudeTrue;
            double vy = Vector3d.Dot(VesselState.surfaceVelocity, up);
            double v_down = Math.Max(0, -vy);
            double hTargetError = horizontalVec.magnitude;
            double vx = Vector3d.Dot(VesselState.surfaceVelocity, horizontalVec.normalized);
            Vector3d hSurfaceVel = Vector3d.Exclude(VesselState.up, VesselState.surfaceVelocity);

            // Check if we should start the burn
            if (!ShouldStartBurn(y, vy, hTargetError))
            {
                Core.Attitude.attitudeTo(-VesselState.surfaceVelocity.normalized, AttitudeReference.INERTIAL, this);
                return recover;
            }

            double g_mag = VesselState.gravityForce.magnitude/ Core.Landing.debug5;
            double a_brake_net = VesselState.limitedMaxThrustAccel - g_mag;
            double r_mag = Math.Sqrt(y * y + hTargetError * hTargetError);
            double v_mag = VesselState.surfaceVelocity.magnitude;
            double disc = v_mag * v_mag + 2 * a_brake_net * r_mag;
            double t_go = Math.Max(0,(disc >= 0) ? (v_down + Math.Sqrt(disc)) / a_brake_net : 9999);
            double gain_scale_x = 1;
            double gain_scale_y = (t_go > 25) ? 0.4 + 0.6 * Mathf.Clamp((float)((30 - t_go) / 30), 0, 1) : 1.0;
            gain_scale_y = Mathf.Clamp((float)gain_scale_y, (float)Core.Landing.debug4, (float)Core.Landing.debug7);
            double zem_x = -hTargetError + vx * t_go;
            double zem_y = -y - vy * t_go + 0.5 * (-g_mag) * t_go * t_go;
            double zev_x = vx;
            double zev_y = -vy -g_mag * t_go;
            double a_cmd_x = gain_scale_x * ((6 / (t_go * t_go)) * zem_x + (4 / t_go) * zev_x);
            double a_cmd_y = gain_scale_y * Math.Max(0,(6 / (t_go * t_go)) * zem_y + (4 / t_go) * zev_y + g_mag + (-Core.Landing.TouchdownSpeed - vy)* Core.Landing.debug1 / t_go);

            // When close to ground, prioritize vertical control to avoid lateral oscillations that can cause instability and excessive horizontal acceleration demands.
            if (y < Core.Landing.debug3)
            {
                if (Math.Abs(a_cmd_x) >= Math.Abs(Core.Landing.debug2 * a_cmd_y))
                {
                    a_cmd_x = (y / Core.Landing.debug3) * Math.Sign(a_cmd_x) * Math.Abs(Core.Landing.debug2 * a_cmd_y);
                }

                // Need to reduce horizontal velocity but this algorithm can't handle it - defer to MoveToTarget 
                if (Math.Abs(vx) > Core.Landing.debug8 *Math.Abs(vy) )
                {
                    recover = true;
                }
                else
                {
                    // Boost vertical command to bleed off horizontal velocity when close to ground, since we won't have time to correct later and we want to avoid oscillations.
                    // This also helps ensure touchdown speed is achieved even if we have some unmodeled horizontal drag or other effects.
                    if ( y < 5 )
                    {
                        a_cmd_x *= 0.20; // reduce lateral command to avoid instability
                        a_cmd_y += 2 * ((-Core.Landing.TouchdownSpeed - vy) * Core.Landing.debug1 / t_go); 
                    }
                }
            }

            Vector3d desired_accel = -a_cmd_x * horizontalVec.normalized + Math.Max(0,a_cmd_y) * up.normalized;

            // Cancel Lateral velocity
            Vector3d parallelDir = horizontalVec.normalized;
            Vector3d velParallel = Vector3d.Project(hSurfaceVel, parallelDir);
            Vector3d velLateral = hSurfaceVel - velParallel;
            double lateralGain = Core.Landing.debug6;
            Vector3d lateralCorrection = -lateralGain * Math.Min(a_cmd_x, velLateral.magnitude) * velLateral.normalized;
            desired_accel += lateralCorrection;

            float desired_mag = (float)desired_accel.magnitude;
            if (Core.Attitude.attitudeAngleFromTarget() < 30)
            {
                Core.Thrust.RequestActiveThrottle(Mathf.Clamp01(desired_mag / (float)VesselState.limitedMaxThrustAccel));
            }
            else
            {
                Core.Thrust.RequestActiveThrottle(0);
            }
            Core.Attitude.attitudeTo(desired_accel.normalized, AttitudeReference.INERTIAL, this);
            if (--debugCounter <= 0)
            {
                Debug.Log($"[LandingBurn] t_go={t_go:F0}, x={hTargetError:F0}, y={y:F0}, vy={vy:F1}, a_cmd_y={a_cmd_y:F1} throttle ={desired_mag / VesselState.limitedMaxThrustAccel:F3}");
                debugCounter = 50;
            }
            return recover;
        }
    }
}
