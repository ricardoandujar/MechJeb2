using System;
using MuMech.Landing;
using UnityEngine;
using static alglib;
using static DishController;

namespace MuMech.LandingAutopilot
{
    /// <summary>
    /// MechJeb autopilot step for powered descent using vector-based ZEM/ZEV guidance.
    /// Controls thrust direction and throttle during final descent.
    /// </summary>
    public class LandingBurn : AutopilotStep
    {
        // Used to print every 50 times
        private long debugCounter = 0;

        // Flag to indicate if burn has started (latches on once triggered)
        private bool shouldBurnStarted = false;
        private double densityAtBurnStart = 0;

        // Flag: is time warp currently active?
        private bool warpOn = false;
        private bool warpOnP = false;

        // Flag: is time warp allowed right now?
        private bool checkWarp = true;

        // Minimum altitude (meters) below which warp is disabled for safety
        private const double ALT_MIN_WARP_CONSTANT = 5000.0;

        // Downrange distance (meters) below which vertical ZEV gain is reduced for smoother final descent (used if ZevV<0 for conditional vertical braking boost)
        private const float DOWNRANGE_THRESHOLD_CONSTANT = 1500.0f;

        private double targetOffset;      // Removed once reached.
        //private double zevCounter;        // Used to enable zevV via a smoothing function.
        private float  minDownRange;      // Minimum downrange distance to target for full ZevV application (used if ZevV<0 for conditional vertical braking boost)
        private double lateralGain = 0.4; // Lateral gain correction
        private double prev_t_go = 0;     // Previous t_go used to force timep step changes for better guidance convergence (not strictly necessary)
        private double lastDesiredAcceleration = 0;

        /// <summary>
        /// Constructor - receives MechJeb core reference.
        /// </summary>
        /// <param name="Core">MechJeb core instance</param>
        public LandingBurn(MechJebCore Core) : base(Core)
        {
            targetOffset = Math.Max(1, Core.Landing.TargetOffset); // Must be non-zero at start to prevent premature burn cutoff, but can be reduced to 0 for final touchdown precision
            //zevCounter = 0;
            minDownRange = 1500.0f;
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
                    return new MoveToTarget2(Core);
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
            if (warpOn == false)
                return this;

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

            // Surface-relative velocity vector
            Vector3d vel = VesselState.surfaceVelocity;

            // Check if burn should be active
            ShouldStartBurn(ref posError, ref vel);

            return this;
        }

        private bool ShouldStartBurn(ref Vector3d posError, ref Vector3d vel)
        {
            if (shouldBurnStarted) return true;

            if (vel.magnitude < 1.0) return false;

            bool     rc;
            Vector3d up = Vessel.up;
            Vector3d retro = -vel.normalized;
            double sin_theta = Math.Abs(Vector3d.Dot(retro, -up));   // sin of angle from horizontal
            double cos_theta = Math.Sqrt(1.0 - sin_theta * sin_theta);
            double Vy = Vector3d.Dot(vel, -up);             // downward component
            if (Vy < 1.0)
            {
                Debug.Log($"[ShouldStartBurn2] sin_theta={sin_theta:F3}, cos_theta={cos_theta:F3}, Vy={Vy:F3}");
                rc = false;
            }
            else
            {
                double Vx = vel.magnitude * cos_theta;
                double g = VesselState.localg;
                double a = VesselState.limitedMaxThrustAccel;

                // Effective accelerations along retrograde direction
                double ay = a * sin_theta - g;                  // vertical (downward positive)
                double ax = a * cos_theta;                      // horizontal

                double h_vert  = (1.05 + Core.Landing.BurnMarginPerc / 100.0) * (Vy * Vy) / (2.0 * ay);
                double d_horiz = (1.00 + Core.Landing.BurnMarginPerc / 100.0) * (Vx * Vx) / (2.0 * ax);

                if (sin_theta < 0.25)
                {
                    h_vert = 0; // If nearly horizontal, ignore vertical stopping distance to prevent premature burn trigger
                }

                double current_h = -Vector3d.Dot(posError, up);

                // Horizontal distance only (projection onto horizontal plane)
                Vector3d posError_horizontal = posError - Vector3d.Dot(posError, up) * up;
                double current_d = posError_horizontal.magnitude;

                rc = (current_h <= (h_vert)) || (current_d <= (d_horiz));
                if (rc)
                {
                    shouldBurnStarted = true;
                    densityAtBurnStart = FlightGlobals.getAtmDensity(FlightGlobals.getStaticPressure(current_h / 5.0, MainBody), FlightGlobals.getExternalTemperature(current_h / 5.0, MainBody));
                }
                else
                {
                    rc = (current_h <= (1.35 * h_vert)) || (current_d <= (1.35 * d_horiz));

                    // Perform warp if allowed and requested
                    if (checkWarp && (rc == false) && Core.Node.Autowarp && !Vessel.LandedOrSplashed)
                    {
                        if (VesselState.altitudeASL < Vessel.mainBody.atmosphereDepth)
                        {
                            if ( warpOnP == false )
                            {
                                //too low to use any regular warp rates. Use physics warp at a max of x2:
                                Core.Warp.WarpPhysicsAtRate(2);
                                warpOnP = true;
                            }
                        }
                        else
                        {
                            // Conservative velocity estimate for 10-second impact buffer
                            double velocityGuess = Math.Max(Math.Abs(VesselState.speedVertical), VesselState.localg * 5);
                            // Safe warp rate: limit based on altitude and orbital period
                            float warpRate = (float)Math.Min(VesselState.altitudeTrue / (5 * velocityGuess), (Orbit.period / 6));

                            Core.Warp.WarpRegularAtRate(warpRate);
                            warpOnP = false;
                            warpOn = true;
                        }
                        h_vert *= 1.35;
                        d_horiz *= 1.35;
                        Debug.Log($"[ShouldStartBurn2:w] current_h={current_h:F0}, h_vert={h_vert:F0}, current_d={current_d:F0}, d_horiz={d_horiz:F0}");
                    }
                    else if (warpOn)
                    {
                        Core.Warp.MinimumWarp();
                        checkWarp = warpOn = warpOnP = false;
                    }
                    else
                    {
                        Debug.Log($"[ShouldStartBurn2] current_h={current_h:F0}, h_vert={h_vert:F0}, current_d={current_d:F0}, d_horiz={d_horiz:F0}");
                    }
                }
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

            if (warpOn == true)
                return recover;

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

            // Get range vectors for target direction
            Vector3d downrangeVec, crossrangeVec, horizontalVec;
            Core.Landing.GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, VesselState.time,
                out downrangeVec, out crossrangeVec, out horizontalVec);

            // Check if burn should be active
            ShouldStartBurn(ref posError, ref vel); 

            // Local gravity magnitude
            Vector3d g = VesselState.gravityForce;

            // Net upward braking acceleration available
            double a_brake_net = VesselState.limitedMaxThrustAccel - g.magnitude;

            // Vertical velocity component (positive up)
            double vy = Vector3d.Dot(vel, Vessel.up);

            // Downward velocity magnitude
            double v_down = Math.Max(0, -vy);

            // Discriminant for vertical suicide-burn time
            double disc = v_down * v_down + 2 * a_brake_net * slantRange;

            // Combined t_go (max of vertical and horizontal)
            // double t_go = Math.Max(1, (disc >= 0) ? (v_down + Math.Sqrt(disc)) / a_brake_net : 9999);
            // =IF(C3>0,IF($AC$5>0,(-E3 + Sqrt(M3)) / ($AB$2-G3),min(N2-$A$3/5,2*K3/(L3))),0)
            double t_go = (Core.Landing.ZevH<0) ? 2 * slantRange / vel.magnitude : (v_down + Math.Sqrt(disc)) / a_brake_net;
            if (prev_t_go > 0 )
            {
                t_go = Math.Max(1, Math.Min(prev_t_go - VesselState.deltaT / 5.0, t_go));
            }
            prev_t_go = t_go;

            // Correct ZEM and ZEV signs (desired - predicted)
            Vector3d predicted_pos = vel * t_go + 0.5 * VesselState.gravityForce * t_go * t_go;
            Vector3d zem = posError - (predicted_pos);   // = target - predicted_position   NOTE: pos was canceled out
            Vector3d zem_x = Vector3d.Exclude(Vessel.up, zem);
            Vector3d zev_x = -VesselState.horizontalSurface;
            Vector3d zem_y = Vector3d.Exclude(zem_x, zem);
            Vector3d zev_y = -(vy*Vessel.up + VesselState.gravityForce * t_go); // = 0 - predicted_velocity

            // If Vertical ZEV gain and vertical velocity is negative, only apply it when vertical velocity is below a specified vertical velocity threshold.
            double ZevV = Core.Landing.ZevV;
            float downrange = (float)downrangeVec.magnitude;
            if ( ZevV < 0 ) 
            {
                if (downrange <= DOWNRANGE_THRESHOLD_CONSTANT || minDownRange < DOWNRANGE_THRESHOLD_CONSTANT)
                {
                    minDownRange = Math.Min(minDownRange, downrange);
                    ZevV *= Mathf.Clamp((DOWNRANGE_THRESHOLD_CONSTANT - minDownRange) / (DOWNRANGE_THRESHOLD_CONSTANT - 10.0f), 0, 1);
                }
                else
                {
                    ZevV = 0;
                }
            }

            // Commanded acceleration (ZEM/ZEV formula)
            Vector3d a_cmd = (Core.Landing.ZemH / (t_go * t_go)) * zem_x + (Core.Landing.ZevH / t_go) * zev_x +
                             (Core.Landing.ZemV / (t_go * t_go)) * zem_y + (ZevV / t_go) * zev_y;

            // below this altitude, switch to recovery mode if not on target (prevents hard landings from guidance errors or bad tuning)
            // Or, if the downrange to altitude ratio is close to minRatio, switch to recovery mode - The final landing is based on tracking vertical and horizontal velocity as we zero in to target.
            if (lastDesiredAcceleration < (0.95*MoveToTarget2.LIMITED_MAX_THRUST_G_RATIO*VesselState.localg))
            {
                if (((downrange / alt) <= (1.1 * Core.Landing.minRatio)) || (alt < Core.Landing.LandWithMoverAlt) || (downrange < Core.Landing.LandWithMoverOffset))
                {
                    recover = true;
                }
            }

            // Apply terminal velocity bias (encourages stronger vertical braking near ground to hit target speed)
            // Final-phase boost (stronger control near ground)
            if ( (alt < 20.0) || (targetOffset==0) )
            {
                targetOffset = 0; // Remove target offset once reached to provide more safety margin for final touchdown
                lateralGain = 0.1;// 0.01; // Reduce lateral gain near ground to prevent overshoot from aggressive vertical braking

                a_cmd = a_cmd - Vector3d.Project(a_cmd, Vessel.up); // Remove vertical component to prevent interference with vertical braking

                // Terminal vertical velocity bias: add extra vertical braking if descending faster than target speed near ground
                a_cmd += (-Math.Abs(Core.Landing.TouchdownSpeed) - vy) * Vessel.up * (6.0 / t_go);
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

            if ( shouldBurnStarted == true )
            {
                // Set throttle (0–1)
                float throttle = (float)(desired_mag / VesselState.limitedMaxThrustAccel);

                if (Core.Attitude.attitudeAngleFromTarget() < 30)
                {
                    lastDesiredAcceleration = desired_mag;
                    Core.Thrust.RequestActiveThrottle(throttle);
                }
                else
                {
                    Core.Thrust.RequestActiveThrottle(0);
                }

                if (--debugCounter <= 0)
                {
                    // Diagnostic log for debugging burn trigger
                    Debug.Log($"[burn] t_go={t_go:F1} alt={alt:F1} vel={Vessel.horizontalSrfSpeed:F1},{vy:F1} t={throttle:F2} {densityAtBurnStart:F3} ");
                    debugCounter = 2;
                }
            }

            // Set attitude to point thrust along desired_accel
            Quaternion targetRot = Quaternion.LookRotation(desired_accel.normalized, Vessel.up)
                                 * Quaternion.Euler(0, 0, (float)Core.Landing.vesselAngle);
            Core.Attitude.attitudeTo(targetRot, AttitudeReference.INERTIAL, this);

            return recover;
        }
    }
}
