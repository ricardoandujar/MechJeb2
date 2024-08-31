#define SIMPLE

using System;
using KSP.Localization;
using ModuleWheels;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class FinalDescent : AutopilotStep
        {
            private const float FINAL_ALT_THRESHOLD_CONSTANT = 4000.0f;//400.0F;
            private const float LANDING_GEAR_ALT_CONSTANT = 200.0F;
            private const float MIN_HORIZONTAL_ALT_THRESHOLD_CONSTANT = FINAL_ALT_THRESHOLD_CONSTANT;
            private const float MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT = 7000.0F;
            private const float ABOVE_FINAL_ALT_SPEED_CONSTANT = -40.0F;
            private const float MIN_WARP_ALT_THRESHOLD_CONSTANT = MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT;
            private const float WARP_EDGE_THRESHOLD_CONSTANT = 1.02F;
            private const float ANGLE_MIN_SPEED_RATIO_CONSTANT = 1.3F;
            private const float ANGLE_MIN_ANGLE_RATIO_CONSTANT = 0.0F;
            private const float ANGLE_MAX_ANGLE_RATIO_CONSTANT = 0.6F;
            private const float ANGLE_ADJUST_I_CONSTANT = 0.00001f;
            private const float ANGLE_ADJUST_P_CONSTANT = -0.01f;
            private const float FINAL_SPEED_FACTOR_CONSTANT = 0.8F;
            private const float MAX_WARP_ANGLE_CONSTANT = 45.0F;
            private bool        _deployedGears;
            private float       magnitude = 0;
            private IDescentSpeedPolicy _aggressivePolicy;
            private bool        warp = false;
            private bool        useRealAlt = false;

            public FinalDescent(MechJebCore core, float _TargetThrottle) : base(core)
            {
                _deployedGears = false;
                Core.Thrust.TargetThrottle = _TargetThrottle;
            }

            private double GetMinAlt()
            {
                double minalt;
                if ( useRealAlt == false )
                {
                    minalt = Math.Min(VesselState.altitudeBottom, Math.Min(VesselState.altitudeASL, VesselState.altitudeTrue));
                    if ( minalt < 5 )
                    {
                        useRealAlt = true;
                    }
                }
                else
                {
                    minalt = VesselState.altitudeTrue;
                }
                return minalt;
            }

            private double GetMaxSpeed(bool updatePolicy, double minalt )
            {
                double maxSpeed;

                if (minalt > FINAL_ALT_THRESHOLD_CONSTANT)
                {
                    if (Core.Landing.FlySafe)
                    {
                        maxSpeed = Math.Sqrt((VesselState.limitedMaxThrustAccel - VesselState.localg) * 2 * minalt);
                    }
                    else
                    {
                        if (updatePolicy || _aggressivePolicy == null)
                        {
                            Vector3d estimatedLandingPosition = VesselState.CoM + VesselState.surfaceVelocity.sqrMagnitude / (2 * VesselState.limitedMaxThrustAccel) * VesselState.surfaceVelocity.normalized;
                            double terrainRadius = MainBody.Radius + MainBody.TerrainAltitude(estimatedLandingPosition);
                            _aggressivePolicy = new GravityTurnDescentSpeedPolicy(terrainRadius, MainBody.GeeASL * 9.81, VesselState.limitedMaxThrustAccel); // this constant policy creation is wastefull...
                        }
                        maxSpeed = _aggressivePolicy.MaxAllowedSpeed(VesselState.CoM - MainBody.position, VesselState.surfaceVelocity);
                        maxSpeed = Math.Max(maxSpeed, Math.Sqrt((VesselState.limitedMaxThrustAccel - VesselState.localg) * 2 * minalt));
                    }
                }
                else
                {
                    maxSpeed = Mathf.Lerp(0,
                        (float)Math.Sqrt((VesselState.limitedMaxThrustAccel - VesselState.localg) * 2 * FINAL_ALT_THRESHOLD_CONSTANT) * FINAL_SPEED_FACTOR_CONSTANT, (float)VesselState.altitudeTrue / FINAL_ALT_THRESHOLD_CONSTANT);
                }

                return maxSpeed;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                double minalt = GetMinAlt();
                double maxSpeed = GetMaxSpeed(false, minalt);

                if (!Core.Node.Autowarp || (minalt < MIN_WARP_ALT_THRESHOLD_CONSTANT) || (maxSpeed < VesselState.speedSurface))
                {
                    if ( warp == true )
                    {
                        warp = false;
                        Core.Warp.MinimumWarp(true);
                    }
                    return this;
                }
                double maxVel = WARP_EDGE_THRESHOLD_CONSTANT * maxSpeed;

                double diffPercent = (maxVel / VesselState.speedSurface - 1) * 100;

                if ( diffPercent > 0 ) //&& Vector3d.Angle(VesselState.forward, -VesselState.surfaceVelocity) < MAX_WARP_ANGLE_CONSTANT)
                {
                    warp = true;
                    Core.Warp.WarpRegularAtRate((float)(diffPercent * diffPercent * diffPercent));
                }
                else
                {
                    if (warp == true)
                    {
                        warp = false;
                        Core.Warp.MinimumWarp(true);
                    }
                }

                return this;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (Vessel.LandedOrSplashed)
                {
                    Core.Landing.StopLanding();
                    return null;
                }

                // TODO perhaps we should pop the parachutes at this point, or at least consider it depending on the altitude.

                double minalt = GetMinAlt();

                // Consider lowering the langing gear
                if (!_deployedGears)
                {
                    if (VesselState.altitudeTrue < LANDING_GEAR_ALT_CONSTANT )
                    {
                        Vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
                        _deployedGears = true;
                    }
                }

                // if we have TWR < 1, just try as hard as we can to decelerate:
                // (we need this special case because otherwise the calculations spit out NaN's)
                // Also used to kill horizontal velocity before we do final approach. 
                if ( (VesselState.limitedMaxThrustAccel < VesselState.localg ) ||
                     ((VesselState.speedSurfaceHorizontal > 5) && (minalt > MIN_HORIZONTAL_ALT_THRESHOLD_CONSTANT) && (minalt < MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT)) )
                {
                    Core.Thrust.Tmode       = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
                    Core.Thrust.TransKillH  = true;
                    Core.Thrust.TransSpdAct = 5.0F;
                }

                // Point Up and set speed to -2 meters/sec
                // This ensures that surface retrograde is never activated when close to zero or positive vertical
                // speed.
                else if ( (VesselState.speedVertical >= -0.1) && (minalt < FINAL_ALT_THRESHOLD_CONSTANT) )
                {
                    // if we have positive vertical velocity, point up and don't thrust:
                    Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, null);
                    Core.Thrust.Tmode = MechJebModuleThrustController.TMode.DIRECT;
                    Core.Thrust.TransSpdAct = -2;
                }

                // Perform Surface Retrograde using calculated speed
                // if we're falling at a significant angle from vertical, our vertical speed might be
                // quite small but we might still need to decelerate. Control the total speed instead
                // by thrusting directly retrograde
                else
                {
                    // Get Max Speed - based on altitude + FlySafe setting
                    Core.Thrust.TransSpdAct = -(float)GetMaxSpeed(true, minalt);

                    // take into account desired landing speed:
                    Core.Thrust.TransSpdAct = (float)Math.Min(-Core.Landing.TouchdownSpeed, Core.Thrust.TransSpdAct);

                    // If the surfaceSpeed is significantly larger then the targeted speed
                    // Attempt to minimize altitude loss to allow more time to burn the horizontal speed, othwerwise
                    // the vessel will crash into terrain.
                    // This is is definitely needed when starting from a low orbit with a low TWR.
                    Vector3d point;
                    if ((minalt > MAX_HORIZONTAL_ALT_THRESHOLD_CONSTANT) && VesselState.speedSurface > Math.Abs(Core.Thrust.TransSpdAct)*ANGLE_MIN_SPEED_RATIO_CONSTANT)
                    {
                        magnitude -= ANGLE_ADJUST_I_CONSTANT * ((float)VesselState.speedVertical);
                        magnitude = Mathf.Min(ANGLE_MAX_ANGLE_RATIO_CONSTANT, Mathf.Max(ANGLE_MIN_ANGLE_RATIO_CONSTANT, magnitude));
                        point = Vector3d.Lerp(Vector3d.back, Vector3d.up, magnitude+(ANGLE_ADJUST_P_CONSTANT * ((float)VesselState.speedVertical)) );
                        Core.Thrust.TransKillH = false;
                    }

                    // No correction needed so use retrograde unchanged
                    else
                    {
                        magnitude = 0.0f;
                        point = Vector3d.back;
                        if ((VesselState.speedSurfaceHorizontal < 5) && (minalt < FINAL_ALT_THRESHOLD_CONSTANT))
                        {
                            Core.Thrust.TransKillH = true;
                        }
                        else
                        {
                            Core.Thrust.TransKillH = false;
                        }
                    }

                    Core.Attitude.attitudeTo(point, AttitudeReference.SURFACE_VELOCITY, null);
                    Core.Thrust.Tmode       =  MechJebModuleThrustController.TMode.KEEP_SURFACE;
                    Core.Thrust.TransSpdAct *= -1;
                }

                // Display target speed rather than altitude since that is already available elsewhere - this
                // allows for monitoring progress.
                Status = Localizer.Format("#MechJeb_LandingGuidance_Status6",
                    Core.Thrust.TransSpdAct >= double.MaxValue ? "∞" : Core.Thrust.TransSpdAct.ToString("F1")); //"Braking: target speed = " +  + " m/s"

                return this;
            }
        }
    }
}
