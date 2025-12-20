using System;
using KSP.Localization;
using UnityEngine;

// FIXME: use a maneuver node

namespace MuMech
{
    namespace Landing
    {
        public class LowDeorbitBurn : AutopilotStep
        {
            private bool   _deorbitBurnTriggered;
            private double _lowDeorbitBurnMaxThrottle;
            private bool   _lowDeorbitEndOnLandingSiteNearer;

            private const double LOW_DEORBIT_BURN_TRIGGER_FACTOR = 2.15; // 2.15<=2.05 <= 2

            public LowDeorbitBurn(MechJebCore core) : base(core)
            {
                _deorbitBurnTriggered = false;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 5)
                {
                    Core.Thrust.RequestActiveThrottle(Mathf.Clamp01((float)_lowDeorbitBurnMaxThrottle), allowZero: true);
                }
                else if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 10 && Core.Thrust.LimiterMinThrottle)
                {
                    Core.Thrust.RequestActiveThrottle(0.0f);
                }
                else
                {
                    Core.Thrust.ThrustOff();
                }

                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                //Decide when we will start the deorbit burn:
                double pitchAngle = 30;//90.0 - Vector3d.Angle(VesselState.surfaceVelocity, VesselState.up);
                double stoppingDistance = Math.Pow(VesselState.speedSurfaceHorizontal, 2) / (2 * VesselState.limitedMaxThrustAccel * Math.Abs(Math.Cos(pitchAngle * UtilMath.Deg2Rad)));
                double triggerDistance = LOW_DEORBIT_BURN_TRIGGER_FACTOR * stoppingDistance;
                double heightAboveTarget = VesselState.altitudeASL - Core.Landing.DecelerationEndAltitude();
                if (triggerDistance < heightAboveTarget) triggerDistance = heightAboveTarget;

                //See if it's time to start the deorbit burn:
                double rangeToTarget = Core.Landing.getHDistanceToTarget();

                if (!_deorbitBurnTriggered && (rangeToTarget < triggerDistance) ) _deorbitBurnTriggered = true;

                Status = Localizer.Format(_deorbitBurnTriggered
                    ? "#MechJeb_LandingGuidance_Status11" //"Executing low deorbit burn"
                    :                                     
                    "#MechJeb_LandingGuidance_Status12"); //"Moving to low deorbit burn point"

                //Warp toward deorbit burn if it hasn't been triggerd yet:
                if (!_deorbitBurnTriggered && Core.Node.Autowarp && (rangeToTarget > 1.2 * triggerDistance))
                {
                    if ((Vessel.angularVelocity.magnitude < 0.005f) && (Core.Attitude.attitudeAngleFromTarget() < 1)) 
                    {
                        Core.Warp.WarpRegularAtRate((float)(Orbit.period / 5));
                    }
                }
                else if (!MuUtils.PhysicsRunning())
                {
                    Core.Warp.MinimumWarp(true);
                }

                //By default, thrust straight back at max throttle
                Vector3d thrustDirection = -VesselState.surfaceVelocity.normalized;
                _lowDeorbitBurnMaxThrottle = 1;

                //If we are burning, we watch the predicted landing site and switch to the braking
                //burn when the predicted landing site crosses the target. We also use the predictions
                //to steer the predicted landing site toward the target
                if ( (_deorbitBurnTriggered==true)  && (Core.Landing.PredictionReady == true) )
                {
                    //angle slightly left or right to fix any cross-range error in the predicted landing site:
                    Vector3d horizontalToLandingSite = Vector3d.Exclude(VesselState.up, Core.Landing.LandingSite - VesselState.CoM).normalized;
                    Vector3d horizontalToTarget =
                        Vector3d.Exclude(VesselState.up, Core.Target.GetPositionTargetPosition() - VesselState.CoM).normalized;
                    const double ANGLE_GAIN = 4;
                    Vector3d angleCorrection = ANGLE_GAIN * (horizontalToTarget - horizontalToLandingSite);
                    if (angleCorrection.magnitude > 0.1) angleCorrection *= 0.1 / angleCorrection.magnitude;
                    thrustDirection = (thrustDirection + angleCorrection).normalized;

                    double rangeToLandingSite = Vector3d.Exclude(VesselState.up, Core.Landing.LandingSite - VesselState.CoM).magnitude;
                    double maxAllowedSpeed = Core.Landing.MaxAllowedSpeed();

                    _lowDeorbitEndOnLandingSiteNearer = rangeToLandingSite > (rangeToTarget + Core.Landing.POST_TARGET_THRESHOLD); // Target ahead of landing site so deceleration will bring it closer.

                    _lowDeorbitBurnMaxThrottle = 1;

                    if (Orbit.PeA < 0)
                    {
                        if (_lowDeorbitEndOnLandingSiteNearer == true)
                        {
                            double maxAllowedSpeedAfterDt = Core.Landing.MaxAllowedSpeedAfterDt(VesselState.deltaT);
                            double speedAfterDt = VesselState.speedSurface +
                                                  VesselState.deltaT * Vector3d.Dot(VesselState.gravityForce, VesselState.surfaceVelocity.normalized);
                            double throttleToMaintainLandingSite;
                            if (VesselState.speedSurface < maxAllowedSpeed) throttleToMaintainLandingSite = 0;
                            else
                                throttleToMaintainLandingSite =
                                    (speedAfterDt - maxAllowedSpeedAfterDt) / (VesselState.deltaT * VesselState.maxThrustAccel);

                            _lowDeorbitBurnMaxThrottle = throttleToMaintainLandingSite + 1 * (rangeToLandingSite / (rangeToTarget + Core.Landing.POST_TARGET_THRESHOLD) - 1) + 0.2;
                        }
                        else
                        {
                            // We are ready for deceleration burn
                            Core.Thrust.TargetThrottle = 0;
                            return new DecelerationBurn(Core);
                        }
                    }
                }

                Core.Attitude.attitudeTo(thrustDirection, AttitudeReference.INERTIAL, Core.Landing);

                return this;
            }
        }
    }
}
