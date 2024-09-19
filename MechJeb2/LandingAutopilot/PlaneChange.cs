using KSP.Localization;
using UnityEngine;

// FIXME: use a maneuver node

namespace MuMech
{
    namespace Landing
    {
        public class PlaneChange : AutopilotStep
        {
            private int   _planeChangeTriggered;
            private double _planeChangeDVLeft;
            private double warpDiv;
            private bool lowGravMode;
            private float deltaVLeft;

            public PlaneChange(MechJebCore core) : base(core)
            {
                double g = MainBody.GeeASL * Core.Landing.EARTH_GRAVITY;

                if (g < Core.Landing.LOW_GRAVITY) // m/sec^2
                {
                    double baseGain = 0.196 / g;
                    warpDiv = 1.5;
                    lowGravMode = true;
                    deltaVLeft = 2.5f*(float)baseGain;
                }
                else
                {
                    warpDiv = 5.0;
                    lowGravMode = false;
                    deltaVLeft = 0.1f;
                }
                _planeChangeTriggered = 0;
            }

            //Could make this an iterative procedure for improved accuracy
            private Vector3d ComputePlaneChange()
            {
                Vector3d targetRadialVector =
                    Core.vessel.mainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0) - MainBody.position;
                Vector3d currentRadialVector = Core.VesselState.CoM - Core.vessel.mainBody.position;
                double angleToTarget = Vector3d.Angle(targetRadialVector, currentRadialVector);
                //this calculation seems like it might be be working right:
                double timeToTarget = Orbit.TimeOfTrueAnomaly(Core.vessel.orbit.trueAnomaly * UtilMath.Rad2Deg + angleToTarget, VesselState.time) -
                                      VesselState.time;
                double planetRotationAngle = 360 * timeToTarget / MainBody.rotationPeriod;
                var planetRotation = Quaternion.AngleAxis((float)planetRotationAngle, MainBody.angularVelocity);
                Vector3d targetRadialVectorOnFlyover = planetRotation * targetRadialVector;
                Vector3d horizontalToTarget = Vector3d.Exclude(VesselState.up, targetRadialVectorOnFlyover - currentRadialVector).normalized;
                return horizontalToTarget;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                double throttleDiv = 2.0;

                if ((_planeChangeTriggered==1) && Core.Attitude.attitudeAngleFromTarget() < 2)
                {
                    if ( lowGravMode )
                    {
                        throttleDiv = 10;
                    }
                    Core.Thrust.TargetThrottle = Mathf.Clamp01((float)(_planeChangeDVLeft / (throttleDiv * Core.VesselState.maxThrustAccel)));
                }
                else
                {
                    Core.Thrust.TargetThrottle = 0;
                }

                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                Vector3d targetRadialVector = MainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0) -
                                              MainBody.position;
                Vector3d currentRadialVector = VesselState.CoM - MainBody.position;
                double angleToTarget = Vector3d.Angle(targetRadialVector, currentRadialVector);
                bool approaching = Vector3d.Dot(targetRadialVector - currentRadialVector, VesselState.orbitalVelocity) > 0;
                Vector3d horizontalToTarget = ComputePlaneChange();
                Vector3d finalVelocity = Quaternion.FromToRotation(VesselState.horizontalOrbit, horizontalToTarget) * VesselState.orbitalVelocity;
                double Angle = Vector3d.Angle(finalVelocity, VesselState.orbitalVelocity);


                // For low gravity worlds to to burn if in the ballpark.
                if (lowGravMode )
                {
                    if ( (angleToTarget <= 30) && ((Angle < 30) || (Angle > (180 - 30))) )
                    {
                        if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp(true);
                        return new DecelerationBurn(Core);
                    }
                }

                if (_planeChangeTriggered==0 && approaching && angleToTarget > 80 && angleToTarget < 92)
                {
                    if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp(true);
                    _planeChangeTriggered = 1;
                }

                if (_planeChangeTriggered==1)
                {
                    Vector3d deltaV = finalVelocity - VesselState.orbitalVelocity;
                    //burn normal+ or normal- to avoid dropping the Pe:
                    var burnDir = Vector3d.Exclude(VesselState.up, Vector3d.Exclude(VesselState.orbitalVelocity, deltaV));
                    if ( Angle <= 90 )
                    {
                        _planeChangeDVLeft = UtilMath.Deg2Rad * Angle * VesselState.speedOrbitHorizontal;
                        Core.Attitude.attitudeTo(burnDir, AttitudeReference.INERTIAL, Core.Landing);
                    }
                    else
                    {
                        _planeChangeDVLeft = UtilMath.Deg2Rad * (180-Angle) * VesselState.speedOrbitHorizontal;
                        Core.Attitude.attitudeTo(-burnDir, AttitudeReference.INERTIAL, Core.Landing);
                    }
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status14",
                        _planeChangeDVLeft.ToString("F0")); //"Executing low orbit plane change of about " +  + " m/s"

                    if (_planeChangeDVLeft < deltaVLeft)
                    {
                        if ( lowGravMode )
                        {
                            // Once in range it will exit
                            _planeChangeTriggered = 2;
                        }
                        else
                        {
                            return new LowDeorbitBurn(Core);
                        }
                    }
                }
                else
                {
                    if (Core.Node.Autowarp) Core.Warp.WarpRegularAtRate((float)(Orbit.period / warpDiv));
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status15"); //"Moving to low orbit plane change burn point"
                }

                return this;
            }
        }
    }
}
