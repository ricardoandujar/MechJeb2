using System;
using KSP.Localization;
using UnityEngine;
using static SoftMasking.SoftMask;

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
            private float deltaVLeft;
            private double maxHorizon = 129600;

            public PlaneChange(MechJebCore core) : base(core)
            {
                if (Core.Landing.UseOnlyMoveToTarget == true)
                {
                    warpDiv = 0.5;
                    deltaVLeft = Mathf.Max(0.1f,2.5f*(float)(0.4 / Core.Landing.g));
                }
                else
                {
                    warpDiv = 5.0;
                    deltaVLeft = 0.1f;
                }
                _planeChangeTriggered = 0;
            }

            // Use Orbital Ground track to determine the plane change burn direction and deltaV required to reach the target.
            private Vector3d ComputePlaneChange(out Vector3d deltaV, out Vector3d burnDirection)
            {
                double distanceToTarget;
                double tUT;

                // Get Target position in BODY-FIXED frame
                Core.Landing.ClosestPointToSurfaceTarget(out Vector3d closestPos, out tUT, out distanceToTarget, out Vector3d surfaceVel, 400, maxHorizon);
                double deltaT = tUT - Planetarium.GetUniversalTime();
                Vector3d targetPos = Core.Target.GetPositionTargetPosition();
                Core.Landing.GetRangeVectorsToSurfaceTarget(Vessel, VesselState, targetPos, tUT, out Vector3d downrangeVec, out Vector3d crossrangeVec, out Vector3d horizontalVec);
                Vector3d targetF = (Core.Landing.RotateRelativePosition(targetPos, deltaT));
                Vector3d targetPointerF = (targetF - closestPos);
                Vector3d herror = Vector3d.Project(targetPointerF, VesselState.normalPlusSurface);
                deltaV = herror / deltaT;
                Vector3d finalVelocity = VesselState.orbitalVelocity + deltaV;
                burnDirection = (Vector3d.Angle(VesselState.normalPlus, herror) > 90) ? -VesselState.normalPlus : VesselState.normalPlus;
                Debug.Log("deltaT:" + deltaT.ToString("F3") + " herror:" + herror.magnitude.ToString("F3") + " deltaV:" + deltaV.magnitude.ToString("F3") + " finalVelocity:" + finalVelocity.magnitude.ToString("F3"));

                return finalVelocity;
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                double throttleDiv = Core.Landing.debug11/Core.Landing.g;

                if ((_planeChangeTriggered==1) && Core.Attitude.attitudeAngleFromTarget() < 2)
                {
                    Core.Thrust.RequestActiveThrottle(Mathf.Max(0.0001f,Mathf.Clamp01((float)(_planeChangeDVLeft / (throttleDiv * Core.VesselState.maxThrustAccel)))));
                }
                else
                {
                    Core.Thrust.ThrustOff();
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
                Vector3d finalVelocity = ComputePlaneChange(out Vector3d deltaV, out Vector3d burnDir);
                double Angle = Vector3d.Angle(finalVelocity, VesselState.orbitalVelocity);

                // Keep the vessel in orbit until it is time to do the plane change burn.
                if (_planeChangeTriggered==0)
                {
                    Debug.Log("waiting for Angle:" + angleToTarget.ToString("F3"));
                    if (approaching && (Math.Abs(angleToTarget - 90) < 20))
                        
                    {
                        if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp(true);
                        _planeChangeTriggered = 1;
                    }
                    else
                    {
                        if (Core.Node.Autowarp) Core.Warp.WarpRegularAtRate((Mathf.Max(10, (float)(Orbit.period / warpDiv))));
                        Status = Localizer.Format("#MechJeb_LandingGuidance_Status15"); //"Moving to low orbit plane change burn point"
                    }
                }

                if (_planeChangeTriggered==1)
                {
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
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status14", _planeChangeDVLeft.ToString("F0")); //"Executing low orbit plane change of about " +  + " m/s"

                    if (_planeChangeDVLeft < deltaVLeft)
                    {
                        Core.Thrust.ThrustOff();
                        return new LowDeorbitBurn(Core); // NOTE: LowDeorbitBurn will wait until configured burn angle to execute the burn, then it will proceed to the next step in the landing sequence.
                    }
                }

                return this;
            }
        }
    }
}
