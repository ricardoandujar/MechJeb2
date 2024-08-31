using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class KillHorizontalVelocity : AutopilotStep
        {
            public KillHorizontalVelocity(MechJebCore core, float _TargetThrottle) : base(core)
            {
                Core.Thrust.TargetThrottle = _TargetThrottle;
            }

            // TODO I think that this function could be better rewritten to much more agressively kill the horizontal velocity. At present on low gravity bodies such as Bop, the craft will hover and slowly drift sideways, loosing the prescion of the landing. 
            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // If Horizontal speed is already below threshold then move on to Final Descent.
                if (VesselState.speedSurfaceHorizontal < 5)
                {
                    return new FinalDescent(Core, Core.Thrust.TargetThrottle);
                }

                if (!Core.Landing.PredictionReady)
                    return this;

                Vector3d horizontalPointingDirection = Vector3d.Exclude(VesselState.up, VesselState.forward).normalized;

                // If Vertical speed is positive then time to move on to final descent - it will kill any residual horizontal speed.
                if (Vector3d.Dot(horizontalPointingDirection, VesselState.surfaceVelocity) > 0)
                {
                    Core.Thrust.TargetThrottle = 0;
                    Core.Attitude.attitudeTo(Vector3.up, AttitudeReference.SURFACE_NORTH, Core.Landing);
                    return new FinalDescent(Core, Core.Thrust.TargetThrottle);
                }

                //control thrust to control vertical speed:
                const double DESIRED_SPEED = 0.0F; //hover until horizontal velocity is killed
                double controlledSpeed = Vector3d.Dot(VesselState.surfaceVelocity, VesselState.up);
                double speedError = DESIRED_SPEED - controlledSpeed;
                const double SPEED_CORRECTION_TIME_CONSTANT = 1.0F;
                const double ANGLE_FACTOR_CONSTANT = 0.3F;
                double desiredAccel = speedError / SPEED_CORRECTION_TIME_CONSTANT;
                double minAccel = -VesselState.localg;
                double maxAccel = -VesselState.localg + Vector3d.Dot(VesselState.forward, VesselState.up) * VesselState.limitedMaxThrustAccel;
                if (maxAccel - minAccel > 0)
                {
                    Core.Thrust.TargetThrottle = Mathf.Clamp((float)((desiredAccel - minAccel) / (maxAccel - minAccel)), 0.0F, 1.0F);
                }
                else
                {
                    Core.Thrust.TargetThrottle = 0;
                }

                //angle up and slightly away from vertical:
                Vector3d desiredThrustVector = (VesselState.up + ANGLE_FACTOR_CONSTANT * horizontalPointingDirection).normalized;

                Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, Core.Landing);

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status10"); //"Killing horizontal velocity before final descent"

                return this;
            }
        }
    }
}
