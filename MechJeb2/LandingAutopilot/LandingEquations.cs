using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KSP.Localization;
using ModuleWheels;
using MuMech;
using MuMech.Landing;
using SaveUpgradePipeline;
using UnityEngine;
using static alglib;


// Latitude: Horizontal lines parallel to equator, measuring north-south distance (0° at equator to 90° at poles, N or S).
// Longitude: Vertical lines from pole to pole, measuring east-west distance (0° at Prime Meridian to 180° E or W).

// When correcting for errors with Targetm execute Plane change (correct for latitude) first at apoapsis,
// then phasing (correct for longitude) at periapsis

namespace MuMech
{
    namespace Landing
    {
        public class LandingEquations
        {
            public double h; // h: current altitude delta to target (m)
            public double dragUp; // Vertical Drag Force
            public double mass; // mass: current vessel mass (kg)
            public double currentVv; // currentVv: current downward vertical velocity(m/s)
            public double k; // Drag Coefficient to be calculated at each sample period.
            public double exp; // Exponent calculated from the altitude delta and k. at each sample period.
            public double beta; // beta: net upward acceleration without drag (T/m - g, m/s^2)
            public double tom; // tom: Thrust-over-mass ratio (m/s^2), i.e., available acceleration. Obtained from vessel.maxThrust / vessel.mass.
            public double g; // g: Local gravitational acceleration magnitude (m/s^2). Calculated as vessel.mainBody.GeeASL * 9.81 or more precisely GM / r^2 where r is distance to body center.
            public const double G_DEFAULT = 9.81; // Default gravitational acceleration in m/s^2
                                                  
                                                  

            public void StartSamplePeriod(double _h, double _dragUp, double _mass, double _currentVv, double _tom, double _g = G_DEFAULT)
            {
                dragUp = _dragUp;
                mass = _mass;
                currentVv = _currentVv;
                h = _h;
                k = ComputeK();
                exp = Math.Exp(2 * k * h) - 1;
                tom = _tom;
                g = _g;
                beta = tom - g;
            }

            // Computes k from dragUp for internal use
            private double ComputeK()
            {
                return (currentVv > 0 && mass > 0 && dragUp > 0) ? dragUp / (mass * currentVv * currentVv) : 0;
            }

            // Calculates the reference vertical velocity during landing burn accounting for drag.
            // Returns: reference downward vertical velocity (m/s)
            public double VerticalRefVelocity()
            {
                if (h <= 0 || beta <= 0) return 0;
                if (k <= 0) return 0;
                return exp > 0 ? Math.Sqrt(beta / k * exp) : 0;
            }

            // Calculates the time-to-go for vertical descent during landing burn with drag.
            // Returns: time to landing (s), or infinity if invalid
            public double VerticalTimeToGo()
            {
                if ((h <= 0) || (beta <= 0) || (k <= 0) || (exp <= 0)) return double.PositiveInfinity;
                return Math.Atan(Math.Sqrt(exp)) / Math.Sqrt(k * beta);
            }

            // Calculates the time-to-go for horizontal closure to target assuming constant deceleration.
            // d: current range to target (m)
            // vh: current horizontal velocity toward target (m/s)
            // Returns: time to target (s), or infinity if vh <= 0
            public static double HorizontalTimeToGo(double d, double vh)
            {
                return vh > 0 ? 2 * d / vh : double.PositiveInfinity;
            }

            // Determines if the landing burn should start by comparing vertical and horizontal times-to-go.
            // d: range to target (m)
            // vh: horizontal velocity (m/s)
            // tol: tolerance for time equality (s)
            // Returns: true if burn should start
            public bool ShouldStartLandingBurn(double d, double vh, double tol = 0.1)
            {
                return Math.Abs(VerticalTimeToGo() - HorizontalTimeToGo(d, vh)) <= tol;
            }

            // Calculates the reference horizontal velocity during landing burn.
            // d: current range to target (m)
            // d_init: initial range at burn start (m)
            // vh_init: initial horizontal velocity at burn start (m/s)
            // Returns: reference horizontal velocity (m/s)
            public static double HorizontalRefVelocity(double d, double d_init, double vh_init)
            {
                return d_init > 0 ? vh_init * Math.Sqrt(d / d_init) : 0;
            }

            // Fallback: Reference vertical velocity without drag.
            // Returns: reference downward vertical velocity (m/s)
            public double VerticalRefVelocityNoDrag()
            {
                return beta > 0 ? Math.Sqrt(2 * beta * h) : 0;
            }

            // Fallback: Time-to-go for vertical descent without drag.
            // Returns: time to landing (s)
            public double VerticalTimeToGoNoDrag()
            {
                return beta > 0 ? Math.Sqrt(2 * h / beta) : double.PositiveInfinity;
            }

            // Fallback: Determines if landing burn should start without drag.
            // d: range (m)
            // vh: horizontal velocity (m/s)
            // tol: tolerance (s)
            // Returns: true if burn should start
            public bool ShouldStartLandingBurnNoDrag(double d, double vh, double tol = 0.1)
            {
                return Math.Abs(VerticalTimeToGoNoDrag() - HorizontalTimeToGo(d, vh)) <= tol;
            }

            // Calculates required altitude delta for entry burn to reduce speed.
            // v: current total speed (m/s)
            // vf: final desired speed (m/s)
            // Returns: required delta height (m)
            public double EntryRequiredDeltaH(double v, double vf)
            {
                return beta > 0 ? (v * v - vf * vf) / (2 * beta) : double.PositiveInfinity;
            }

            // Determines if entry burn should start.
            // delta_h_avail: available altitude delta (m)
            // req_delta_h: required delta height (m)
            // Returns: true if burn should start
            public static bool ShouldStartEntryBurn(double delta_h_avail, double req_delta_h)
            {
                return delta_h_avail >= req_delta_h;
            }

            // Computes the desired attitude (thrust direction) unit vector for burns during deorbit or landing phases.
            // Adjusts for position error to minimize deviation from target landing position.
            // Suitable for deorbit (primarily retrograde with latitude adjustments) or landing (anti-gravity with horizontal corrections).
            // vel: Current velocity vector of the vessel (m/s). Obtained from vessel.obt_velocity or vessel.srf_velocity depending on frame.
            // posErr: Position error vector (m) = targetPosition - estimatedLandingPosition. Computed using trajectory prediction from MechJeb's simulators.
            // targetDir: Unit vector pointing horizontally toward the target. Derived by normalizing the projection of (targetPos - vesselPos) onto the surface plane (perpendicular to upAxis).
            // tgo: Estimated time-to-go to landing or periapsis (s). Sourced from LandingEquations.VerticalTimeToGo or similar predictive functions.
            // Kp: Proportional gain for correction (default 0.5). A tunable parameter to control how aggressively the position error is corrected; values between 0.1-1.0 recommended to avoid oscillation.
            // Returns: Unit vector indicating the direction the ship should point for thrust application.
            public Vector3d BurnAttitude(Vector3d vel, Vector3d posErr, Vector3d targetDir, double tgo, double Kp = 0.5)
            {
                // Check for invalid time-to-go; fallback to pure retrograde burn if tgo is non-positive.
                if (tgo <= 0) return -vel.normalized; // Pure retrograde: opposite to velocity for basic deceleration.

                // Compute anti-gravity direction: assumes Vector3d.up is the radial outward direction (up from surface).
                Vector3d up = -g * Vector3d.up.normalized; // Direction to counteract gravity; scaled by g but normalized later.

                // Calculate horizontal correction acceleration: proportional to position error, inversely to tgo squared (quadratic guidance law approximation).
                Vector3d horizCorr = Kp * posErr / (tgo * tgo); // Acceleration needed to null error over remaining time, scaled by Kp.

                // Compose desired acceleration: anti-gravity component + horizontal correction + closure toward target using excess thrust.
                Vector3d desAccel = up * g + horizCorr + targetDir * beta; // Total accel: hover (up*g), error corr, and use remaining accel (tom-g) for target approach.

                // Normalize to get unit vector for attitude control.
                return desAccel.normalized; // Final thrust direction.
            }

            // Computes combined attitude vector for simultaneous latitude and longitude error reduction.
            // Optimizes by vector addition: normal for lat, pro/retro for lon.
            // vessel: Vessel object containing orbit, obt_velocity, vessel, etc.
            // targetLat: Target latitude (degrees).
            // targetLon: Target longitude (degrees).
            // Returns: Unit vector for combined burn direction.
            public Vector3d CombinedAttitudeForAdj(Vessel vessel, double targetLat, double targetLon,
                ref double latErrorDeg, ref double lonErrorDeg)
            {
                Orbit orbit = vessel.orbit; // Current orbit from Vessel.
                Vector3d vel = orbit.vel; // Orbital velocity from Orbit.
                CelestialBody body = vessel.mainBody; // Main body for calculations.

                double currentUT = Planetarium.GetUniversalTime(); // Current universal time.
                double ut = orbit.NextPeriapsisTime(currentUT); // Time to next periapsis.
                Vector3d pos = orbit.getPositionAtUT(ut); // Position at periapsis.

                double predLat = body.GetLatitude(pos); // Predicted latitude at PE (degrees).
                double fixedLon = body.GetLongitude(pos); // Fixed-frame longitude at PE (degrees).

                double deltaT = ut - currentUT; // Time delta to PE (s).
                double rotAngle = 360 * deltaT / body.rotationPeriod; // Body rotation during deltaT (degrees).
                double predLon = (fixedLon - rotAngle + 360) % 360 - 180; // Predicted surface longitude at PE (-180 to 180).

                latErrorDeg = targetLat - predLat; // Latitude error (degrees).
                lonErrorDeg = targetLon - predLon; // Longitude error (degrees).
                lonErrorDeg = (lonErrorDeg + 180) % 360 - 180; // Normalize to -180 to 180.

                double latErrorRad = latErrorDeg * Math.PI / 180; // Latitude error (radians).
                double lonErrorRad = lonErrorDeg * Math.PI / 180; // Longitude error (radians).
                double targetLatRad = targetLat * Math.PI / 180; // Target latitude (radians).

                double vBurn = vel.magnitude; // Burn velocity magnitude (m/s).
                double dvLat = 2 * vBurn * Math.Sin(latErrorRad / (2 * Math.Cos(targetLatRad))); // DV for lat adjustment (approximated from NormalDVForLatAdj).
                double dvLon = 2 * vBurn * Math.Sin(lonErrorRad / 2); // DV for lon phasing (approximate using similar formula for small angles).

                Vector3d prograde = vel.normalized; // Prograde direction.
                Vector3d normal = orbit.GetOrbitNormal().normalized; // Normal direction.
                Vector3d latDir = (latErrorRad > 0 ? normal : -normal); // Lat correction dir.
                Vector3d lonDir = (lonErrorRad > 0 ? prograde : -prograde); // Lon correction dir.
                Vector3d combined = (latDir * dvLat + lonDir * dvLon).normalized; // Weighted sum, normalized.
                return combined;
            }

            // Computes remaining time before adjustment burn if deorbit timing fails.
            // Assumes burn at apoapsis if lat error dominant, periapsis if lon dominant.
            // vessel: Vessel with orbit and body
            // targetLat: Target lat (degrees).
            // targetLon: Target lon (degrees).
            // OUTPUT => latErrorDeg: Calculated Latitude error between target & vessel latitude at periapsis.
            // OUTPUT => lonErrorDeg: Calculated Longitude error between target & vessel Longitude at periapsis.
            // Returns: Time from now to burn start (s).
            public double TimeRemainingToAdjustmentBurn(Vessel vessel, double targetLat, double targetLon,
                ref double latErrorDeg, ref double lonErrorDeg)
            {
                Orbit orbit = vessel.orbit;
                CelestialBody body = vessel.mainBody;

                // Compute errors as in CombinedAttitudeForAdj.
                double currentUT = Planetarium.GetUniversalTime();
                double ut = orbit.NextPeriapsisTime(currentUT);
                Vector3d pos = orbit.getPositionAtUT(ut);
                double predLat = body.GetLatitude(pos);
                double fixedLon = body.GetLongitude(pos);
                double deltaT = ut - currentUT;
                double rotAngle = 360 * deltaT / body.rotationPeriod;
                double predLon = (fixedLon - rotAngle + 360) % 360 - 180;
                latErrorDeg = Math.Abs(targetLat - predLat);
                lonErrorDeg = Math.Abs((targetLon - predLon + 180) % 360 - 180);

                double vBurn = vessel.obt_velocity.magnitude;
                double latErrorRad = latErrorDeg * Math.PI / 180;
                double lonErrorRad = lonErrorDeg * Math.PI / 180;
                double targetLatRad = targetLat * Math.PI / 180;
                double dvLat = 2 * vBurn * Math.Sin(latErrorRad / (2 * Math.Cos(targetLatRad)));
                double dvLon = 2 * vBurn * Math.Sin(lonErrorRad / 2);
                double totalDv = Math.Sqrt(dvLat * dvLat + dvLon * dvLon); // Approx total DV for combined.
                double burnDur = EstBurnTime(totalDv, tom);

                bool latDominant = latErrorDeg > lonErrorDeg;
                double timeToPoint = latDominant ? orbit.timeToAp : orbit.timeToPe;
                return timeToPoint - burnDur / 2;
            }

            // Computes attitude vector for deorbit burn combined with lat/long adjustments if optimal (small errors <5°; saves Δv ~5-10%).
            // Otherwise, fallback to pure retrograde.
            // vessel: Vessel with orbit, velocity
            // targetLat: Target lat (degrees).
            // targetLon: Target lon (degrees).
            // peDesAlt: Desired PE alt (m, e.g., 70e3).
            // Returns: Unit vector for burn direction.
            public Vector3d CombinedDeorbitAttitude(Vessel vessel, double targetLat, double targetLon, double peDesAlt,
                ref double latErrorDeg, ref double lonErrorDeg)
            {
                Orbit orbit = vessel.orbit;
                Vector3d vel = vessel.obt_velocity;
                CelestialBody body = vessel.mainBody;
                double mu = body.gravParameter;
                double R = body.Radius;

                double dvDeorbit = DeorbitDVKeepApo(mu, R, orbit.ApA, orbit.PeA, peDesAlt); // Deorbit DV.

                // Compute errors as in CombinedAttitudeForAdj.
                double currentUT = Planetarium.GetUniversalTime();
                double ut = orbit.NextPeriapsisTime(currentUT);
                Vector3d pos = orbit.getPositionAtUT(ut);
                double predLat = body.GetLatitude(pos);
                double fixedLon = body.GetLongitude(pos);
                double deltaT = ut - currentUT;
                double rotAngle = 360 * deltaT / body.rotationPeriod;
                double predLon = (fixedLon - rotAngle + 360) % 360 - 180;
                latErrorDeg = Math.Abs(targetLat - predLat);
                lonErrorDeg = Math.Abs((targetLon - predLon + 180) % 360 - 180);

                if (latErrorDeg < 5 && lonErrorDeg < 5) // Optimal to combine for small errors.
                {
                    double latErrorRad = (targetLat - predLat) * Math.PI / 180;
                    double lonErrorRad = (targetLon - predLon) * Math.PI / 180;
                    double targetLatRad = targetLat * Math.PI / 180;
                    double vBurn = vel.magnitude;
                    double dvLat = 2 * vBurn * Math.Sin(latErrorRad / (2 * Math.Cos(targetLatRad)));
                    double dvLon = 2 * vBurn * Math.Sin(lonErrorRad / 2);

                    Vector3d retro = -vel.normalized; // Deorbit retrograde.
                    Vector3d normal = orbit.GetOrbitNormal().normalized;
                    Vector3d latDir = (latErrorRad > 0 ? normal : -normal);
                    Vector3d lonDir = (lonErrorRad > 0 ? retro : -retro); // Lon adj aligns with retro/pro.
                    Vector3d combined = (retro * dvDeorbit + latDir * dvLat + lonDir * dvLon).normalized;
                    return combined;
                }
                else
                {
                    return -vel.normalized; // Pure retrograde if not optimal.
                }
            }

            // Computes remaining time before deorbit burn start.
            // vessel: Vessel with orbit and body
            // targetLat: Target lat (degrees).
            // targetLon: Target lon (degrees).
            // peDesAlt: Desired PE alt (m).
            // Returns: Time from now to burn start (s), or NaN if no solution.
            public double TimeRemainingToDeorbitBurn(Vessel vessel, double targetLat, double targetLon, double peDesAlt)
            {
                Orbit orbit = vessel.orbit;
                CelestialBody body = vessel.mainBody;
                double mu = body.gravParameter;
                double R = body.Radius;
                double rotPeriod = body.rotationPeriod;

                double dv = DeorbitDVKeepApo(mu, R, orbit.ApA, orbit.PeA, peDesAlt);
                double burnDur = EstBurnTime(dv, tom);

                // Use iterative method for accuracy with target lon.
                return TimeToBurnIterative(vessel, targetLon * Math.PI / 180, mu, R, peDesAlt, rotPeriod, tom) - burnDur / 2;
            }

            // Deorbit DV to drop periapsis (PE) to peDesAlt by burning retro at apoapsis (keeps apoapsis ~fixed).
            // Used for low TWR: higher entry altitude reduces heating.
            // mu: gravitational parameter of body (m^3/s^2) - from vessel.mainBody.Mu; scales orbital mechanics.
            // R: equatorial radius of body (m) - from vessel.mainBody.Radius; baseline for altitudes.
            // apoAlt: current apoapsis altitude (m) - from orbit.apoapsis; burn location.
            // peAltCurr: current periapsis altitude (m) - from orbit.periapsis; initial low point.
            // peDesAlt: desired periapsis altitude for entry (m, e.g., 70km Earth) - sets entry interface.
            // Returns: required delta-v (m/s) for burn.
            public static double DeorbitDVKeepApo(double mu, double R, double apoAlt, double peAltCurr, double peDesAlt)
            {
                double r_ap = R + apoAlt; // Apoapsis radius (m): adds alt to body radius.
                double r_pe_curr = R + peAltCurr; // Current PE radius (m).
                double a_curr = (r_ap + r_pe_curr) / 2; // Current semi-major axis (m): orbit energy param.
                double v_curr = Math.Sqrt(mu * (2 / r_ap - 1 / a_curr)); // Current velocity at apo (m/s): vis-viva.
                double r_pe_des = R + peDesAlt; // Desired PE radius (m).
                double v_req = Math.Sqrt(mu * (2 / r_ap - 2 / (r_ap + r_pe_des))); // Required velocity at apo for new orbit (m/s).
                return v_curr - v_req; // DV needed (m/s): difference lowers PE.
            }

            // Deorbit DV to drop PE to peDesAlt by burning retro at periapsis (raises apoapsis to old PE level).
            // Used for high TWR: quicker deorbit, steeper entry.
            // mu, R, peAltCurr, peDesAlt: as above.
            // apoAltCurr: current apoapsis altitude (m) - from orbit.apoapsis.
            // Returns: required delta-v (m/s).
            public static double DeorbitDVRaiseApo(double mu, double R, double peAltCurr, double apoAltCurr, double peDesAlt)
            {
                double r_pe = R + peAltCurr; // PE radius (m): burn location.
                double r_ap_curr = R + apoAltCurr; // Current apo radius (m).
                double a_curr = (r_ap_curr + r_pe) / 2; // Current semi-major axis (m).
                double v_curr = Math.Sqrt(mu * (2 / r_pe - 1 / a_curr)); // Current velocity at PE (m/s).
                double r_pe_des = R + peDesAlt; // Desired PE radius (m).
                double v_req = Math.Sqrt(mu * (2 / r_pe - 2 / (r_pe + r_pe_des))); // Required velocity at PE (m/s).
                return v_curr - v_req;
            }

            // Estimates burn duration assuming constant thrust.
            // dv: delta-v required (m/s) - from DeorbitDV* methods.
            // tom: thrust-over-mass acceleration (m/s^2) - maxThrust / mass; limits burn accel.
            // Returns: burn time (s).
            public static double EstBurnTime(double dv, double tom)
            {
                return tom > 0 ? dv / tom : double.PositiveInfinity;
            }

            // Estimates time from deorbit burn start (at apo) to new periapsis.
            // Used in lon timing adj.
            // mu, R, apoAlt, peDesAlt: as above.
            // Returns: time to PE post-burn (s).
            public static double EstTimeToPEAfterDeorbit(double mu, double R, double apoAlt, double peDesAlt)
            {
                double ra = R + apoAlt; // Apo radius (m).
                double rp = R + peDesAlt; // Desired PE radius (m).
                double a = (ra + rp) / 2; // New semi-major axis (m).
                double period = 2 * Math.PI * Math.Sqrt(a * a * a / mu); // New orbital period (s).
                return period / 2; // Half-period: apo to PE time (s).
            }

            // Computes timing adjustment for longitude error to align PE with target.
            // deltaLonRad: longitude error (rad) = predicted PE lon - target lon (>0: PE east of target, delay burn).
            // mu, R, apoAlt, peDesAlt: as above.
            // rotPeriod: body rotation period (s) - mainBody.rotationPeriod; for surface motion.
            // Returns: seconds to add to nominal burn time (+ delay).
            public static double DeorbitLonTimingAdj(double deltaLonRad, double mu, double R, double apoAlt, double peDesAlt, double rotPeriod)
            {
                if (rotPeriod <= 0) return 0;
                double tpe = EstTimeToPEAfterDeorbit(mu, R, apoAlt, peDesAlt); // Time apo to PE (s).
                double omegaB = 2 * Math.PI / rotPeriod; // Body angular velocity (rad/s).
                double ra = R + apoAlt; // Apo radius (m).
                double n = Math.Sqrt(mu / (ra * ra * ra)); // Mean motion at apo (rad/s).
                double rel = n - omegaB; // Relative angular rate (rad/s): orbit vs surface.
                return rel > 0 ? deltaLonRad / rel : 0; // Adj time (s).
            }

            // Computes normal (out-of-plane) DV for small latitude plane change.
            // latErrorRad: latitude error (rad) = predicted lat - target lat.
            // targetLatRad: target latitude (rad).
            // vBurn: burn velocity (~orbital speed at burn, m/s).
            // Returns: additional DV (m/s).
            public static double NormalDVForLatAdj(double latErrorRad, double targetLatRad, double vBurn)
            {
                double cosL = Math.Cos(targetLatRad); // Cosine of target lat: adjusts for plane change efficiency.
                double deltaI = latErrorRad / cosL; // Effective inclination change (rad).
                return 2 * vBurn * Math.Sin(deltaI / 2); // Plane change DV (m/s): small-angle approx.
            }

            // Determines deorbit mode: KeepApo (low TWR, safer entry) vs RaiseApo (high TWR, quicker).
            // twr: current TWR = maxThrust / (mass * g); measures accel capability.
            // threshold: TWR switch point (e.g., 1.5); below = KeepApo.
            // Returns: 0=KeepApo, 1=RaiseApo.
            public static int ChooseDeorbitMode(double twr, double threshold = 1.5)
            {
                return twr >= threshold ? 1 : 0; // 1=RaiseApo if TWR high.
            }

            // General time to deorbit burn start, using true anomaly.
            // orbit: Current Orbit - vessel.orbit.
            // deltaTA: Adjustment from current (rad) - from DeltaTAForLonAdj.
            // burnDur: Est burn duration (s) - from EstBurnTime.
            // Returns: Time from now to burn start (s).
            public static double TimeToBurnGeneral(Orbit orbit, double deltaTA, double burnDur)
            {
                double currTA = orbit.trueAnomaly; // Current true anomaly (rad).
                double desTA = currTA + deltaTA; // Desired TA for burn center.
                double timeToDesTA = orbit.GetDTforTrueAnomaly(desTA, 7200); // Time to des TA (s).
                return timeToDesTA - burnDur / 2; // Start half-burn before center.
            }

            // Fallback: Time to optimal deorbit node using numerical prediction.
            // Predicts impact lon after deorbit from burnTime, iterates to minimize error.
            // targetLonRad: Target longitude (rad).
            // mu, R, peDesAlt, rotPeriod: As before.
            // tom: Thrust/mass (m/s^2).
            // maxIter: Max iterations (default 20).
            // Returns: Time to burn start (s), or NaN if no convergence.
            public static double TimeToBurnIterative(Vessel vessel, double targetLonRad, double mu, double R, double peDesAlt, double rotPeriod, double tom, int maxIter = 20)
            {
                Orbit orbit = vessel.orbit; // Current orbit from Vessel.
                double tol = 1e-3; // Time tol (s).
                double guess = orbit.GetTimeToPeriapsis() - EstBurnTime(DeorbitDVKeepApo(mu, R, orbit.ApA, orbit.PeA, peDesAlt), tom) / 2; // Initial guess at PE.
                for (int i = 0; i < maxIter; i++)
                {
                    // Simulate deorbit at guess time: compute new orbit, PE lon.
                    double predPeLon = PredictPeLonAfterBurn(vessel, guess, mu, R, peDesAlt);
                    double deltaLon = predPeLon - targetLonRad;
                    double adj = DeorbitLonTimingAdj(deltaLon, mu, R, orbit.ApA, peDesAlt, rotPeriod); // Reuse adj.
                    guess += adj;
                    if (Math.Abs(adj) < tol) return guess;
                }
                return double.NaN; // No convergence.
            }

            // Predicts the longitude of the periapsis after performing a deorbit burn at the specified guess time.
            // Assumes a retrograde burn to set the new periapsis to peDesAlt, treating the burn point as the new apoapsis (approximation valid for low-eccentricity orbits).
            // orbit: Current orbit of the vessel.
            // guess: Time from now to the burn start (s).
            // mu: Gravitational parameter of the body (m^3/s^2) - vessel.mainBody.GM.
            // R: Radius of the body (m) - vessel.mainBody.Radius.
            // peDesAlt: Desired periapsis altitude (m).
            // body: Reference celestial body - vessel.mainBody.
            // Returns: Predicted longitude at the new periapsis (degrees).
            public static double PredictPeLonAfterBurn(Vessel vessel, double guess, double mu, double R, double peDesAlt)
            {
                Orbit orbit = vessel.orbit; // Current orbit from Vessel.
                CelestialBody body = vessel.mainBody;
                double currentUT = Planetarium.GetUniversalTime();
                double burnUT = currentUT + guess;
                Vector3d relPos = orbit.getRelativePositionAtUT(burnUT);
                Vector3d vel = orbit.getOrbitalVelocityAtUT(burnUT);
                double r = relPos.magnitude;
                double v_curr = vel.magnitude;
                double rp = R + peDesAlt;
                double a_new = (r + rp) / 2;
                if (a_new <= 0 || rp >= r) return double.NaN; // Invalid orbit
                double v_new = Math.Sqrt(mu * (2 / r - 1 / a_new));
                double dv = v_curr - v_new;
                Vector3d retroDir = -vel.normalized;
                Vector3d newVel = vel + dv * retroDir;
                Orbit newOrbit = new Orbit();
                newOrbit.UpdateFromStateVectors(relPos, newVel, body, burnUT);
                double peUT = newOrbit.NextPeriapsisTime(burnUT);
                Vector3d pePos = newOrbit.getPositionAtUT(peUT);
                double peLon = body.GetLongitude(pePos);
                return peLon;
            }

            // Computes combined attitude vector using closest approach for errors.
            // vessel: Vessel object.
            // targetLat: Target latitude (degrees).
            // targetLon: Target longitude (degrees).
            // latErrorDeg: Output latitude error (degrees).
            // lonErrorDeg: Output longitude error (degrees).
            // Returns: Unit vector for combined burn direction.
            public Vector3d CombinedAttitudeForAdjGT(Vessel vessel, double targetLat, double targetLon, out double latErrorDeg, out double lonErrorDeg)
            {
                double timeToClosest;
                CalculateClosestApproach(vessel, targetLat, targetLon, out timeToClosest);

                Orbit orbit = vessel.orbit;
                Vector3d vel = vessel.obt_velocity;
                CelestialBody body = vessel.mainBody;

                double currentUT = Planetarium.GetUniversalTime();
                double ut = currentUT + timeToClosest;
                Vector3d pos = orbit.getPositionAtUT(ut);

                double predLat = body.GetLatitude(pos);
                double fixedLon = body.GetLongitude(pos);

                double deltaT = timeToClosest;
                double rotAngle = 360 * deltaT / body.rotationPeriod;
                double predLon = (fixedLon - rotAngle + 360) % 360 - 180;

                latErrorDeg = targetLat - predLat;
                lonErrorDeg = targetLon - predLon;
                lonErrorDeg = (lonErrorDeg + 180) % 360 - 180;

                double latErrorRad = latErrorDeg * Math.PI / 180;
                double lonErrorRad = lonErrorDeg * Math.PI / 180;
                double targetLatRad = targetLat * Math.PI / 180;

                double vBurn = vel.magnitude;
                double dvLat = 2 * vBurn * Math.Sin(latErrorRad / (2 * Math.Cos(targetLatRad)));
                double dvLon = 2 * vBurn * Math.Sin(lonErrorRad / 2);

                Vector3d prograde = vel.normalized;
                Vector3d normal = orbit.GetOrbitNormal().normalized;
                Vector3d latDir = (latErrorRad > 0 ? normal : -normal);
                Vector3d lonDir = (lonErrorRad > 0 ? prograde : -prograde);
                Vector3d combined = (latDir * dvLat + lonDir * dvLon).normalized;
                return combined;
            }

            // Computes time to adjustment burn using closest approach.
            // vessel: Vessel object.
            // targetLat: Target lat (degrees).
            // targetLon: Target lon (degrees).
            // latErrorDeg: Output latitude error (degrees).
            // lonErrorDeg: Output longitude error (degrees).
            // Returns: Time to burn start (s).
            public double TimeRemainingToAdjustmentBurnGT(Vessel vessel, double targetLat, double targetLon, out double latErrorDeg, out double lonErrorDeg)
            {
                double timeToClosest;
                CalculateClosestApproach(vessel, targetLat, targetLon, out timeToClosest);

                Orbit orbit = vessel.orbit;
                CelestialBody body = vessel.mainBody;

                double currentUT = Planetarium.GetUniversalTime();
                double ut = currentUT + timeToClosest;
                Vector3d pos = orbit.getPositionAtUT(ut);
                double predLat = body.GetLatitude(pos);
                double fixedLon = body.GetLongitude(pos);
                double deltaT = timeToClosest;
                double rotAngle = 360 * deltaT / body.rotationPeriod;
                double predLon = (fixedLon - rotAngle + 360) % 360 - 180;
                latErrorDeg = targetLat - predLat;
                lonErrorDeg = targetLon - predLon;
                lonErrorDeg = (lonErrorDeg + 180) % 360 - 180;

                double latErrorAbs = Math.Abs(latErrorDeg);
                double lonErrorAbs = Math.Abs(lonErrorDeg);

                double vBurn = vessel.obt_velocity.magnitude;
                double latErrorRad = latErrorDeg * Math.PI / 180;
                double lonErrorRad = lonErrorDeg * Math.PI / 180;
                double targetLatRad = targetLat * Math.PI / 180;
                double dvLat = 2 * vBurn * Math.Sin(latErrorRad / (2 * Math.Cos(targetLatRad)));
                double dvLon = 2 * vBurn * Math.Sin(lonErrorRad / 2);
                double totalDv = Math.Sqrt(dvLat * dvLat + dvLon * dvLon);
                double burnDur = EstBurnTime(totalDv, tom);

                bool latDominant = latErrorAbs > lonErrorAbs;
                double timeToPoint = latDominant ? orbit.timeToAp : orbit.timeToPe; // Still use ap/pe for burn point.
                return timeToPoint - burnDur / 2;
            }

            // Computes attitude for deorbit burn combined with adjustments using closest approach.
            // vessel: Vessel object.
            // targetLat: Target lat (degrees).
            // targetLon: Target lon (degrees).
            // peDesAlt: Desired PE alt (m).
            // latErrorDeg: Output latitude error (degrees).
            // lonErrorDeg: Output longitude error (degrees).
            // Returns: Unit vector for burn direction.
            public Vector3d CombinedDeorbitAttitudeGT(Vessel vessel, double targetLat, double targetLon, double peDesAlt,
                out double latErrorDeg, out double lonErrorDeg)
            {
                double timeToClosest;
                CalculateClosestApproach(vessel, targetLat, targetLon, out timeToClosest);

                Orbit orbit = vessel.orbit;
                Vector3d vel = vessel.obt_velocity;
                CelestialBody body = vessel.mainBody;
                double mu = body.gravParameter;
                double R = body.Radius;

                double dvDeorbit = DeorbitDVKeepApo(mu, R, orbit.ApA, orbit.PeA, peDesAlt);

                double currentUT = Planetarium.GetUniversalTime();
                double ut = currentUT + timeToClosest;
                Vector3d pos = orbit.getPositionAtUT(ut);
                double predLat = body.GetLatitude(pos);
                double fixedLon = body.GetLongitude(pos);
                double deltaT = timeToClosest;
                double rotAngle = 360 * deltaT / body.rotationPeriod;
                double predLon = (fixedLon - rotAngle + 360) % 360 - 180;
                latErrorDeg = Math.Abs(targetLat - predLat);
                lonErrorDeg = Math.Abs((targetLon - predLon + 180) % 360 - 180);

                if (latErrorDeg < 5 && lonErrorDeg < 5)
                {
                    double latErrorRad = (targetLat - predLat) * Math.PI / 180;
                    double lonErrorRad = (targetLon - predLon) * Math.PI / 180;
                    double targetLatRad = targetLat * Math.PI / 180;
                    double vBurn = vel.magnitude;
                    double dvLat = 2 * vBurn * Math.Sin(latErrorRad / (2 * Math.Cos(targetLatRad)));
                    double dvLon = 2 * vBurn * Math.Sin(lonErrorRad / 2);

                    Vector3d retro = -vel.normalized;
                    Vector3d normal = orbit.GetOrbitNormal().normalized;
                    Vector3d latDir = (latErrorRad > 0 ? normal : -normal);
                    Vector3d lonDir = (lonErrorRad > 0 ? retro : -retro);
                    Vector3d combined = (retro * dvDeorbit + latDir * dvLat + lonDir * dvLon).normalized;
                    return combined;
                }
                else
                {
                    return -vel.normalized;
                }
            }

            // Computes time to deorbit burn using closest approach.
            // vessel: Vessel object.
            // targetLat: Target lat (degrees).
            // targetLon: Target lon (degrees).
            // peDesAlt: Desired PE alt (m).
            // Returns: Time to burn start (s), or NaN if no solution.
            public double TimeRemainingToDeorbitBurnGT(Vessel vessel, double targetLat, double targetLon, double peDesAlt)
            {
                double timeToClosest;
                CalculateClosestApproach(vessel, targetLat, targetLon, out timeToClosest);

                CelestialBody body = vessel.mainBody;
                double mu = body.gravParameter;
                double R = body.Radius;
                //double rotPeriod = body.rotationPeriod;

                double dv = DeorbitDVKeepApo(mu, R, vessel.orbit.ApA, vessel.orbit.PeA, peDesAlt);
                double burnDur = EstBurnTime(dv, tom);

                return timeToClosest - burnDur / 2; // Burn before closest to set PE there.
            }

            // Calculates the closest approach distance on the groundtrack to the target location and the time to it.
            // Samples the orbit at regular intervals (default 360 points for 1 deg resolution).
            // If timeToClosestCenter is provided, refines around it within 2 deg range with finer sampling (error ≤5 km on Earth).
            // vessel: Vessel object with orbit and mainBody.
            // targetLat: Target latitude (degrees).
            // targetLon: Target longitude (degrees).
            // out timeToClosest: Time from now to closest approach (s).
            // samples: Number of samples over one orbital period (default 360).
            // timeToClosestCenter: Optional center time for refinement (s, default NaN for rough search only).
            // Returns: Closest great-circle distance (m), or double.PositiveInfinity if invalid.
            public static double CalculateClosestApproach(Vessel vessel, double targetLat, double targetLon, out double timeToClosest, int samples = 360, double timeToClosestCenter = double.NaN)
            {
                Orbit orbit = vessel.orbit;
                CelestialBody body = vessel.mainBody;
                double mu = body.gravParameter;
                double rotPeriod = body.rotationPeriod;
                double R = body.Radius;
                double period = orbit.period;

                if (double.IsNaN(period) || period <= 0)
                {
                    timeToClosest = double.NaN;
                    return double.PositiveInfinity;
                }

                double currentUT = Planetarium.GetUniversalTime();
                double minDist = double.PositiveInfinity;
                double bestTime = double.NaN;
                double step = period / samples;

                if (double.IsNaN(timeToClosestCenter))
                {
                    // Rough search.
                    for (int i = 0; i < samples; i++)
                    {
                        double ut = currentUT + i * step;
                        Vector3d pos = orbit.getPositionAtUT(ut);
                        double lat, lon, alt;
                        body.GetLatLonAlt(pos, out lat, out lon, out alt);

                        double deltaT = ut - currentUT;
                        double rotAngle = 360 * deltaT / rotPeriod;
                        double surfaceLon = (lon - rotAngle + 360) % 360 - 180;

                        double dist = ComputeRangeToTargetFromLatLon(lat, surfaceLon, targetLat, targetLon, R);

                        if (dist < minDist)
                        {
                            minDist = dist;
                            bestTime = deltaT;
                        }
                    }
                }
                else
                {
                    bestTime = timeToClosestCenter;
                }

                // Refine if center provided or after rough.
                if (!double.IsNaN(bestTime))
                {
                    double degRange = 2.0;
                    double fineSamples = degRange / 0.045; // ~44 samples for 5 km on Earth.
                    double fineStep = (period * degRange / 360) / fineSamples;
                    double startTime = bestTime - (period * degRange / 720); // Center around best.

                    minDist = double.PositiveInfinity;
                    bestTime = double.NaN;

                    for (int i = 0; i < (int)fineSamples; i++)
                    {
                        double deltaT = startTime + i * fineStep;
                        double ut = currentUT + deltaT;
                        Vector3d pos = orbit.getPositionAtUT(ut);
                        double lat, lon, alt;
                        body.GetLatLonAlt(pos, out lat, out lon, out alt);

                        double rotAngle = 360 * deltaT / rotPeriod;
                        double surfaceLon = (lon - rotAngle + 360) % 360 - 180;

                        double dist = ComputeRangeToTargetFromLatLon(lat, surfaceLon, targetLat, targetLon, R);

                        if (dist < minDist)
                        {
                            minDist = dist;
                            bestTime = deltaT;
                        }
                    }
                }

                timeToClosest = bestTime;
                return minDist;
            }

            // Computes great-circle distance between two lat/lon points (m).
            // lat1, lon1: First point lat/lon (deg).
            // lat2, lon2: Second point lat/lon (deg).
            // radius: Body radius (m).
            // Returns: Surface distance (m).
            public static double ComputeRangeToTargetFromLatLon(double lat1, double lon1, double lat2, double lon2, double radius)
            {
                double lat1Rad = lat1 * Math.PI / 180;
                double lat2Rad = lat2 * Math.PI / 180;
                double dlonRad = (lon1 - lon2) * Math.PI / 180;
                double a = Math.Sin((lat2Rad - lat1Rad) / 2) * Math.Sin((lat2Rad - lat1Rad) / 2) +
                           Math.Cos(lat1Rad) * Math.Cos(lat2Rad) * Math.Sin(dlonRad / 2) * Math.Sin(dlonRad / 2);
                double ang = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
                return radius * ang;
            }

            public static double TimeToNodeForSurfacePoint2(
                Vessel vessel, double targetLatDeg, double targetLonDeg)
            {
                Orbit current = vessel.orbit;
                CelestialBody body = current.referenceBody;

                double now = Planetarium.GetUniversalTime();

                // --- 1. Surface point in body-fixed frame ---
                Vector3d bodyFixed = body.GetRelSurfaceNVector(targetLatDeg, targetLonDeg);

                // --- 2. Planet rotation rate (rad/s) ---
                double rotRate = 2.0 * Math.PI / body.rotationPeriod;

                // Helper function: compute target plane normal at a given UT
                Vector3d GetTargetPlaneNormal(double UT)
                {
                    double dt = UT - now;

                    // Rotation angle since "now"
                    double angle = rotRate * dt;

                    // Rotate body-fixed vector around planet's rotation axis (Z)
                    QuaternionD rot = QuaternionD.AngleAxis(angle * Mathf.Rad2Deg, Vector3d.forward);
                    Vector3d rotatedBodyFixed = rot * bodyFixed;

                    // Convert to world space
                    Vector3d worldVec = body.transform.rotation * rotatedBodyFixed;

                    // Target plane normal = cross(surface vector, rotation axis)
                    Vector3d axisWorld = body.transform.up;
                    return Vector3d.Cross(worldVec, axisWorld).normalized;
                }

                // --- 3. First guess: compute target plane normal at current time ---
                Vector3d nTargetNow = GetTargetPlaneNormal(now);

                // --- 4. Build synthetic orbit for initial estimate ---
                Orbit targetOrbitNow = BuildOrbitFromPlaneNormal(current, body, nTargetNow);

                // --- 5. Get initial AN/DN UTs ---
                double utAN0 = current.TimeOfAscendingNode(targetOrbitNow, now);
                double utDN0 = current.TimeOfDescendingNode(targetOrbitNow, now);

                // --- 6. Recompute target plane at those future UTs ---
                Vector3d nTargetAN = GetTargetPlaneNormal(utAN0);
                Vector3d nTargetDN = GetTargetPlaneNormal(utDN0);

                // --- 7. Build corrected target orbits ---
                Orbit targetOrbitAN = BuildOrbitFromPlaneNormal(current, body, nTargetAN);
                Orbit targetOrbitDN = BuildOrbitFromPlaneNormal(current, body, nTargetDN);

                // --- 8. Compute corrected node times ---
                double utAN = current.TimeOfAscendingNode(targetOrbitAN, now);
                double utDN = current.TimeOfDescendingNode(targetOrbitDN, now);

                // --- 9. Convert to time offsets ---
                double timeToAN = utAN - now;
                double timeToDN = utDN - now;

                if (timeToAN < 0) timeToAN += current.period;
                if (timeToDN < 0) timeToDN += current.period;

                return (timeToAN == -1) ? (timeToDN) : ((timeToDN == -1) ? (timeToAN) : ((timeToAN < timeToDN) ? timeToAN : timeToDN));
            }

            // Helper: build an orbit from a plane normal
            private static Orbit BuildOrbitFromPlaneNormal(Orbit current, CelestialBody body, Vector3d n)
            {
                double inc = Math.Acos(n.z) * Mathf.Rad2Deg;
                double LAN = Math.Atan2(n.x, -n.y) * Mathf.Rad2Deg;
                if (LAN < 0) LAN += 360.0;

                return new Orbit(
                    inc,
                    current.eccentricity,
                    current.semiMajorAxis,
                    LAN,
                    current.argumentOfPeriapsis,
                    current.meanAnomalyAtEpoch,
                    current.epoch,
                    body
                );
            }

            /// <summary>
            /// Computes time until ascending/descending node relative to the orbital plane
            /// that passes over a fixed surface point (lat, lon) on the planet.
            /// </summary>
            public double TimeToNodeForSurfacePoint(
                Vessel vessel, double targetLatDeg, double targetLonDeg)
            {
                Orbit current = vessel.orbit;
                CelestialBody body = current.referenceBody;

                double now = Planetarium.GetUniversalTime();

                // 1. Convert target lat/lon to a world-space vector
                Vector3d surfacePoint = body.GetWorldSurfacePosition(
                    targetLatDeg, targetLonDeg, 0.0);

                // Convert to body-centered coordinates
                Vector3d r = surfacePoint - body.position;
                r.Normalize();

                // 2. Compute the normal of the orbital plane passing through that point
                // The plane must contain the planet's up-axis (body.transform.up)
                Vector3d bodyUp = body.transform.up;
                Vector3d planeNormal = Vector3d.Cross(r, bodyUp).normalized;

                // 3. Extract inclination and LAN from the plane normal
                double inc = Math.Acos(planeNormal.z) * Mathf.Rad2Deg;
                double LAN = Math.Atan2(planeNormal.x, -planeNormal.y) * Mathf.Rad2Deg;

                // Normalize angles
                if (LAN < 0) LAN += 360.0;

                // 4. Build synthetic target orbit
                Orbit targetOrbit = new Orbit(
                    inc,
                    current.eccentricity,
                    current.semiMajorAxis,
                    LAN,
                    current.argumentOfPeriapsis,
                    current.meanAnomalyAtEpoch,
                    current.epoch,
                    body
                );

                // 5. Compute AN/DN true anomalies
                double anTA = current.TimeOfAscendingNode(targetOrbit, now);
                double dnTA = current.TimeOfDescendingNode(targetOrbit, now);

                // 6. Convert to UT
                double utAN = current.GetUTforTrueAnomaly(anTA, now);
                double utDN = current.GetUTforTrueAnomaly(dnTA, now);

                // 7. Time differences
                double timeToAN = utAN - now;
                double timeToDN = utDN - now;

                if (timeToAN < 0) timeToAN += current.period;
                if (timeToDN < 0) timeToDN += current.period;

                return (timeToAN == -1) ? (timeToDN) : ((timeToDN == -1) ? (timeToAN) : ((timeToAN < timeToDN) ? timeToAN : timeToDN));
            }

            /// <summary>
            /// Computes the burn attitude (world-space direction) needed to reduce
            /// the plane difference between the current orbit and the target plane
            /// defined by a surface point (lat, lon).
            /// </summary>
            public static Vector3d GetPlaneCorrectionDirection(
                Vessel vessel, double targetLatDeg, double targetLonDeg)
            {
                Orbit current = vessel.orbit;
                CelestialBody body = current.referenceBody;

                // --- 1. Current orbit plane normal ---
                Vector3d nCurrent = current.GetOrbitNormal().normalized;

                // --- 2. Target plane normal from surface point ---
                Vector3d surfacePoint = body.GetWorldSurfacePosition(
                    targetLatDeg, targetLonDeg, 0.0);

                Vector3d r = (surfacePoint - body.position).normalized;
                Vector3d bodyUp = body.transform.up;

                // Plane normal = cross(surface point vector, planet up-axis)
                Vector3d nTarget = Vector3d.Cross(r, bodyUp).normalized;

                // --- 3. Compute rotation axis to reduce plane error ---
                // This is the direction you burn toward.
                Vector3d correctionAxis = Vector3d.Cross(nCurrent, nTarget);

                // If the planes are already aligned, return zero vector
                if (correctionAxis.sqrMagnitude < 1e-10)
                    return Vector3d.zero;

                correctionAxis.Normalize();

                // --- 4. Convert correction axis into a burn direction ---
                // The burn direction is perpendicular to the orbital plane,
                // so we project the correction axis into the vessel's orbital frame.
                Vector3d prograde = current.GetVel().normalized;
                Vector3d radial = Vector3d.Cross(nCurrent, prograde).normalized;
                Vector3d normal = nCurrent;

                // Decompose correction axis into orbital frame
                double cP = Vector3d.Dot(correctionAxis, prograde);
                double cR = Vector3d.Dot(correctionAxis, radial);
                double cN = Vector3d.Dot(correctionAxis, normal);

                // The burn direction is the component perpendicular to velocity
                Vector3d burnDir = (cR * radial + cN * normal).normalized;

                return burnDir;
            }

            // Computes time to next ascending/descending node relative to target latitude plane.
            // vessel: Vessel.
            // targetLatRad: Target latitude (rad).
            // ascending: True for AN, false for DN.
            // Returns: Time to node (s), or double.NaN if parallel.
            public static double TimeToNode(Vessel vessel, double targetLatRad, bool ascending)
            {
                Orbit orbit = vessel.orbit;
                double deg2rad = Math.PI / 180;
                double taAN = -orbit.argumentOfPeriapsis * deg2rad;
                double taNode = ascending ? taAN : taAN + Math.PI;

                double currentUT = Planetarium.GetUniversalTime();
                double time = orbit.TimeOfTrueAnomaly(taNode, currentUT);
                return time - currentUT;
            }

            public double TimeToNode(Vessel vessel, double targetLatDeg)
            {
                double UT;
                double targetLatRad = targetLatDeg * Math.PI / 180;
                double AUT = TimeToNode(vessel, targetLatRad, true);
                double DUT = TimeToNode(vessel, targetLatRad, false);

                UT = (AUT == -1) ? (DUT) : ((DUT == -1) ? (AUT) : ((AUT<DUT)?AUT:DUT) );
                return UT;
            }


            public Vector3d NodeBasedAlignmentAttitude2(
                Vessel vessel,
                double targetLatDeg,
                double targetLonDeg,
                double nodeUT = 0)
            {
                Orbit current = vessel.orbit;
                CelestialBody body = current.referenceBody;

                double now = Planetarium.GetUniversalTime();
                nodeUT = (nodeUT == 0) ? Planetarium.GetUniversalTime() + 1: nodeUT;

                // --- 1. Surface point in body-fixed frame ---
                Vector3d bodyFixed = body.GetRelSurfaceNVector(targetLatDeg, targetLonDeg);

                // --- 2. Planet rotation rate (rad/s) ---
                double rotRate = 2.0 * Math.PI / body.rotationPeriod;

                // --- 3. Compute target plane normal at the node UT ---
                double dt = nodeUT - now;
                double angle = rotRate * dt;

                // Rotate body-fixed vector around planet's rotation axis (Z)
                QuaternionD rot = QuaternionD.AngleAxis(angle * Mathf.Rad2Deg, Vector3d.forward);
                Vector3d rotatedBodyFixed = rot * bodyFixed;

                // Convert to world space
                Vector3d worldVec = body.transform.rotation * rotatedBodyFixed;

                // Target plane normal = cross(surface vector, rotation axis)
                Vector3d axisWorld = body.transform.up;
                Vector3d nTarget = Vector3d.Cross(worldVec, axisWorld).normalized;

                // --- 4. Current orbit plane normal ---
                Vector3d nCurrent = current.GetOrbitNormal().normalized;

                // --- 5. Correction axis: direction to rotate current plane toward target plane ---
                Vector3d correctionAxis = Vector3d.Cross(nCurrent, nTarget);

                if (correctionAxis.sqrMagnitude < 1e-10)
                    return Vector3d.zero; // Already aligned

                correctionAxis.Normalize();

                // --- 6. Convert correction axis into burn direction in orbital frame ---
                Vector3d prograde = current.GetVel().normalized;
                Vector3d radial = Vector3d.Cross(nCurrent, prograde).normalized;
                Vector3d normal = nCurrent;

                double cP = Vector3d.Dot(correctionAxis, prograde);
                double cR = Vector3d.Dot(correctionAxis, radial);
                double cN = Vector3d.Dot(correctionAxis, normal);

                // Burn direction is radial + normal components (no prograde)
                Vector3d burnDir = (cR * radial + cN * normal).normalized;

                return burnDir;
            }

            // Efficient alignment using nodes for latitude.
            public Vector3d NodeBasedAlignmentAttitude(Vessel vessel, double targetLat, out double latErrorDeg, double maxDvPerBurn = 150, double minErrorDeg = 0.05)
            {
                // Compute lat error as before.
                latErrorDeg = ComputeLatError(vessel, targetLat); // Reuse prior method.

                if (Math.Abs(latErrorDeg) < minErrorDeg) return Vector3d.zero;

                double v = vessel.obt_velocity.magnitude;
                double dvFull = 2 * v * Math.Sin(Math.Abs(latErrorDeg * Math.PI / 180) / 2);
                double scale = Math.Min(1.0, maxDvPerBurn / dvFull);
                Vector3d normal = vessel.orbit.GetOrbitNormal().normalized;
                Vector3d dir = latErrorDeg > 0 ? normal : -normal;
                return dir * scale;
            }

            // Computes predicted latitude error at closest approach.
            public double ComputeLatError(Vessel vessel, double targetLat)
            {
                double timeToClosest;
                CalculateClosestApproach(vessel, targetLat, 0, out timeToClosest); // Lon irrelevant for lat.

                Orbit orbit = vessel.orbit;
                CelestialBody body = vessel.mainBody;
                double currentUT = Planetarium.GetUniversalTime();
                double ut = currentUT + timeToClosest;
                Vector3d pos = orbit.getPositionAtUT(ut);
                double predLat = body.GetLatitude(pos);
                return targetLat - predLat;
            }
        }
    }
}
