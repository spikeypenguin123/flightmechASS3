V = 180*0.5144;
aircraft = Initialisation("CG1", 180, false);

[rho, Q] = FlowProperties(aircraft, V);

CL_level = 0.24;

A = Q*aircraft.geo.S*aircraft.geo.b^2/(aircraft.inertial.Ixx*2*V);

B = Q*aircraft.geo.S*aircraft.geo.b/aircraft.inertial.Ixx*aircraft.aero.Clda;

t_90 = 5;
p = deg2rad(90/t_90);
rad2deg(p);

delta_a = -p*aircraft.geo.b/(2*V)*aircraft.aero.Clp/aircraft.aero.Clda;

CL = aircraft.aero.CLo + deg2rad(1.2)*aircraft.aero.CLa + deg2rad(0.6)*aircraft.aero.CLad;
CD = aircraft.aero.Cdo+CL^2;
delta_t = CD*V/aircraft.prop.eta/aircraft.prop.P_max;

disp("Aileron deflection to turn 90deg in 5s: " + rad2deg(delta_a))

C = [0 aircraft.aero.Cydr aircraft.aero.Cyb; aircraft.aero.Clda aircraft.aero.Cldr aircraft.aero.Clb; aircraft.aero.Cnda aircraft.aero.Cndr aircraft.aero.Cnb];

F = [CL_level,0,0];

U = C/F;

rad2deg(U)

CL_level/aircraft.aero.Cndr

delta_e = rad2deg(CL/aircraft.aero.CLde)