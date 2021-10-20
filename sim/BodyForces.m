function [Forces, Moments] = BodyForces(FlightData,alpha,beta,alphadot,betadot,rho,V,[States],[Controls])

% INPUT:
% OUTPUT:

%% Computing the Body Forces and moments of the aircraft in the Body axes
% Content from lecture Week 3A and Week 8B
% NOTE: Still need to do rotation on matrix 20/10

% Reading in data
Cm0 = FlightData.Aero.;
Cma = FlightData.Aero.;
Camd = FlightData.Aero.;
Cmq = FlightData.Aero.;
Cmde = FlightData.Aero.;
CY0 = FlightData.Aero.;
CYb = FlightData.Aero.;
CYbd = FlightData.Aero.;
CYr = FlightData.Aero.;
CYp = FlightData.Aero.;
CYda = FlightData.Aero.;
CYdr = FlightData.Aero.;
Cl0 = FlightData.Aero.;
Clb = FlightData.Aero.;
Clbd = FlightData.Aero.;
Clr = FlightData.Aero.;
Clp = FlightData.Aero.;
Clda = FlightData.Aero.;
Cldr = FlightData.Aero.;
Cn0 = FlightData.Aero.;
Cnb = FlightData.Aero.;
Cnbd = FlightData.Aero.;
Cnr = FlightData.Aero.;
Cnp = FlightData.Aero.;
Cnda = FlightData.Aero.;
Cndr = FlightData.Aero.;

p = States(4);
q = States(5);
r = States(6);

delta_a = Controls(3);
delta_e = Controls(2);

S = FlightData.Geom.S;
b = FlightData.Geom.b;
c = FlightData.Geom.c;

Q = 0.5*rho*V^2;

% Non-dimensionalising
qhat = (q*c)/(2*V);
phat = (p*c)/(2*V);
rhat = (r*c)/(2*V);

alphadot_hat = (alphadot*c)/(2*V);
betadot_hat = (betadot*c)/(2*V);

% Force Coefficients
CY = CY0 + CYb*beta + CYbd*beta_dot + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;
[CL,CD] = WindForces(FlightData,alpha,V,[AngularRates],[States],[Controls]);

% Forces
D = CD*Q*S;
Y = CY*Q*S;
L = CL*Q*S;

% Calcualting moment coefficients
Cl = Cl0 + Clb*beta + Clbd*beta_dot + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;
Cm = Cm0 + Cma*alpha + Cmalpha*alpha_dot + Cmq*qhat + Cmde*delta_e;
Cn = Cn0 + Cnb*beta + Cnbd*beta_dot + Cnr*rhat + Cnp*phat + Cnda*delta_a + Cndr*delta_r;

% Moments
L_moment = Cl*Q*S*b;
M = Cm*Q*S*c;
N = Cn*Q*S*b;

Forces = [-D Y L];
Moments = [L_moment M N]; 

end