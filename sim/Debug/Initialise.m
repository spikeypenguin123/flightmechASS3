function [X0 U0 FlightData] = Initialise()
fprintf('===PC9 CG2 @ 100kts @ 1000ft=== \n\n')
fprintf('===DEBUG DATA with X0+0.1 & U0+0.1=== \n\n')


FlightData = aero3560_LoadFlightDataPC9_nominalCG1;
load ICs_PC9_CG2_100Kn_1000ft