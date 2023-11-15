%
% Aircraft: Navion
% Flight condition: Cruise
% Altitude (ft): 10000
% Airspeed (kn): 135
% Mach number: 0.211
% Script name: Navion_aircraft.m
%
% Data source:
% Seckel, E. M. J. J. (1971), 'The stability derivatives of the Navion aircraft estimated by various methods
%   and derived from flight test data' (FAA-RD-7106), Technical report, Department of Transportation, Federal
%   Aviation Administration.
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%
% aircraft data
aircraft.aircraftName = 'Navion';
aircraft.flightCondition = 'Cruise';
% aircraft flight contitions data;
aircraft.h = 10000*0.3048;
aircraft.V = 135*1852/3600;
aircraft.g = 9.80665;
% aircraft geometry
aircraft.S = 184*0.3048^2;
aircraft.b = 33.38*0.3048;
aircraft.cbar = 5.7*0.3048;
aircraft.A = aircraft.b^2/aircraft.S;
% aircraft aerodynamic parameters
aircraft.M0 = 0.211;
aircraft.CD0 = 0.04;
aircraft.CLmindrag = 0.3;
aircraft.e = 0.77;
CDu = 0;
aircraft.CDM = CDu/aircraft.M0;
CYbeta = -0.61;
CYdeltaa = 0;
CYdeltar = 0.157;
CYp = 0;
CYr = 0;
aircraft.CCbeta = -CYbeta;
aircraft.CCdeltaa = -CYdeltaa;
aircraft.CCdeltar = -CYdeltar;
aircraft.CCp = -CYp;
aircraft.CCr = -CYr;
aircraft.CL0 = 0.3;
aircraft.CLalpha = 6.04;
aircraft.CLdeltaf = 0;
aircraft.CLih = 0;
aircraft.CLdeltae = 0.355;
aircraft.CLalphadot = 0;
aircraft.CLq = 3.8;
CLu = 0;
aircraft.CLM = CLu/aircraft.M0;
aircraft.Clbeta = -0.067;
aircraft.Cldeltaa = -0.152;
aircraft.Cldeltar = 0.107;
aircraft.Clp = -0.46;
aircraft.Clr = 0.069;
aircraft.Cm0 = 0;
aircraft.Cmalpha = -0.715;
aircraft.Cmdeltaf = 0;
aircraft.Cmih = 0;
aircraft.Cmdeltae = -1.42;
aircraft.Cmalphadot = -4.91;
aircraft.Cmq = -13.39;
Cmu = 0;
aircraft.CmM = Cmu/aircraft.M0;
aircraft.Cnbeta = 0.086;
aircraft.Cndeltaa = 0.0047;
aircraft.Cndeltar = -0.075;
aircraft.Cnp = -0.038;
aircraft.Cnr = -0.088;
% aircraft mass and inertia parameters
aircraft.m = 2948*4.448222/aircraft.g;
aircraft.Ix = 1284*14.593903*0.3048^2;
aircraft.Iy = 2773*14.593903*0.3048^2;
aircraft.Iz = 3235*14.593903*0.3048^2;
aircraft.Ixz = 0*14.593903*0.3048^2;
aircraft.Ib = [aircraft.Ix           0        -aircraft.Ixz;
                       0          aircraft.Iy         0;
                -aircraft.Ixz         0         aircraft.Iz];
aircraft.Ibinv = inv(aircraft.Ib);
% aircraft propulsion system parameters
aircraft.Tmax = 1000*4.448222;
