%
% Aircraft: Cessna 310
% Flight condition: Approach
% Altitude (ft): 0
% Airspeed (kn): 82
% Mach number: 0.124
% Script name: Navion_aircraft.m
%
% Data source:
% Seckel, E. M. J. J. (1971), 'The stability derivatives of the Navion aircraft estimated by various methods
%   and derived from flight test data' (FAA-RD-7106), Technical report, Department of Transportation, Federal
%   Aviation Administration.
%
% Author:
%   Camilo José Sierra Otero
%   camiloj.sierra@udea.edu.co
%
% aircraft data
aircraft.aircraftName = 'Cessna 310';
aircraft.flightCondition = 'Approach';
% aircraft flight contitions data;
aircraft.h = 0*0.3048;
aircraft.V = 82*1852/3600;
aircraft.g = 9.80665;
% aircraft geometry
aircraft.S = 175*0.3048^2;
aircraft.b = 36.9*0.3048;
aircraft.cbar = 4.79*0.3048;
aircraft.A = aircraft.b^2/aircraft.S;
% aircraft aerodynamic parameters
aircraft.M0 = 0.124;
aircraft.CD0 = 0.0811;
aircraft.CLmindrag = -0.1032;
aircraft.e = 0.73;
CDu = 0;
aircraft.CDM = CDu/aircraft.M0;
CYbeta = -0.577;
CYdeltaa = 0;
CYdeltar = 0.230;
CYp = -0.2897;
CYr = 0.355;
aircraft.CCbeta = -CYbeta;
aircraft.CCdeltaa = -CYdeltaa;
aircraft.CCdeltar = -CYdeltar;
aircraft.CCp = -CYp;
aircraft.CCr = -CYr;
aircraft.CL0 = 0.640;
aircraft.CLalpha = 4.58;
aircraft.CLdeltaf = 0;
aircraft.CLih = 0;
aircraft.CLdeltae = 0.77;
aircraft.CLalphadot = 4.1;
aircraft.CLq = 8.4;
CLu = 0;
aircraft.CLM = CLu/aircraft.M0;
aircraft.Clbeta = -0.0965;
aircraft.Cldeltaa = -0.1720;
aircraft.Cldeltar = 0.0192;
aircraft.Clp = -0.566;
aircraft.Clr = 0.2433;
aircraft.Cm0 = 0.10;
aircraft.Cmalpha = -0.619;
aircraft.Cmdeltaf = 0;
aircraft.Cmih = 0;
aircraft.Cmdeltae = -2.16;
aircraft.Cmalphadot = -11.4;
aircraft.Cmq = -25.1;
Cmu = 0;
aircraft.CmM = Cmu/aircraft.M0;
aircraft.Cnbeta = 0.1683;
aircraft.Cndeltaa = 0.0676;
aircraft.Cndeltar = -0.1152;
aircraft.Cnp = -0.1021;
aircraft.Cnr = -0.1947;
% aircraft mass and inertia parameters
aircraft.m = 4600*4.448222/aircraft.g;
aircraft.Ix = 8884*14.593903*0.3048^2;
aircraft.Iy = 1939*14.593903*0.3048^2;
aircraft.Iz = 11001*14.593903*0.3048^2;
aircraft.Ixz = 0*14.593903*0.3048^2;
aircraft.Ib = [aircraft.Ix           0        -aircraft.Ixz;
                       0          aircraft.Iy         0;
                -aircraft.Ixz         0         aircraft.Iz];
aircraft.Ibinv = inv(aircraft.Ib);
% aircraft propulsion system parameters
aircraft.Tmax = 2*1100*4.448222;
