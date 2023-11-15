% testTrimConditionsPullup.m
% script to test trimConditionsPullup function to calculate steady
% pullup/pullover flight conditions for a fixed wing aircraft
format long g

% recall aircraft data structure
Navion_aircraft

% steady pullup/pullover flight conditions data
V = aircraft.V;
h = aircraft.h;
pitchRate = 5*pi/180;
deltaCGb = [0;0;0];

[alpha,deltat,ih,deltae,fval,flag] = trimConditionsPullup(V,h,pitchRate,deltaCGb, aircraft);

alphadeg = alpha*180/pi
deltat
ThrustRequired = aircraft.Tmax*deltat
ThrustRequired_lb = ThrustRequired/4.44822
ihdeg = ih*180/pi
deltaedeg = deltae*180/pi
n = loadFactor(V,h,alpha,0,alpha,[0;pitchRate;0],0,ih,deltae,aircraft)
fval
flag