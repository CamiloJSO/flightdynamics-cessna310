% testTrimConditionsRoll.m
% script to test trimConditionsRoll function to calculate steady
% roll flight conditions for a fixed wing aircraft

% recall aircraft data structure
Navion_aircraft

% steady rectilinear flight conditions data
V = aircraft.V;
h = aircraft.h;
rollRate = 5*pi/180;
deltaCGb = [0;0;0];

[theta,alpha,beta,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsRoll(V,h,rollRate,deltaCGb, aircraft);
thetadeg = theta*180/pi
alphadeg = alpha*180/pi
betadeg = beta*180/pi
deltat
ThrustRequired = aircraft.Tmax*deltat
ThrustRequired_lb = ThrustRequired/4.44822
ihdeg = ih*180/pi
deltaedeg = deltae*180/pi
deltaadeg = deltaa*180/pi
deltardeg = deltar*180/pi
n = loadFactor(V,h,alpha,0,theta,[rollRate;0;0],0,ih,deltae,aircraft)
fval
flag