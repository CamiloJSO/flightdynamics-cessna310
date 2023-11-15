% testTrimConditionsRect.m
% script to test trimConditionsRect function to calculate steady
% rectiliniar flight conditions for a fixed wing aircraft

% recall aircraft data structure
Navion_aircraft

% steady rectilinear flight conditions data
V = aircraft.V;
h = aircraft.h;
Vvert = 0*0.3048/60;
deltaCGb = [0;0;0];

[theta,gamma,alpha,deltat,ih,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb, aircraft);

thetadeg = theta*180/pi
gammadeg = gamma*180/pi
alphadeg = alpha*180/pi
deltat
ThrustRequired = aircraft.Tmax*deltat
ThrustRequired_lb = ThrustRequired/4.44822
ihdeg = ih*180/pi
deltaedeg = deltae*180/pi
fval
flag