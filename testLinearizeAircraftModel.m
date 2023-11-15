% testLinearizeAircraftModel.m
% script to test linearizeAircraftModel function to calculate linearized
% model for a fixed wing aircraft

% recall aircraft data structure
Navion_aircraft

V = aircraft.V
h = aircraft.h
deltaCGb = [0;0;0]

[E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0] = linearizeAircraftModel(V,h,deltaCGb,aircraft)

