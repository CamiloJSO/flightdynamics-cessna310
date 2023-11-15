% simulateAircraft.m
% function to simulate a fixed wing aircraft under given conditions
% usage
%   [t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft)
% where
%   tfinal : final simulation time (s)
%   x0 = [pe;Phi;Vb;omegab] : initial state vector
%   delta = [deltat;deltaf;ih;deltae;deltaa;deltar] : controls vector
%       with
%       deltat : propulsion system control (0<=deltat<=1)
%       deltaf : flap (rad)
%       ih : horizontal tail incidence (rad)
%       deltae : elevator (rad)
%       deltaa : aileron (rad)
%       deltar : rudder (rad)
%       deltaaero = [deltaf;ih;deltae;deltaa;deltar] : aerodynamic controls
%                                                      vector
%   Vwe : wind velocity expressen in earth frame (m/s)
%   deltaCGb : position of aircraft CG respect to nominal CG position
%       expressed in body frame (m)
%   aircraft : aircraft data structure
%   t : simulation times vector
%   X : matrix with simulation results. Row ith of X is x' for t=t(i)
%   Y : matrix with [V,alpha,beta] for each t(i)


function [t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft)
 % initialize Vbdot
 global Vbdot
 Vbdot = [0;0;0];
 
 % set ode45 numerical method parameters
 options = odeset('AbsTol',1e-3,'RelTol',1e-3,'MaxStep',0.01,'InitialStep',0.01);
 
 % solve system of differential equations defined by faircraft function
 % for given aircraft and given conditions
 [t,X] = ode45(@(t,x)faircraft(t,x,delta,Vwe,deltaCGb,aircraft),[0 tfinal],x0,options);

 % initialize Y
 n = length(t);
 Y = zeros(n,3);
 
 % calculate Y, where each row contains values of [V,alpha,beta]
 for i=1:n
     Phi = X(i,4:6)';
     Vb = X(i,7:9)';
     Cbe = DCM(Phi);
     Vrelb = Vb-Cbe*Vwe;
     V = norm(Vrelb);
     alpha = atan(Vrelb(3)/Vrelb(1));
     beta = asin(Vrelb(2)/V);
     Y(i,:) = [V,alpha,beta];
 end
end