% propulsion.m
% calculate propulsion system forces and moments
% usage
%    [Ftb,Mtb] = propulsion (deltat,deltaCGB,aircraft)
% where 
%  deltat : propulsion system control (0<deltat<1)
%  deltaCGb : position of aircraft CG respect to nominal position 
%             expressed in body frame (m) 
%  aircraft : aircraft data structure 
%  Ftb : propulsion system net force expressed in body frame
%  Mtb : propulsion system net moment expressed in body frame
%

function [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft)
    Ftb = [1;0;0]*aircraft.Tmax*deltat;
    Mtb = -cross(deltaCGb,Ftb);
end