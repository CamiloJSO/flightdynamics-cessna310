%   aerodynamics.m
% function to calculate aerodynamic forces and moments
% usage
%   [Fab, Mab] = aerodynamics(V,alpha,beta,alphadot,omegab,deltaaero,qbar,M,deltaCGb,aircraft);
% where
%   V : airspeed (m/s)
%   alpha: angle of attack (rad)
%   beta : angle of sideslip (rad)
%   alphadot : derivative of angle of attack (rad/s)
%   omegabdot = [p;q;r] : derivative of aircraft angular velocity 
%                         respect to earth expressed in body frame (rad/s^2).
% deltaaero = [deltaf;ih;deltae;deltar] : aerodynamics controls vector
%   with
%       deltaf : flap (rad).
%       ih : horizontal tail incidence (rad).
%       deltae : elevator (rad).
%       deltaa : aileron (rad).
%       deltar : rudder (rad).
%       deltaaero = [deltaf;ih;deltae;deltaa;deltar] : aerodynamic controls
%                                                      vector.
%   qbar: dynamic pressure (Pa)
%  M : Mach number
%       deltaCGb : position of aircraft CG respect a nominal CG position
%                  expressed in body frame.
%       aircraft: aircraft data structure
%   Fab : net aerodybamic force expressed in body frame (N)
%   Mab : net aerodynamic moment expressed in body frame (Nm)

 function [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,omegab,deltaaero,qbar,M,deltaCGb,aircraft)
 % extract components of omegab and deltaaero
 p = omegab(1,1);
 q = omegab(2,1);
 r = omegab(3,1);
 deltaf = deltaaero(1,1);
 ih = deltaaero(2,1);
 deltae = deltaaero(3,1);
 deltaa = deltaaero(4,1);
 deltar = deltaaero(5,1);
 
 %calculate dimensionless aerodynamic force coefficients
 CL = aircraft.CL0+aircraft.CLalpha*alpha+aircraft.CLdeltaf*deltaf+aircraft.CLih*ih+...
      aircraft.CLdeltae*deltae+aircraft.cbar/(2*V)*(aircraft.CLalphadot*alphadot+...
      aircraft.CLq*q)+aircraft.CLM*(M-aircraft.M0);
 CD = aircraft.CD0+(CL-aircraft.CLmindrag)^2/(pi*aircraft.A*aircraft.e)+...
      aircraft.CDM*(M-aircraft.M0);
 CC = aircraft.CCbeta*beta+aircraft.CCdeltaa*deltaa+aircraft.CCdeltar*deltar+...
      aircraft.b/(2*V)*(aircraft.CCp*p+aircraft.CCr*r);
 
 % denormalize aerodynamic forces
 L = qbar*aircraft.S*CL;
 D = qbar*aircraft.S*CD;
 C = qbar*aircraft.S*CC;
 
 % assemble aerodynamic force expressed in relative wind frame
 Faw = [-D;-C;-L];
 
 % calculate aerodynamic force expressed in body frame
 Cbw = Cbwmatrix(alpha,beta);
 Fab = Cbw*Faw;
 
 % calculate dimensionless aerodynamic moment coefficients
 Cl = aircraft.Clbeta*beta+aircraft.Cldeltaa*deltaa+aircraft.Cldeltar*deltar+...
      aircraft.b/(2*V)*(aircraft.Clp*p+aircraft.Clr*r);
 Cm = aircraft.Cm0+aircraft.Cmalpha*alpha+aircraft.Cmdeltaf*deltaf+aircraft.Cmih*ih+...
      aircraft.Cmdeltae*deltae+aircraft.cbar/(2*V)*(aircraft.Cmalphadot*alphadot+...
      aircraft.Cmq*q)+aircraft.CmM*(M-aircraft.M0);
 Cn = aircraft.Cnbeta*beta+aircraft.Cndeltaa*deltaa+aircraft.Cndeltar*deltar+...
      aircraft.b/(2*V)*(aircraft.Cnp*p+aircraft.Cnr*r);
  
 % denormalize aerodynamic moments
 l = qbar*aircraft.S*aircraft.b*Cl;
 m = qbar*aircraft.S*aircraft.cbar*Cm;
 n = qbar*aircraft.S*aircraft.b*Cn;
 
 % assemble aerodynamic moment expressed in body frame
 Mab = [l;m;n]-cross(deltaCGb,Fab);
  
  
  
  
 end
