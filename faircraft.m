% faircraft.m
% Function to define the model for a fixed wing aircraft such that
%  xdot = faircraft(t, x, delta, Vwe, deltaCGb, aircraft)
% Usage
%  [xdot, y] = faircraft(t, x, delta, Vwe, deltaCGb, aircraft)
% where
%   t : time (s).
%   x = [pe;Phi;Vb;omegab] : state vector
%       with
%       pe = [xe;ye;ze] : position of aircraft CG respect to earth
%                         expressed in earth frame (m).
%       Phi = [phi;theta;psi] : euler angles (rad).
%       Vb = [u;v;w] : aircraft velocity respect to earth expressed in body
%                      frame (m/s).
%       omegab = [p;q;r] : aircraft angular velocity respect to earth
%                          expressed in body frame (rad/s).
%   delta = [deltat;deltaf;ih;deltae;deltaa;deltar] : controls vector.
%       with
%       deltat : propulsion system control (0<=deltat<=1).
%       deltaf : flap (rad).
%       ih : horizontal tail incidence (rad).
%       deltae : elevator (rad).
%       deltaa : aileron (rad).
%       deltar : rudder (rad).
%       deltaaero = [deltaf;ih;deltae;deltaa;deltar] : aerodynamic controls
%                                                      vector.
%   Vwe : wind velocity expressed in earth frame (m/s).
%   deltaCGb : position of aircraft CG respect nominal CG position 
%              expressed in body frame(m).
%   aircraft : aircraft data structure.
%   xdot = [pedot;Phidot;Vbdot;omegabdot] : derivative of state vector
%       with
%       pedot = [xe;ye;ze] = Ve : derivative of position of aircraft CG 
%                            respect to earth expressed in earth frame (m/s).
%       Phidot = [phi;theta;psi] : derivative of euler angles (rad/s).
%       Vbdot = [u;v;w] : derivative of aircraft velocity respect to earth 
%                         expressed in body frame (m/s^2).
%       omegabdot = [p;q;r] : derivative of aircraft angular velocity 
%                          respect to earth expressed in body frame (rad/s^2).
%   y = [V;alpha;beta] : where V is the airspeed (m/s), alpha is the angle
%                        of attack (rad) and beta is the angle of sideslip (rad).

function [xdot,y] = faircraft(t,x,delta,Vwe,deltaCGb,aircraft)
 global Vbdot
 % extract components of x and delta
 pe = x(1:3,1);
 h = -pe(3,1);
 Phi = x(4:6,1);
 Vb = x(7:9,1);
 omegab = x(10:12,1);
 deltat = delta(1,1);
 deltaaero = delta(2:6,1);
 
 % transational kinematics
 Cbe = DCM(Phi); %Direction Cosine Matrix
 pedot = Cbe'*Vb;
 
 % rotational kinematics
 Phidot = H(Phi)*omegab;
 
 % calculate forces and moments
 % calculate weight
 Ge = [0;0;aircraft.g];
 Gb = Cbe*Ge;
 Wb = aircraft.m*Gb;
 
 % calculate aerodynamic forces and moments
 % calculate relative velocity
 Vrelb = Vb-Cbe*Vwe;
 % calculate airspeed
 V = norm(Vrelb);
 % calculate angle of attack
 alpha = atan(Vrelb(3)/Vrelb(1));
 % calculate angle of sideslip
 beta = asin(Vrelb(2)/V);
 % calculate alphadot
 Vrelbdot = Vbdot+cross(omegab,Cbe*Vwe);
 alphadot = (Vrelbdot(3)*Vrelb(1)-Vrelbdot(1)*Vrelb(3))/(Vrelb(1)^2+Vrelb(3)^2);
 
 % calculate atmosphere parameters based on standard atmosphere model
 [rho,P,T,a] = atmosphere(h);
 
 % calculate dynamic pressure
 qbar = rho*V^2/2;
 
 % calculate Mach number
 M = V/a;
 
 % calculate aerodynamic forces and moments
 [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,omegab,deltaaero,qbar,M,deltaCGb,aircraft);
 
 % calculate propulsion system forces and moments
 [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft);
 
 % calculate net force expressed in body frame
 Fnetb = Wb+Fab+Ftb;
 
 % evaluate translational dynamics
 Vbdot = Fnetb/aircraft.m-cross(omegab,Vb);
 
 % calculate net moment expressed in body frame
 Mnetb = Mab+Mtb;
 
 % evaluate rotational dynamics
 omegabdot = aircraft.Ibinv*(Mnetb-cross(omegab,aircraft.Ib*omegab));
 
 % assemble derivative of state vector xdot
 xdot = [pedot;Phidot;Vbdot;omegabdot];
 
 % assemble y
 y = [V;alpha;beta];
end