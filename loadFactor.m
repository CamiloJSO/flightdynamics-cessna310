% loadFactor.m
% function to calculate load factor for a fixed wing aircraft

function n = loadFactor(V,h,alpha,phi,theta,Phidot,deltaf,ih,deltae,aircraft)
 % calculate atmosphere parameters based on standard atmosphere model
 [rho,P,T,a] = atmosphere(h);
 % calculate dynamic pressure
 qbar = rho*V^2/2;
 % calculate Mach number
 M = V/a;
 % calculate angular velocity expressed in body frame
 Phi = [phi;theta;0];
 omegab = Hinv(Phi)*Phidot;
 q = omegab(2);
 alphadot = 0;
 % calculate lift coefficient
 CL = aircraft.CL0+aircraft.CLalpha*alpha+aircraft.CLdeltaf*deltaf+aircraft.CLih*ih+...
      aircraft.CLdeltae*deltae+aircraft.cbar/(2*V)*(aircraft.CLalphadot*alphadot+...
      aircraft.CLq*q)+aircraft.CLM*(M-aircraft.M0);
 % calculate lift
 L = qbar*aircraft.S*CL;
 % calculate load factor
 n = L/(aircraft.m*aircraft.g);
end
