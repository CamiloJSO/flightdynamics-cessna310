% ftrimPullup.m
% function to be minimized to calculate steady pullup/pullover flight
% conditions for a fixed wing aircraft
% usage
%   y = ftrimPullup(Xi,V,h,pitchRate,ihTrimRect,deltaCGb,aircraft)
% where
%   Xi = [alpha;deltat;pitchControl] : vector with variables to be
%        calculated in steady pullup flight condition
%     with
%     alpha : angle of attack (rad)
%     deltat : propulsion system control (0<=deltat<=1)
%     pitchControl : pitch control, it is ih if aircraft.Cmih~=0, or deltae
%                    otherwise (rad)
%   V : airspeed (m/s)
%   h : altitude (m)
%   pitchRate : pitch rate (rad/s)
%   ihTrimRect : value of ih for steady rectilinear flight conditions (rad)
%                (only for aircraft with ih and deltae)
%   deltaCGb : position of aircraft CG respect a nominal CG position
%              expressed in body frame (m)
%   aircraft : aicraft data structure
%   y : value of ftrimPullup
%

function y = ftrimPullup(Xi,V,h,pitchRate,ihTrimRect,deltaCGb,aircraft)
  global Vbdot
  
  % extract components of Xi
  alpha = Xi(1,1);
  deltat = Xi(2,1);
  pitchControl = Xi(3,1);
  
  % define components of state vector x
  pe = [0;0;-h];
  phi = 0;
  theta = alpha;
  psi = 0;
  Phi = [phi;theta;psi];
  beta = 0;
  Vb = [V*cos(alpha)*cos(beta);V*sin(beta);V*sin(alpha)*cos(beta)];
  Phidot = [0;pitchRate;0];
  omegab = Hinv(Phi)*Phidot;
  % assemble state vector x
  x = [pe;Phi;Vb;omegab];
  
  % define components of derivative of state vector xdot
  Cbe = DCM(Phi);
  pedot = Cbe'*Vb;
  Vbdot = [0;0;0];
  omegabdot = [0;0;0];
  
  % assemble derivative of state vector xdot
  xdot = [pedot;Phidot;Vbdot;omegabdot];
  
  % define components of controls vector delta
  deltaf = 0;
  if aircraft.Cmdeltae~=0
      ih = ihTrimRect;
      deltae = pitchControl;
  else
      ih = pitchControl;
      deltae = 0;
  end
  deltaa = 0;
  deltar = 0;
 
  % assemble controls vector delta
  delta = [deltat;deltaf;ih;deltae;deltaa;deltar];
  
  % other variables
  t = 0;
  Vwe = [0;0;0];
  
  % calculate result of ftrimRect
  y = norm(xdot-faircraft(t,x,delta,Vwe,deltaCGb,aircraft))^2+pedot(3)^2;
end


 