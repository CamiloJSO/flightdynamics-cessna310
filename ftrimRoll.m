% ftrimRoll.m
% function to be minimized to calculate steady Roll flight
% conditions for a fixed wing aircraft
% usage
%   y = ftrimRoll(Xi,V,h,rollRate,ihTrimRect,deltaCGb,aircraft)
% where
%   Xi = [theta;alpha;beta;deltat;pitchControl,deltaa,deltar] : vector with variables to be
%        calculated in steady roll flight condition
%     with
%     phi : roll (rad)
%     theta : pitch (rad)
%     alpha : angle of attack (rad)
%     beta : angle of sideslip (rad)
%     deltat : propulsion system control (0<=deltat<=1)
%     pitchControl : pitch control, it is ih if aircraft.Cmih~=0, or deltae
%                    otherwise (rad)
%     deltaa : aileron (rad)
%     deltar : rudder (rad)
%   V : airspeed (m/s)
%   h : altitude (m)
%   rollRate : roll rate (rad/s)
%   ihTrimRect : value of ih for steady roll flight conditions (rad)
%                (only for aircraft with ih and deltae)
%   deltaCGb : position of aircraft CG respect a nominal CG position
%              expressed in body frame (m)
%   aircraft : aicraft data structure
%   y : value of ftrimRoll
%  

function y = ftrimRoll(Xi,V,h,rollRate,ihTrimRect,deltaCGb,aircraft)
  global Vbdot
  
  % extract components of Xi
  theta = Xi(1,1);
  alpha = Xi(2,1);
  beta = Xi(3,1);
  deltat = Xi(4,1);
  pitchControl = Xi(5,1);
  deltaa = Xi(6,1);
  deltar = Xi(7,1);
  
  % define components of state vector x
  pe = [0;0;-h];
  phi = 0;
  psi = 0;
  Phi = [phi;theta;psi];
  Vb = [V*cos(alpha)*cos(beta);V*sin(beta);V*sin(alpha)*cos(beta)];
  Phidot = [rollRate;0;0];
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

  % assemble controls vector delta
  delta = [deltat;deltaf;ih;deltae;deltaa;deltar];
  
  % other variables
  t = 0;
  Vwe = [0;0;0];
  
  % calculate result of ftrimRect
  y = norm(xdot-faircraft(t,x,delta,Vwe,deltaCGb,aircraft))^2+pedot(3)^2;
end


 