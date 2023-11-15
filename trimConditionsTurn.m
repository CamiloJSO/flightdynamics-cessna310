% trimConditionsTurn.m
% function to calculate steady turn flight condition for a fixed
% wing aircraft
% usage
%   [phi,theta,gamma,alpha,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsTurn(V,h,Vvert,turnRate,deltaCGb, aircraft)
% where
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   turnRate : turn rate (rad/s)
%   deltaCGb : position of aircraft CG respect a nominal CG position
%              expressed in body frame (m)
%   aircraft : aicraft data structure
%   phi : roll angle (rad) in steady turn flight
%   theta : pitch angle in steady turn flight (rad)
%   gamma : flight path angle in steady turn flight (rad)
%   alpha : angle of attack in steady turn flight(rad)
%   deltat : propulsion system control (0<=deltat<=1)
%   ih : horizontal tail incidence in steady turn flight(rad)
%   deltae : elevator in steady turn flight(rad)
%   deltaa : aileron in steady turn flight(rad)
%   deltar : rudder in steady turn flight(rad)
%   fval : value of ftrimRect in calculated condition (it should be close to zero)
%   flag : code for termination condition of numerical optimization method
%
 
 
 function [phi,theta,gamma,alpha,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsTurn(V,h,Vvert,turnRate,deltaCGb, aircraft)
    % calculate steady rectilinear flight conditions ibn case the aircraft
    % has horizontal tail incidence and elevator controls
    if aircraft.Cmih ~= 0 && aircraft.Cmdeltae~=0
        [theta,gamma,alpha,deltat,ihTrimRect,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb, aircraft)
    else
        gamma = asin(Vvert/V);
        ihTrimRect = 0;
    end
    
    % set initial value for Xi
    phi0 = atan(V*turnRate/aircraft.g);
    theta0 = gamma;
    alpha0 = 0;
    deltat0 = 0.5;
    pitchControl0 = 0; 
    deltaa0 = 0;
    deltar0 = 0;
    Xi0 = [phi0;theta0;alpha0;deltat0;pitchControl0;deltaa0;deltar0];
    % set lower bounds
    phimin = -60*pi/180;
    thetamin = -15*pi/180;
    alphamin = -5*pi/180;
    deltatmin = 0;
    pitchControlmin = -20*pi/180;
    deltaamin = -20*pi/180;
    deltarmin = -20*pi/180;
    lb = [phimin;thetamin;alphamin;deltatmin;pitchControlmin;deltaamin;deltarmin];
    % set upper bounds
    phimax = 60*pi/180;
    thetamax = 22*pi/180;
    alphamax = 12*pi/180;
    deltatmax = 1;
    pitchControlmax = 20*pi/180;
    deltaamax = 20*pi/180;
    deltarmax = 20*pi/180;
    ub = [phimax;thetamax;alphamax;deltatmax;pitchControlmax;deltaamax;deltarmax];
    
   
    maxiter = 100000;
    tol = 1e-9;
    options = optimoptions('fmincon','Display','off','Algorithm','sqp',...
        'MaxIter',maxiter, 'TolX',tol,'TolFun',tol);
    
    [Xitrim,fval,flag] = fmincon(@(Xi)ftrimTurn(Xi,V,h,Vvert,turnRate,ihTrimRect,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
   
    % extract components of Xitrim
    phi = Xitrim(1,1);
    theta = Xitrim(2,1);
    alpha = Xitrim(3,1);
    deltat = Xitrim(4,1);
    pitchControl = Xitrim(5,1);
    if aircraft.Cmdeltae~=0
      ih = ihTrimRect;
      deltae = pitchControl;
    else
      ih = pitchControl;
      deltae = 0;
    end
    deltaa = Xitrim(6,1);
    deltar = Xitrim(7,1);
 end