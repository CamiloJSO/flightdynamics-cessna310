% trimConditionsRoll.m
% function to calculate steady roll flight condition for a fixed
% wing aircraft
% usage
%   [theta,alpha,beta,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsRoll(V,h,rollRate,deltaCGb, aircraft)
% where
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   rollRate : roll rate (rad/s)
%   deltaCGb : position of aircraft CG respect a nominal CG position
%              expressed in body frame (m)
%   aircraft : aicraft data structure
%   theta : pitch angle in steady roll flight (rad)
%   alpha : angle of attack in steady roll flight(rad)
%   beta : angle of sideslip in steady roll flight(rad)
%   deltat : propulsion system control (0<=deltat<=1)
%   ih : horizontal tail incidence in steady roll flight(rad)
%   deltae : elevator in steady roll flight(rad)
%   deltaa : aileron in steady roll flight(rad)
%   deltar : rudder in steady roll flight(rad)
%   fval : value of ftrimRect in calculated condition (it should be close to zero)
%   flag : code for termination condition of numerical optimization method
%
 
 
 function [theta,alpha,beta,deltat,ih,deltae,deltaa,deltar,fval,flag] = trimConditionsRoll(V,h,rollRate,deltaCGb, aircraft)
    % calculate steady rectilinear flight conditions ibn case the aircraft
    % has horizontal tail incidence and elevator controls
    if aircraft.Cmih ~= 0 && aircraft.Cmdeltae~=0
        [theta,gamma,alpha,deltat,ihTrimRect,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb, aircraft)
    else
        ihTrimRect = 0;
    end
    
    % set initial value for Xi
    theta0 = 0;
    alpha0 = 0;
    beta0 = 0;
    deltat0 = 0.5;
    pitchControl0 = 0; 
    deltaa0 = 0;
    deltar0 = 0;
    Xi0 = [theta0;alpha0;beta0;deltat0;pitchControl0;deltaa0;deltar0];
    % set lower bounds
    thetamin = -15*pi/180;
    alphamin = -5*pi/180;
    betamin = -10*pi/180;
    deltatmin = 0;
    pitchControlmin = -20*pi/180;
    deltaamin = -20*pi/180;
    deltarmin = -20*pi/180;
    lb = [thetamin;alphamin;betamin;deltatmin;pitchControlmin;deltaamin;deltarmin];
    % set upper bounds
    thetamax = 22*pi/180;
    alphamax = 12*pi/180;
    betamax = 10*pi/180;
    deltatmax = 1;
    pitchControlmax = 20*pi/180;
    deltaamax = 20*pi/180;
    deltarmax = 20*pi/180;
    ub = [thetamax;alphamax;betamax;deltatmax;pitchControlmax;deltaamax;deltarmax];
    
   
    maxiter = 100000;
    tol = 1e-9;
    options = optimoptions('fmincon','Display','off','Algorithm','sqp',...
        'MaxIter',maxiter, 'TolX',tol,'TolFun',tol);
    
    [Xitrim,fval,flag] = fmincon(@(Xi)ftrimRoll(Xi,V,h,rollRate,ihTrimRect,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
   
    % extract components of Xitrim
    theta = Xitrim(1,1);
    alpha = Xitrim(2,1);
    beta = Xitrim(3,1);
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