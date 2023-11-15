% trimConditionsPullup.m
% function to calculate steady pullup/pullover flight condition for a fixed
% wing aircraft
% usage
%   [alpha,deltat,ih,deltae,fval,flag] = trimConditionsPullup(V,h,pitchRate,deltaCGb, aircraft)
% where
%   V : airspeed (m/s)
%   h : altitude (m)
%   pitchRate : pitch rate (rad/s)
%   deltaCGb : position of aircraft CG respect a nominal CG position
%              expressed in body frame (m)
%   aircraft : aicraft data structure
%   alpha : angle of attack in steady pullup/pullover flight(rad)
%   deltat : propulsion system control (0<=deltat<=1)
%   ih : horizontal tail incidence in steady pullup/pullover flight(rad)
%   deltae : elevator (rad)
%   fval : value of ftrimRect in calculated condition (it should be close to zero)
%   flag : code for termination condition of numerical optimization method
%
 
 
 function [alpha,deltat,ih,deltae,fval,flag] = trimConditionsPullup(V,h,pitchRate,deltaCGb, aircraft)
    % calculate steady pullup/pullover flight conditions ibn case the aircraft
    % has horizontal tail incidence and elevator controls
    if aircraft.Cmih ~= 0 && aircraft.Cmdeltae~=0
        [theta,gamma,alpha,deltat,ihTrimRect,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb, aircraft)
    else
        ihTrimRect = 0;
    end
    % set initial value for Xi
    alpha0 = 0;
    deltat0 = 0.5;
    pitchControl0 = 0; 
    Xi0 = [alpha0;deltat0;pitchControl0];
    % set lower bounds
    alphamin = -5*pi/180;
    deltatmin = 0;
    pitchControlmin = -20*pi/180;
    lb = [alphamin;deltatmin;pitchControlmin];
    % set upper bounds
    alphamax = 12*pi/180;
    deltatmax = 1;
    pitchControlmax = 20*pi/180;
    ub = [alphamax;deltatmax;pitchControlmax];
    
    maxiter = 100000;
    tol = 1e-9;
    options = optimoptions('fmincon','Display','off','Algorithm','sqp',...
        'MaxIter',maxiter, 'TolX',tol,'TolFun',tol);
    
    [Xitrim,fval,flag] = fmincon(@(Xi)ftrimPullup(Xi,V,h,pitchRate,ihTrimRect,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
   
    % extract components of Xitrim
    alpha = Xitrim(1,1);
    deltat = Xitrim(2,1);
    pitchControl = Xitrim(3,1);
    if aircraft.Cmdeltae~=0
      ih = ihTrimRect;
      deltae = pitchControl;
    else
      ih = pitchControl;
      deltae = 0;
    end
 end