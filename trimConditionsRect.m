% trimConditionsRect.m
% function to calculate steady rectilinear flight condition for a fixed
% wing aircraft
% usage
%   [theta,gamma,alpha,deltat,ih,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb, aircraft)
% where
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   deltaCGb : position of aircraft CG respect a nominal CG position
%              expressed in body frame (m)
%   aircraft : aicraft data structure
%   theta : pitch angle in steady rectilinear flight (rad)
%   gamma : flight path angle in steady rectilinear flight (rad)
%   alpha : angle of attack in steady rectilinear flight(rad)
%   deltat : propulsion system control (0<=deltat<=1)
%   ih : horizontal tail incidence in steady rectilinear flight(rad)
%   deltae : elevator (rad)
%   fval : value of ftrimRect in calculated condition (it should be close to zero)
%   flag : code for termination condition of numerical optimization method
%
 
 
 function [theta,gamma,alpha,deltat,ih,deltae,fval,flag] = trimConditionsRect(V,h,Vvert,deltaCGb, aircraft)
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
    
    [Xitrim,fval,flag] = fmincon(@(Xi)ftrimRect(Xi,V,h,Vvert,deltaCGb,aircraft),Xi0,[],[],[],[],lb,ub,[],options);
   
    % extract components of Xitrim
    alpha = Xitrim(1,1);
    deltat = Xitrim(2,1);
    pitchControl = Xitrim(3,1);
    if aircraft.Cmih~=0
      ih = pitchControl;
      deltae = 0;
    else
      ih = 0;
      deltae = pitchControl;
    end
    
    % generate results
    gamma = asin(Vvert/V);
    theta = gamma+alpha;
 end