% f.m
% fuction to define the model for a fixed wing aircraft such that
%    f(xdot,x,u,h,deltaCGb,aircraft)
% usage
%    y = f(xdot,x,u,h,deltaCGb,aircraft)
% where
%    xdot = [Vdot;betadot,;alfadot;Phidot;omegasdot] : derivative of state vector
%       with
%       Vdot = derivative of airspeed (m/s)
%       betadot : derivative of angle of sideslip (rad/s)
%       alphadot : derivative of angle of attack (rad/s)
%       Phidot = [phidot;thetadot;psidot] : derivative of Euler angles 
%                                           (rad/s)
%       omegasdot = [psdot;qdot;rsdot] : derivative of aircraft angular 
%                                      velocity respect to earth 
%                                      expressed in stability frame (rad/s^2)
%    x = [V;beta,;alpha;Phi;omegas] : state vector
%       with
%       V : airspeed (m/s)
%       beta : angle of sideslip (rad)
%       alpha : angle of attack (rad)
%       Phi = [phi;theta;psi] : Euler angles (rad)
%       omegas = [ps;q;rs] : aircraft angular velocity respect to earth 
%                            expressed in stability frame (rad/s) 
%    u = [deltat;ih;deltae;deltaa;deltar] : controls vector
%       with
%       deltat : propulsion system control (0<=deltat<=1)
%       ih : horizontal incidence (rad)
%       deltae : elevator (rad)
%       deltaa : airleron (rad)
%       deltar : rudder (rad)
%    h : altitude (m)
%    deltaCGb : position of aircraft CG respect to nominal CG position
%               expressed in body frame (m)
%    aircraft : aircraft data structure
%    y : value of f(xdot,x,u,h,deltaCGb,aircraft)

function y = f(xdot,x,u,h,deltaCGb,aircraft)
    % extract components of xdot, x and u
    Vdot = xdot(1,1);
    betadot = xdot(2,1);
    alphadot = xdot(3,1);
    Phidot = xdot(4:6,1);
    omegasdot = xdot(7:9,1);

    V = x(1,1);
    beta = x(2,1);
    alpha =  x(3,1);
    Phi = x(4:6,1);
    omegas = x(7:9,1);

    deltat = u(1,1);
    deltaaero = [0;u(2:5,1)];
    
    % rotational kinematics
    Cbs = Cbsmatrix(alpha);
    omegab = Cbs*omegas;
    y2 = -Phidot + H(Phi)*omegab;
    
    % calculate forces and moments
    % calculate weight
    Ge = [0;0;aircraft.g];
    Cbe = DCM(Phi);
    Gb = Cbe*Ge;
    Cbw = Cbwmatrix(alpha, beta);
    Gw = Cbw'*Gb;
    Ww = aircraft.m*Gw;
    
    % calculate atmosphere parameters based on standard atmosphere model
    [rho,P,T,a] = atmosphere(h);
    
    % calculate dynamic pressure
    qbar = rho*V^2/2;
    
    % calculate Mach number
    M = V/a;
    
    % calculate aerodynamic forces and moments
    [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,omegab,deltaaero,qbar,M,deltaCGb,aircraft);
    Faw = Cbw'*Fab;
    Mas = Cbs'*Mab;
    
    % calculate propulsin system forces and moments
    [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft);
    Ftw = Cbw'*Ftb;
    Mts = Cbs'*Mtb;
    
    % calculate net force expressed in relative wind frame
    Fnetw = Ww + Faw + Ftw;
    
    % calculate net moment expressed in stability frame
    Mnets = Mas + Mts;
    
    % calculate other required terms
    Vrelwdot = [Vdot;0;0];
    Vrelw = [V;0;0];
    omega_w_b_w = [-sin(beta)*alphadot;-cos(beta)*alphadot;betadot];
    omegaw = Cbw'*omegab;
    omega_s_b_s = [0;-alphadot;0];
    Is = Cbs'*aircraft.Ib*Cbs;
    Isinv = Cbs'*aircraft.Ibinv*Cbs;
    
    % translational dynamics expressed in relative wind frame
    y1 = -aircraft.m*(Vrelwdot + cross(omega_w_b_w,Vrelw) + Fnetw - aircraft.m*cross(omegaw,Vrelw));

    % rotational dynamics expressed in stability frame
    y3 = -omegasdot - cross(omega_s_b_s,omegas) + Isinv*(Mnets - cross(omegas,Is*omegas));

    % assemble y
    y = [y1;y2;y3];
    
end