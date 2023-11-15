%linearizeAircraftModel.m
% function to linearize aircraft model for a fixed wing aircraft
% usage:
%   [E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0]...
%   = linearizeAircraftModel(V,h,deltaCGb,aircraft)
%   % where
%   V : airspeed (m/s)
%   h : altitude (m)
%   Vvert : vertical velocity (m/s)
%   deltaCGb : position of aircraft CG respect a nominal CG position
%              expressed in body frame (m)
%   aircraft : aicraft data structure
%   E,A,B : matrices of the model, such that E xtildedot = A xtilde + B utilde
%   Elon,Alon,Blon = matrices of the longitudinal linearized model, such 
%                    that Elon xlontildedot = Alon xlontilde + Blon ulontilde
%   Alonprime,Blonprime :  matrices of the longitudinal linerized model,
%                          such that xlontildedot = Alonprime xlontilde +
%                          Blonprime ulontilde
%   Elat,Alat,Blat = matrices of the lateral linearized model, such 
%                    that Elat xlattildedot = Alat xlattilde + Blat ulattilde
%   Alatprime,Blatprime :  matrices of the lateral linerized model,
%                          such that xlattildedot = Alatprime xlattilde +
%                          Blatprime ulattilde
%   eigLon : longitudinal eigenvalues
%   eigLat : lateral eigenvalues
%   x0 : state vector in steady level rectilinear flight
%   u0 : controls vector in steady level rectilinear flight
%

function [E,A,B,Elon,Alon,Blon,Alonprime,Blonprime,Elat,Alat,Blat,Alatprime,Blatprime,eigLon,eigLat,x0,u0] = linearizeAircraftModel(V,h,deltaCGb,aircraft)
    % calculate steady rectilinear flight conditions
    [theta,gamma,alpha,deltat,ih,deltae,fval,flag] = trimConditionsRect(V,h,0,deltaCGb, aircraft);
    % define derivative of state vector in steady level rectilinear flight conditions
    xdot0 = [0;0;0;0;0;0;0;0;0];
    % define state vector in steady level rectilinear flight conditions
    x0 = [V;0;alpha;0;theta;0;0;0;0];
    % define controls vector in steady level rectilinear flight
    u0 = [deltat;ih;deltae;0;0];
    
    % calculate f in this condition
    y0 = f(xdot0,x0,u0,h,deltaCGb, aircraft);
    
    % allocate memory for E,A,B
    E = zeros(9);
    A = zeros(9);
    B = zeros(9,5);
    dx = 1e-6;
    for j = 1:9
        delta = zeros(9,1);
        delta(j) = dx;
        E(:,j) = -(f(xdot0+delta,x0,u0,h,deltaCGb, aircraft)-y0)/dx;
        A(:,j) = -(f(xdot0+delta,u0,h,deltaCGb, aircraft)-y0)/dx;
    end
    for j=1:5
        delta = zeros(5,1);
        delta(j) = dx;
        B(:,j) = -(f(xdot0,x0,u0+delta,h,deltaCGb, aircraft)-y0)/dx;
    end
    
    % extract matrices for linearized longitudinal model
    Elon = [E(1,:);E(3,:);E(5,:);E(8,:)];
    Elon = [Elon(:,1),Elon(:,3),Elon(:,5),Elon(:,8)];
    Alon = [A(1,:);A(3,:);A(5,:);A(8,:)];
    Alon = [Alon(:,1),Alon(:,3),Alon(:,5),Alon(:,8)];
    Blon = [B(1,:);B(3,:);B(5,:);B(8,:)];
    Blon = Blon(:,1:3);
    Alonprime = Elon\Alon;
    Blonprime = Elon\Blon;
    
    % extract matrices for linearized lateral model
    Elat = [E(2,:);E(5,:);E(7,:);E(9,:)];
    Elat = [Elat(:,2),Elat(:,4),Elat(:,7),Elat(:,9)];
    Alat = [A(2,:);A(4,:);A(7,:);A(9,:)];
    Alat = [Alat(:,2),Alat(:,4),Alat(:,7),Alat(:,9)];
    Blat = [B(2,:);B(4,:);B(7,:);B(9,:)];
    Blat = Blat(:,4:5);
    Alatprime = Elat\Alat;
    Blatprime = Elat\Blat;
    
    % calculate longitudinal eigenvalues
    eigLon = eig(Alonprime);
    [realEigLon, i] = sort(real(EigLon),'descend');
    eigLon = eigLon(i);
    
    % calculate lateral eigenvalues
    eigLat = eig(Alatprime);
    [realEigLat, i] = sort(real(EigLat),'descend');    
    eigLat = eigLat(i);    

end