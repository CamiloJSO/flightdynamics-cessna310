%Hinv.m
% function to calculate the inverse Hmatrix such that
%   omegab = Hinvmatrix*Phidot
% usage
%   Hinvmatrix = Hinv(Phi)
% where
% Phi = [phi;theta;psi] : euler angles (rad).
% Hinvmatrix : matrix such that omegab = Hinvmatrix*Phidot

function Hinvmatrix = Hinv(Phi)
 % extract Euler angles from Phi 
 phi = Phi(1,1);
 theta = Phi(2,1);
 psi = Phi(3,1);
 % calculate cosine and sine of Euler angles
 cphi = cos(phi);
 sphi = sin(phi);
 ctheta = cos(theta);
 stheta = tan(theta);
 %calculate Hmatrix
 Hinvmatrix = [1     0     -stheta;
               0    cphi   ctheta*sphi;
               0   -sphi   ctheta*cphi];
end