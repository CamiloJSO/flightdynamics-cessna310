%H.m
% function to calculate Hmatrix such that
%   Phidot = Hmatrix*omegab
% usage
%   Hmatrix = H(Phi)
% where
% Phi = [phi;theta;psi] : euler angles (rad).
% Hmatrix : matrix such that Phidot = Hmatrix*omegab

function Hmatrix = H(Phi)
 % extract Euler angles from Phi 
 phi = Phi(1,1);
 theta = Phi(2,1);
 psi = Phi(3,1);
 % calculate cosine and sine of Euler angles
 cphi = cos(phi);
 sphi = sin(phi);
 ctheta = cos(theta);
 ttheta = tan(theta);
 %calculate Hmatrix
 Hmatrix = [1   ttheta*sphi   ttheta*cphi;
            0   cphi          -sphi;
            0   sphi/ctheta   cphi/ctheta];
end