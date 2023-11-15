% Cbwmatrix.m
% function to calculate transformation matrix from relative wind frame to
% body frame
% usage
%   Cbw = Cbwmatrix(alpha,beta);
% where:
%   alpha : angle of attack (rad)
%   beta : angle of sideslip (rad)
%   Cbw : transformation matrix from relative wind frame to body frame
%

function Cbw = Cbwmatrix(alpha,beta)
 % calculate cosine and sine of alpha and beta
 calpha = cos(alpha);
 salpha = sin(alpha);
 cbeta = cos(beta);
 sbeta = sin(beta);
 % calculate Cbw
 Cbw = [calpha*cbeta   -calpha*sbeta    -salpha;
        sbeta            cbeta             0   ;
        salpha*cbeta   -salpha*sbeta    calpha];
 end
