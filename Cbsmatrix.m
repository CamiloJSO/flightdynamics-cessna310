% Cbsmatrix.m
% function to calculate transformation matrix from stability frame to
% body frame
% usage
%   Cbs = Cbsmatrix(alpha);
% where:
%   alpha : angle of attack (rad)
%   Cbs : transformation matrix from stability frame to body frame
%

function Cbs = Cbsmatrix(alpha)
 % calculate cosine and sine of alpha
 calpha = cos(alpha);
 salpha = sin(alpha);
 % calculate Cbs
 Cbs = [calpha   0    -salpha;
          0      1       0   ;
        salpha   0    calpha];
 end
