% atmosphere.m
% Function to calculate ISA atmosphere model
% Model valid for h<20000m (up to isothermal layer in stratosphere)
% Usage:
%   [rho,P,T,a] = atmosphere(h)
% with:
%   h : altitude above mean sea level [=] m
%   rho : air density according to ISA atmosphere model
%   P : atmospheric pressure according to ISA atmosphere model
%   T : temeperature according to ISA atmosphere model
%   a : speed of sound according to ISA atmosphere model
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function [rho,P,T,a] = atmosphere(h)

Tsl_ISA = 15;  % temperature at mean sea level (ISA model) (C)
deltaT = 0; % latitude temperature deviation from ISA (C)
Tsl_C = Tsl_ISA+deltaT; % temperature at mean sea level (C)
Tsl = Tsl_C+273.15; % absolute temperature at mean sea level (K)

g = 9.80665; % gravity for ISA atmosphere model
Psl = 101325; % pressure at mean sea level (ISA model) (Pa)
L = 0.0065; % lapse rate (ISA model) (K/m)
Rstar = 8.31432; % universal gas constant (J/mole�K)
M = 0.0289644; % mean molecular mass of air (kg/mole)
R = Rstar/M; % gas constant for air (J/(kg.K))
gamma = 1.4; % ratio of specific heat capacities of air, cp/cv
n = g/(R*L);

hs = 11000; % ISA atmosphere troposphere altitude (m)
Ts = Tsl-L*hs; % ISA atmosphere temperature at stratosphere (K)

% air density at stratosphere altitude
Ps = Psl*(Ts/Tsl)^n; % pressure at stratosphere altitude

if h<hs % in troposphere
    T = Tsl-L*h; % temperature at altitude (K)
    P = Psl*(T/Tsl).^n; % pressure at altitude
else % in stratosphere
    T = Ts;
    P = Ps*exp(g*(hs-h)/(R*Ts)); % pressure at altitude
end
rho = P./(R*T); % density at altitude

a = sqrt(gamma*R*T);
