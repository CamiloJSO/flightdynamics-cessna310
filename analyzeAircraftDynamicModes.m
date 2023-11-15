% analyzeAircraftDynamicModes.m
% function to analyze longitudinal and lateral aircraft dynamic modes
% usage
% aicraftDynamicModes = analyzeAircraftDynamicModes(eigLon,eigLat)
% where
%   eigLon : longitudinal eigenvalues
%   eigLat : lateral eigenvalues
% aircraftDynamicModes : structure with parameters of aircraft dynamic
% modes

function aicraftDynamicModes = analyzeAircraftDynamicModes(eigLon,eigLat,aircraft)
    % plot longitudinal eigenvalues
    figure(1)
    plot(real(eigLon),imag(eigLon), 'xr');axis equal;grid on;xlabel('Re');ylabel('Im');
    title(['Longitudinal eigenvalues for', aircraft.aircraftName]);
    
    figure(2)
    plot(real(eigLat),imag(eigLat), 'xr');axis equal;grid on;xlabel('Re');ylabel('Im');
    title(['Lateral eigenvalues for', aircraft.aircraftName]);
    
    % longitudinal eigenvalues
    % phugoid mode
    eig1 = eigLon(1);
    eig2 = eigLon(2);
    wn = sqrt(abs(eig1*eig2));
    zeta = -real(eig1+eig2)/(2*wn);
    if zeta<1
        wd = abs(ima(eig1));
        T = 2*pi/wd;
    end
    tau = -1/real(eig1);
    phugoid.eig1 = eig1;
    phugoid.eig2 = eig2;
    phugoid.wn = wn;
    phugoid.zeta = zeta;
    if zeta<1
        phugoid.wd = wd;
        phugoid.T = T;
    end
    phugoid.tau = tau;
    
    % short period mode
    eig1 = eigLon(3);
    eig2 = eigLon(4);
    wn = sqrt(abs(eig1*eig2));
    zeta = -real(eig1+eig2)/(2*wn);
    if zeta<1
        wd = abs(ima(eig1));
        T = 2*pi/wd;
    end
    tau = -1/real(eig1);
    shortPeriod.eig1 = eig1;
    shortPeriod.eig2 = eig2;
    shortPeriod.wn = wn;
    shortPeriod.zeta = zeta;
    if zeta<1
        shortPeriod.wd = wd;
        shortPeriod.T = T;
    end
    shortPeriod.tau = tau;
    
    % lateral eigenvalues
    % spiral mode
    eig1 = eigLat(1);
    tau = -1/eig1;
    spiral.eig = eig1;
    spiral.tau = tau;
    
    % dutch roll mode
    eig1 = eigLon(2);
    eig2 = eigLon(3);
    wn = sqrt(abs(eig1*eig2));
    zeta = -real(eig1+eig2)/(2*wn);
    if zeta<1
        wd = abs(ima(eig1));
        T = 2*pi/wd;
    end
    tau = -1/real(eig1);
    dutchRoll.eig1 = eig1;
    dutchRoll.eig2 = eig2;
    dutchRoll.wn = wn;
    dutchRoll.zeta = zeta;
    if zeta<1
        dutchRoll.wd = wd;
        dutchRoll.T = T;
    end
    dutchRoll.tau = tau;
    
    % roll mode
    eig1 = eigLat(4);
    tau = -1/eig1;
    roll.eig = eig1;
    roll.tau = tau;
    
    aircraftDynamicModes.phugoid = phugoid;
    aircraftDynamicModes.shortPeriod = shortPeriod;
    aircraftDynamicModes.spiral = spiral;
    aircraftDynamicModes.dutchRoll = dutchRoll;
    aircraftDynamicModes.roll = roll;
end