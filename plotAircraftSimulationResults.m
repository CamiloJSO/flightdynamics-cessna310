% plotAircraftSimulationResults.m
% function to plot aircraft simulation results obtained with
% simulateAircraft.
%  usage
%    plotAircraftSimulationResults(t,X,Y,aircraft)
%  where:
%   t : simulation times vector
%   X : matrix with simulation results. Row ith of X is x' for t=t(i)
%   Y : matrix with [V,alpha,beta] for each t(i).
%   aircraft : aircraft data structure

function plotAircraftSimulationResults(t,X,Y,aircraft)
 % airspeed
 figure(1)
 plot(t,Y(:,1)*3600/1852,'-b');xlabel('t (s)');ylabel('V (kn)');title(['Airspeed for ',aircraft.aircraftName, ' aircraft'])

 % angle of attack
 figure(2)
 plot(t,Y(:,2)*180/pi,'-b');xlabel('t (s)');ylabel('\alpha (deg)');title(['Angle of attack for ',aircraft.aircraftName, ' aircraft'])
 
 % angle of sideslip
 figure(3)
 plot(t,Y(:,3)*180/pi,'-b');xlabel('t (s)');ylabel('\beta (deg)');title(['Angle of sideslip ',aircraft.aircraftName, ' aircraft'])
 
 % roll
 figure(4)
 plot(t,X(:,4)*180/pi,'-b');xlabel('t (s)');ylabel('\phi (deg)');title(['Roll for ',aircraft.aircraftName, ' aircraft'])
 
 % pitch
 figure(5)
 plot(t,X(:,5)*180/pi,'-b');xlabel('t (s)');ylabel('\theta (deg)');title(['Pitch for ',aircraft.aircraftName, ' aircraft'])
 
 % heading
 figure(6)
 plot(t,X(:,6)*180/pi,'-b');xlabel('t (s)');ylabel('\psi (deg)');title(['Heading for ',aircraft.aircraftName, ' aircraft'])
 
 % roll rate
 figure(7)
 plot(t,X(:,10),'-b');xlabel('t (s)');ylabel('p (rad/s)');title(['Roll rate for ',aircraft.aircraftName, ' aircraft'])
 
 % pitch rate
 figure(8)
 plot(t,X(:,11),'-b');xlabel('t (s)');ylabel('q (rad/s)');title(['Pitch rate for ',aircraft.aircraftName, ' aircraft'])
 
 % yaw rate
 figure(9)
 plot(t,X(:,12),'-b');xlabel('t (s)');ylabel('r (rad/s)');title(['Yaw rate for ',aircraft.aircraftName, ' aircraft'])
 
 % aircraft position
 figure(10)
 plot(X(:,2),X(:,1),'-b');axis equal;xlabel('ye ->E (m)');ylabel('xe -> N (m)');title(['Position for ',aircraft.aircraftName, ' aircraft'])
 
 % altitude
 figure(11)
 plot(t,-X(:,3)/0.3048,'-b');xlabel('t (s)');ylabel('h (ft)');title(['Altitude for ',aircraft.aircraftName, ' aircraft'])

end