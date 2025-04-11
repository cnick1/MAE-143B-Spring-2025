%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% MAE143B Spring 2025, Problem Session 1 %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;

%% Part a) Plotting the Step Response
car = tf(0.04, [1 .1])
[y,t] = step(car);

figure
plot(t,y*100,'LineWidth',2)
title('Full throttle car step response')
xlabel('time (s)')
ylabel('speed (m/s)')
legend('speed response')
grid on

%% Part b) P-control
Kp=10; pcontrol = tf(Kp,1)
pcloop = feedback(pcontrol*car,1)
uloop = feedback(pcontrol, car)

figure
[y,t] = step(pcloop);
plot(t,y,'LineWidth',2)
title('One m/s reference → speed closed-loop step response')
xlabel('time (s)')
ylabel('speed (m/s)')
grid on

figure
[y,t] = step(uloop);
plot(t,y,'LineWidth',2)
title('One m/s reference → pedal closed-loop step response')
xlabel('time (s)')
ylabel('pedal position (mm)')
grid on

%% Part c) PI-control
Kp=10; Ki=5; picontrol = tf([Kp Ki],[1 0])
picloop = feedback(picontrol*car,1)
cpiloop = feedback(picontrol, car)
isstable(picloop)

figure
[y,t] = step(picloop);
plot(t,y,'LineWidth',2)
title('One m/s reference → speed closed-loop step response')
xlabel('time (s)')
ylabel('speed (m/s)')
grid on

figure
[y,t] = step(cpiloop);
plot(t,y,'LineWidth',2)
title('One m/s reference → pedal closed-loop step response')
xlabel('time (s)')
ylabel('pedal position (mm)')
grid on