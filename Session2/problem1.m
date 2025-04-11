%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% MAE143B Spring 2025, Problem Session 1 %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;

%% Part a) Plotting the Step Response
car = tf(0.04, [1 .1])
[y,t] = step(car);

plot(t,y*100,'LineWidth',2)
title('Car step response')
xlabel('time (s)')
ylabel('speed (m/s)')
legend('speed response')
grid on

%% Part b) P-control
Kp=10; pcontrol = tf(Kp,1)
pcloop = feedback(pcontrol*car,1);
uloop = feedback(pcontrol, car);

[y,t] = step(uloop);
plot(t,y,'LineWidth',2)
title('Car step response with unity feedback')
xlabel('time (s)')
ylabel('speed (m/s)')
legend('speed response')
grid on