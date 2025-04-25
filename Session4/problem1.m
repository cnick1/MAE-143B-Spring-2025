%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% MAE143B Spring 2025, Problem Session 4, Problem 1 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;

%% Setting up the throttle valve servo plant model
c = 0.2; J = 1; b = .104;
G_theta = tf(c,[J b 0]); 

dt = 0.01;                          % Time step
t = -1:dt:12;                       % Total time vector
% t = 0:dt:12;                       % Total time vector

%% P vs PD Control for Step Response Test
% Simulate the closed-loop step response using P vs PD control
figure; hold on; grid on; 
Kp = 5; Kd = 20; 

D_p = tf(Kp,1);
D_pd1 = tf([Kd Kp],1);
H_p = feedback(D_p*G_theta,1); % closed-loop tf for P-control
H_pd1 = feedback(D_pd1*G_theta,1); % closed-loop tf for PD-control

y_p  = step(H_p, t);
y_pd1 = step(H_pd1,t);

plot(t,y_p,'LineWidth',2); hold on
plot(t,y_pd1,'LineWidth',2);

title('Step response for different controllers')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
legend('P with K_p=100','PD with K_p=100 and K_d=10')

%% Set up nominal throttle curve and ±5% bounds
% During hotfire, demonstrate controlled nominal thrust at or above 500 lbf
% for 4 seconds, smoothly throttle down to 40% of that nominal thrust and
% hold constant for at least 2 seconds, then return to the nominal thrust 
% for at least 1 second. Bounds of ±5% are considered acceptable.

% Parameters
nominal_thrust = 1.0;              % Normalized to 1
low_thrust = 0.4 * nominal_thrust; % 40% throttle
ramp_duration = 2.0;               % Duration of throttle ramp (seconds)

% Convert durations to indices
idx_4s = round(5 / dt);                           % End of initial 4s
idx_ramp = round(ramp_duration / dt);             % Ramp duration in indices
idx_hold_low = round(2 / dt);                     % Duration to hold low throttle
idx_end_ramp_down = idx_4s + idx_ramp;            % End of ramp down
idx_end_hold = idx_end_ramp_down + idx_hold_low;  % End of low hold
idx_end_ramp_up = idx_end_hold + idx_ramp;        % End of ramp up

% Initialize thrust trace
yref = zeros(size(t));

% Step 1: First 4 seconds at nominal thrust
yref(1:idx_4s) = nominal_thrust;

% Step 2: Ramp down to 40%
yref(idx_4s+1:idx_end_ramp_down) = linspace(nominal_thrust, low_thrust, idx_ramp);

% Step 3: Hold at 40% for 2 seconds
yref(idx_end_ramp_down+1:idx_end_hold) = low_thrust;

% Step 4: Ramp up to 100%
yref(idx_end_hold+1:idx_end_ramp_up) = linspace(low_thrust, nominal_thrust, idx_ramp);

% Step 5: Hold at nominal thrust for remaining time
yref(idx_end_ramp_up+1:end) = nominal_thrust;

% yref = t./2;

% Plot for verification
figure; hold on; grid on; 
plot(t, yref,'k', 'LineWidth', 2); hold on;
plot(t, yref-0.05, 'r--'); % 5% below
plot(t, yref+0.05, 'r--'); % 5% above 
xlabel('Time (s)')
ylabel('Throttle (normalized)')
title('Throttle Trace')
% line([0 0], [0 1.1], 'Color', 'g');
fill([-1 0 0 -1],[0 0 1.1 1.1],'red','FaceAlpha',0.3)
xticks(-1:1:12)
ylim([0, 1.1])
xlim([-1, 12])


% P vs PD Control for Throttle Trace Response Test
% Simulate the closed-loop throttle trace response using P vs PD control
% figure; hold on; grid on; 

D_p = tf(Kp,1);
D_pd1 = tf([Kd Kp],1);
D_pd2 = tf([50 Kp],1);
H_p = feedback(D_p*G_theta,1); % closed-loop tf for P-control
H_pd1 = feedback(D_pd1*G_theta,1); % closed-loop tf for PD-control
H_pd2 = feedback(D_pd2*G_theta,1); % closed-loop tf for PD-control

y_p  = lsim(H_p,  yref, t);
y_pd1 = lsim(H_pd1, yref, t);
y_pd2 = lsim(H_pd2, yref, t);

plot(t,y_p,'LineWidth',2); hold on
plot(t,y_pd1,'LineWidth',2);
plot(t,y_pd2,'LineWidth',2);

title('Throttle trace response for different controllers')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
legend('Throttle trace reference','Upper/Lower 5% bounds','','Pre-test start-up','P with K_p=5','PD with K_p=5 and K_d=20','PD with K_p=5 and K_d=50')
% Our closed-loop system is type 1: it has zero steady-state error for step
% inputs, but a constant offset for ramp inputs. We need our closed-loop 
% system to be type 2, so that it can track the ramp inputs. We need to add
% integral action. 

% Note: if we only care about step inputs, integral action is not
% necessary, hence why last week we didn't use it. Only now that we have 
% learned more and encountered a problem where we need it will we apply it. 


%% PD vs PID Control for Throttle Trace Response Test
% Simulate the closed-loop throttle trace response using PD vs PID control
% figure; hold on; grid on; 
figure; hold on; grid on; 
plot(t, yref,'k', 'LineWidth', 2); hold on;
plot(t, yref-0.05, 'r--'); % 5% below
plot(t, yref+0.05, 'r--'); % 5% above 
xlabel('Time (s)')
ylabel('Throttle (normalized)')
title('Throttle Trace')
% line([0 0], [0 1.1], 'Color', 'g');
fill([-1 0 0 -1],[0 0 1.1 1.1],'red','FaceAlpha',0.3)
xticks(-1:1:12)
ylim([0, 1.1])
xlim([-1, 12])
plot(t,y_pd2,'Color','#4DBEEE','LineWidth',2);

% Kp = 5; Kd = 50; Ki = 2;      % First  attempt
Kp = 5; Kd = 50; Ki = 10;     % Second attempt
% Kp = 5; Kd = 100; Ki = 10;    % Third  attempt
% Kp = 20; Kd = 100; Ki = 10;   % Fourth attempt
% Kp = 50; Kd = 200; Ki = 10;   % Fifth  attempt
% Kp = 50; Kd = 200; Ki = 0;    % Sixth  attempt

D_pid = tf([Kd Kp Ki],[1 0]);
H_pid = feedback(D_pid*G_theta,1); % closed-loop tf for PD-control
pole(H_pid)

y_pid = lsim(H_pid, yref, t);

plot(t,y_pid,'LineWidth',2);

title('Throttle trace response for different controllers')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
legend('Throttle reference','Upper/Lower 5% bounds','','Pre-test start-up','PD with K_p=5 and K_d=50', sprintf('PID with K_p=%i, K_d=%i, and K_i = %i',Kp,Kd,Ki))

%% PID Overshoot and Windup 
% In theory the PID controller should make the system Type 2 and should 
% track the ramp and step input with no steady-state error. But only if we
% let it reach steady state! Even though we think of rise-time as the time
% to reach steady-state, that is only true for seecond-order system with no
% zeros! Our system is not that! So yes, we have fast rise time, but the
% time to reach steady state is much longer. 

Kp = 5; Kd = 50; Ki = 10;     % Second attempt

D_pid = tf([Kd Kp Ki],[1 0]);
H_pid = feedback(D_pid*G_theta,1); % closed-loop tf for PD-control


figure; hold on; grid on; 
t2 = [0:.01:100];
[y_pid2] = step(H_pid,t2);
plot(t2,y_pid2,'LineWidth',2);

title('Step response for PID controllers')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
legend('P with K_p=100','PD with K_p=100 and K_d=10')