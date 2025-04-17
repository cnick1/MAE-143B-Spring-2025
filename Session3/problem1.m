%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% MAE143B Spring 2025, Problem Session 3, Problem 1 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;

% Setting up the plant model
c = 0.2; J = 1; b = 0.104;
G_theta = tf(c,[J b 0]); 

% simple check of our tf derivation
feedback(2*G_theta,1) % closed-loop tf for K=1

%% Part (a): Proportional Control
% Plot the step response for different gains
figure; hold on; grid on; 

for K = [.1 1 5]
    G_p = tf(K,1);
    H = feedback(G_p*G_theta,1); % closed-loop tf for our K
    [y,t] = step(H);
    plot(t,y,'LineWidth',2); 
end

title('Step response for different proportional gains K')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
legend('K=0.1','K=1','K=5')


%% Part (c): Proportional-Derivative Control
% Plot the step response for different gains
figure; hold on; grid on; t = 0:.01:80;
Kp = 1;
for Kd = [.1 1 1.5192 5]
    G_pd = tf([Kd Kp],1);
    H = feedback(G_pd*G_theta,1); % closed-loop tf for our K
    y = step(H,t);
    plot(t,y,'LineWidth',2); 
end

title('Step response for K_p=1 and different derivative gains K_d')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
legend('K_d=0.1','K_d=1','K_d=1.5192','K_d=5')





