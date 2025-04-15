%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% MAE143B Spring 2025, Problem Session 3, Problem 1 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;

%% Part (a): Derive the closed-loop transfer function
c = 0.2; J = 1; b = 0.104;
G_theta = tf(c,[J b 0]); 

fprintf('\n(a) The transfer function from theta_r to theta_m (for K=1 here) is:\n')
Gcloop = feedback(G_theta,1) % closed-loop tf for K=1


%% Part (b): Plot the step response for different gains
figure; hold on; grid on;

for K = [.1 1 5]
    G = feedback(K*G_theta,1); % closed-loop tf for our K
    [y,t] = step(G);
    plot(t,y,'LineWidth',2); 
end

title('Step response for different gains K')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
legend('K=0.1','K=0.5','K=1','K=2')


%%
% First compute zeta
zeta = sqrt((log(0.2)/-pi)^2/(1+(log(0.2)/-pi)^2));

% Rearranging for K gives this
K = b^2/(4*zeta^2*0.2);
fprintf('\n(b) The maximum gain that will produce an overshoot of 20 percent or less is K=%2.4f\n',K)

% Check if it is reasonable: this formula says it should be close to 20%
% Mp = exp(-pi*zeta/sqrt(1-zeta^2))

% We can (and should) also plot the closed-loop response verify the result
G_thetaR_to_thetaM = feedback(K*G_theta,1); % closed-loop tf for our K
[y,t] = step(G_thetaR_to_thetaM);

figure
plot(t,y,'LineWidth',2); grid on;
title('Unit step response with K=0.065 producing 20% overshoot ')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
xlim([0 80]); ylim([-.1 1.3]); legend('K=0.065','AutoUpdate','off')

% Annotate overshoot
% Compute max overshoot
[y_peak, peak_idx] = max(y);
Mp = (y_peak - 1) / 1;
tp = t(peak_idx);

line([tp tp], [1 y_peak], 'Color', 'k', 'LineStyle', '--');
line([tp tp+25], [1 1], 'Color', 'k', 'LineStyle', '--');
line([tp tp+25], [1.2 1.2], 'Color', 'k');
text(tp+20, y_peak - 0.1, sprintf('M_p = %.1f%%', Mp * 100), ...
    'HorizontalAlignment', 'left', 'Color', 'k');

%% Part (c) 
% Now neglecting the overshoot, if we want to get a rise time of 4 seconds,
% we need this gain: 

K = 1.8^2/4^2/.2;
fprintf('\n(c) A gain of K=%2.4f will produce a rise time around 4 seconds.\n',K)

% Check if it is reasonable: this formula says it should be close to 4 sec
% t_r = 1.8/sqrt(0.2*K)

% We can (and should) also plot the closed-loop response verify the result
G_thetaR_to_thetaM = feedback(K*G_theta,1); % closed-loop tf for our K
[y,t] = step(G_thetaR_to_thetaM,0:.01:80);

figure
plot(t,y,'LineWidth',2); grid on; hold on;
title('Unit step response with K=1.0125 producing rise time of ~4 sec ')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
xlim([0 80]); ylim([-.2 1.8]); legend('K=1.0125','AutoUpdate','off')

% Annotate rise time
% Compute rise time (from 10% to 90%)
y_final = y(end);
tr_start_idx = find(y >= 0.1 * y_final, 1, 'first');
tr_end_idx = find(y >= 0.9 * y_final, 1, 'first');
tr_start = t(tr_start_idx);
tr_end = t(tr_end_idx);

plot([tr_start tr_end], [-0.1 -0.1]*y_final, 'k', 'LineWidth', 1.5);
text(mean([tr_start, tr_end]), -0.13*y_final, 't_r', 'HorizontalAlignment', 'center');

% 10% line
line([0 tr_start], [0.1 0.1], 'Color', 'k', 'LineStyle', '--');
line([tr_start tr_start], [-0.1 0.1], 'Color', 'k');

% 90% line
line([0 tr_end], [0.9 0.9], 'Color', 'k', 'LineStyle', '--');
line([tr_end tr_end], [-0.1 0.9], 'Color', 'k');

%% Part (d)
figure
title('Step response, various gains: tradeoff between rise time and overshoot ')
xlabel('time (s)'); ylabel('\theta_m(t) (rad)')
grid on; hold on; t = 0:0.01:60;

for K = [0.5, 1, 2, 0.065]
    G_thetaR_to_thetaM = feedback(K*G_theta,1);
    [y] = step(G_thetaR_to_thetaM,t);
    plot(t,y,'LineWidth',2); 

end

xlim([0 60]); ylim([-.25 2])
legend('K=0.5','K=1','K=2','K=0.065')


