clear all; close all; clc;

p = parameters();
z0 = [.1, 0/360*2*pi, -80/360*2*pi, -pi/12, 0, 0, 0, 0]';
% z0 = [0, 0, 0, -0.3, 0, 0, 0, 0]';

z0 = [0, 0/360*2*pi, -0/360*2*pi, -pi, 0, 0, 0, 0]';
p(6) = 180/360*2*pi; % delta

% gs = gam_solved_climber(z0, p);
% z0(4) = gs(1);
% z0(8) = gs(2);

% disp(gs(1))


%%
tspan = [0 0.9];
ctrl.tf = 0.05;
ctrl.T1 = [0.0 0.0 0.0];
ctrl.T2 = [-19 -9.0 -0.0];

[tout, zout, uout, indices] = hybrid_simulation_1motor(z0, ctrl, p, tspan);

%% Plot actual torques delivered
figure(1)
plot(tout, uout)
legend('motor1', 'motor2')

%% Plot torque control profile
figure(2)
ctrl_t = linspace(0, ctrl.tf, 50);
ctrl_pt_t = linspace(0, ctrl.tf, length(ctrl.T1));
n = length(ctrl_t);
ctrl_input = zeros(1,n);

for i=1:n
    ctrl_input1(i) = BezierCurve(ctrl.T1,ctrl_t(i)/ctrl.tf);
    ctrl_input2(i) = BezierCurve(ctrl.T2,ctrl_t(i)/ctrl.tf);
end

hold on
plot(ctrl_t, ctrl_input1, 'b');
plot(ctrl_t, ctrl_input2, 'g');
plot(ctrl_pt_t, ctrl.T1, 'bo');
plot(ctrl_pt_t, ctrl.T2, 'go');
hold off
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Control Input Trajectory')
legend

%% Plot Joint Angles
figure(3)
subplot(4,1,1)
plot(tout, zout(1,:))
ylabel('y (m)')
title('State Values')

subplot(4,1,2)
plot(tout, zout(2,:)*360/(2*pi))
ylabel('theta1 (deg)')

subplot(4,1,3)
plot(tout, zout(3,:)*360/(2*pi))
ylabel('theta2 (deg)')

subplot(4,1,4)
plot(tout, zout(4,:)*360/(2*pi))
ylabel('gamma (deg)')
xlabel('Time (s)')

%% Animate
figure('Position', [10 10 900 1000])
speed = 0.1;
animate_side(tout, zout, p, speed);

%% Energy
E = energy_climber(zout,p);
%     figure(2); clf
%     plot(tout,E);xlabel('Time (s)'); ylabel('Energy (J)');

%% Print integral of torque squared
tausq = tout(1:indices(1)).*uout(:,1:indices(1)).^2;
int_tau2 = sum(tausq, 'all'); 
fprintf('Integral Torque^2: %d\n', int_tau2);
%%
%     tf: 0.3340
%     T1: [1.2184 -1.9316 -2.1051]
%     T2: [0.9509 -1.8951 -1.8685]