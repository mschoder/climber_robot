clear all; close all; clc;

p = parameters();
z0 = [0.0, -30/360*2*pi, -130/360*2*pi, -pi/12, 0, 0, 0, 0]';
% z0 = [0, 0, 0, -.3, 0, 0, 0, 0]';
gs = gam_solved_climber(z0, p);
z0(4) = gs(1);
z0(8) = gs(2);
disp(gs(1))

%%
tspan = [0 4];
ctrl.tf = 0.9;
ctrl.T1 = [1.0548 0.0365 -1.6144]/2;
ctrl.T2 = [-0.2796 -1.3954 -1.7567]/2;

[tout,zout_l,zout_r,uout_l,uout_r,indices] = hybrid_simulation_2leg(z0,ctrl,p,tspan);

%% Plot torque control profile
figure(1)
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
figure(2)
subplot(4,1,1)
hold on
plot(tout, zout_l(1,:))
plot(tout, zout_r(1,:))
ylabel('y (m)')
title('State Values')

subplot(4,1,2)
hold on
plot(tout, zout_l(2,:)*360/(2*pi))
plot(tout, zout_r(2,:)*360/(2*pi))
ylabel('theta1 (deg)')

subplot(4,1,3)
plot(tout, zout_l(3,:)*360/(2*pi))
plot(tout, zout_r(3,:)*360/(2*pi))
ylabel('theta2 (deg)')

subplot(4,1,4)
plot(tout, zout_l(4,:)*360/(2*pi))
plot(tout, zout_r(4,:)*360/(2*pi))
ylabel('gamma (deg)')
xlabel('Time (s)')

%% Animate
figure(3)
speed = 0.2;
hold on
animate_side(tout, zout_l, p, speed);
% animate_size(tout, zout_r, p, speed);

%% Energy
% E = energy_climber(zout,p);
%     figure(2); clf
%     plot(tout,E);xlabel('Time (s)'); ylabel('Energy (J)');

%% Print integral of torque squared
tausq_l = tout(1:indices(1)).*uout_l(:,1:indices(1)).^2;
tausq_r = tout(1:indices(1)).*uout_r(:,1:indices(1)).^2;
int_tau2 = sum(tausq_l, 'all') + sum(tausq_r, 'all'); 
fprintf('Integral Torque^2: %d\n', int_tau2);
%%
%     tf: 0.3340
%     T1: [1.2184 -1.9316 -2.1051]
%     T2: [0.9509 -1.8951 -1.8685]