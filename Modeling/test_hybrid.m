clear all; close all; clc;

p = parameters();
z0 = [0.0, -pi/8, -pi/4, -pi/12, 0, 0, 0, 0]';
% z0 = [0, 0, 0, -.3, 0, 0, 0, 0]';
gs = gam_solved_climber(z0, p);
z0(4) = gs(1);
z0(8) = gs(2);

%%
tspan = [0 0.9];
ctrl.tf = 0.35;
ctrl.T1 = [1.2 -1.9 -2.1];
ctrl.T2 = [0.95 -1.9 -1.9];

[tout, zout, uout, indices] = hybrid_simulation(z0, ctrl, p, tspan);

%% Animate
speed = 0.25;
animate_side(tout, zout, p, speed);

%% Energy
E = energy_climber(zout,p);
%     figure(2); clf
%     plot(tout,E);xlabel('Time (s)'); ylabel('Energy (J)');


%%
%     tf: 0.3340
%     T1: [1.2184 -1.9316 -2.1051]
%     T2: [0.9509 -1.8951 -1.8685]