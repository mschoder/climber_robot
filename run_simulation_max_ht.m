clear all; close all; clc;

setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

p = parameters();                           % get parameters from file
z0 = [0; -pi/8; -pi/4; -.2968; 0; 0; 0; 0]; % set initial state [y, th1, th2, gamma, dy, dth1, dth2, dgamma]

% set guess
tf = .7;                                    % simulation final time (will set to range [0.4, 1])
ctrl.tf = .4;                               % control time points (duration of control period)
ctrl.T1 = [1.2 -1.9 -2.1];                  % control values (array from [0, ctrl.tf])
ctrl.T2 = [0.95 -1.9 -1.9];

x = [tf, ctrl.tf, ctrl.T1, ctrl.T2];

% % setup and solve nonlinear programming problem
problem.objective = @(x) objective_max_ht(x,z0,p);     % create anonymous function that returns objective
problem.nonlcon = @(x) constraints_max_ht(x,z0,p);     % create anonymous function that returns nonlinear constraints
problem.x0 = [tf ctrl.tf ctrl.T1 ctrl.T2];      % initial guess for decision variables
problem.lb = [.4 .1 -2.2*ones(size(ctrl.T1)) ...
    -2.2*ones(size(ctrl.T2))];                    % lower bound on decision variables
problem.ub = [1  1  2.2*ones(size(ctrl.T1)) ...
    2.2*ones(size(ctrl.T2))];                     % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');   % set options
problem.solver = 'fmincon';                     % required
x = fmincon(problem);                           % solve nonlinear programming problem

%% Run Sim with optimized results
% re-define tf, tfc, and ctrl here to reflect your solution.

tf = x(1); ctrl.tf = x(2); ctrl.T1 = x(3:5); ctrl.T2 = x(6:8);

[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation

%% Plot COM trajectory
figure(1)
COM = COM_climber(z,p);
plot(t,COM(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('Center of Mass Trajectory')

%% control input profile
figure(2) 
ctrl_t = linspace(0, ctrl.tf, 50);
ctrl_pt_t = linspace(0, ctrl.tf, length(ctrl.T1));
n = length(ctrl_t);
ctrl_input = zeros(1,n);

for i=1:n
    ctrl_input1(i) = BezierCurve(x(3:5),ctrl_t(i)/ctrl.tf);
    ctrl_input2(i) = BezierCurve(x(6:8),ctrl_t(i)/ctrl.tf);
end

hold on
plot(ctrl_t, ctrl_input1, 'b');
plot(ctrl_t, ctrl_input2, 'g');
plot(ctrl_pt_t, x(3:5), 'bo');
plot(ctrl_pt_t, x(6:8), 'go');
hold off
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('Control Input Trajectory')
legend('th1', 'th2', 'th1 ctrl points', 'th2 control points')
%% Plot Energy

E = energy_climber(z,p);
figure(3)
plot(t, E);
title('Energy')
xlabel('Time (s)')
ylabel('Energy (J)')

%% Plot Joint Angles
figure(4)
subplot(4,1,1)
plot(t, z(1,:))
ylabel('y (m)')
title('State Values')

subplot(4,1,2)
plot(t, z(2,:)*360/(2*pi))
ylabel('theta1 (deg)')

subplot(4,1,3)
plot(t, z(3,:)*360/(2*pi))
ylabel('theta2 (deg)')

subplot(4,1,4)
plot(t, z(4,:)*360/(2*pi))
ylabel('gamma (deg)')
xlabel('Time (s)')

%% Run the animation
figure(5)                          % get the coordinates of the points to animate
speed = .05;                       % set animation speed
clf                                % clear fig
animate_side(t,z,p,speed)          % run animation

