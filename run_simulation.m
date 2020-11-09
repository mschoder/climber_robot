clear all; close all; clc;

setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

p = parameters();                           % get parameters from file
z0 = [0; pi/8; pi/4; pi/8; 0; 10; 10; 0];   % set initial state [y, th1, th2, gamma, dy, dth1, dth2, dgamma]

% set guess
tf = .8;                                    % simulation final time (will set to range [0.4, 1])
ctrl.tf = .45;                              % control time points (duration of control period)
ctrl.T1 = [1.0 1.0 1.0];                    % control values (array from [0, ctrl.tf])
ctrl.T2 = [1.0 1.0 1.0];

x = [tf, ctrl.tf, ctrl.T1, ctrl.T2];

% % setup and solve nonlinear programming problem
problem.objective = @(x) objective(x,z0,p);     % create anonymous function that returns objective
problem.nonlcon = @(x) constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
problem.x0 = [tf ctrl.tf ctrl.T1 ctrl.T2];      % initial guess for decision variables
problem.lb = [.4 .1 -2*ones(size(ctrl.T1)) ...
    -2*ones(size(ctrl.T2))];                    % lower bound on decision variables
problem.ub = [1  1   2*ones(size(ctrl.T1)) ...
    2*ones(size(ctrl.T2))];                     % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');   % set options
problem.solver = 'fmincon';                     % required
x = fmincon(problem);                           % solve nonlinear programming problem

%%
% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.

tf = x(1); ctrl.tf = x(2); ctrl.T1 = x(3:5); ctrl.T2 = x(6:8);

[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation

%% Plot COM for your submissions
figure(1)
COM = COM_climber(z,p);
plot(t,COM(2,:))
xlabel('time (s)')
ylabel('CoM Height (m)')
title('Center of Mass Trajectory')

figure(2)  % control input profile
ctrl_t = linspace(0, ctrl.tf, 50);
ctrl_pt_t = linspace(0, ctrl.tf, length(ctrl.T));
n = length(ctrl_t);
ctrl_input = zeros(1,n);

for i=1:n
    ctrl_input(i) = BezierCurve(x(3:end),ctrl_t(i)/ctrl.tf);
end

hold on
plot(ctrl_t, ctrl_input);
plot(ctrl_pt_t, x(3:end), 'o');
hold off
xlabel('time (s)')
ylabel('torque (Nm)')
title('Control Input Trajectory')
%%
% Run the animation
figure(3)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
clf                                         % clear fig
animate_simple(t,z,p,speed)                 % run animation