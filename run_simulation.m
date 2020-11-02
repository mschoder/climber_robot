clear all; close all; clc;

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

p = parameters();                           % get parameters from file
z0 = [0; pi/6; 0 ; 0; 200];                   % set initial state [y, th, dy, dth, (int_tau^2)]
% Note: 5th state is the integral of torque squared over time
% An equation has been added to dynamics_continuous and dynamics_discrete
% to integrate this new state.

% set guess
tf = .8;                                        % simulation final time (will set to range [0.4, 1])
ctrl.tf = .35;                                  % control time points (duration of control period)
ctrl.T = [1.0 1.0 1.0];                          % control values (array from [0, ctrl.tf])??

x = [tf, ctrl.tf, ctrl.T];
% % setup and solve nonlinear programming problem
problem.objective = @(x) objective(x,z0,p);     % create anonymous function that returns objective
problem.nonlcon = @(x) constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
problem.x0 = [tf ctrl.tf ctrl.T];               % initial guess for decision variables
problem.lb = [.4 .1 -2*ones(size(ctrl.T))];     % lower bound on decision variables -- CHECK should this be 0.4??
problem.ub = [1  1   2*ones(size(ctrl.T))];     % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');   % set options
problem.solver = 'fmincon';                     % required
x = fmincon(problem);                           % solve nonlinear programming problem

% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.

tf = x(1); ctrl.tf = x(2); ctrl.T = x(3:end);

[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation

%% Plot COM for your submissions
figure(1)
COM = COM_jumping_leg(z,p);
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