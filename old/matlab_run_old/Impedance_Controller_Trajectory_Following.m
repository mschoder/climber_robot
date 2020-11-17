%
%
% MODIFIED THE CODE FROM LAB 5
% This is the main MATLAB script for Lab 5.
%
%
%% SET YOUR INPUTS HERE
% Phase offset between the two legs between 1 and 7
% Choose 0 for perfectly in phase (bounding)
% Choose 7 for perfectly out of phase (alternating)
phase_offset = 1;

% Number of step cycles executed
num_cycles = 1;

% Bezier curve control points
%const_point = [0.096; -0.1400]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
%pts_foot = repmat(const_point,1,8);

% TRAJECTORY: NEUTRAL-CROUCHING-EXTENDED (FOR REFERENCE)
% step_traj =   [0.0960   0.0960   0.0960   0.0960   0.0960   0.0960
% 0.0960   0.0960;
%    -0.1400   -0.0767   -0.0133    0.0500   0.0500   -0.0500   -0.1500   -0.2500];

% TRAJECTORY: BOUNDING
%  step_traj =   [-0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960;
%     0.0500   -0.0500   -0.1500   -0.2500   -0.1400   -0.0767   -0.0133    0.0500];

% Zero traj
 step_traj =   [-0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960;
    -0.1400   -0.1400   -0.1400   -0.1400   -0.1400   -0.1400   -0.1400    -0.1400];

% stipulate right leg starts in crouched position
R_pts_foot = step_traj;

% offset left leg by phase_offset and correct sign issues
L_pts_foot = [step_traj(:,phase_offset:end) step_traj(:,1:phase_offset-1)];
L_pts_foot(1,:) = -1 * L_pts_foot(1,:);
%%
        
% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
R_angle1_init = 0;
R_angle2_init = -pi/2; 
L_angle1_init = 0;
L_angle2_init = pi/2;

% Total experiment time is buffer,trajectory,buffer
traj_time         = 2;
pre_buffer_time   = 2; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 0;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 300; % joint = 1; cart = 50; op = 300
gains.K_yy = 300; % 310
gains.K_xy = 0; % 10

gains.D_xx = 25; % joint = 0.05; cart = 50; op = 100
gains.D_yy = 25;
gains.D_xy = 0;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 0.1;

%% Run Experiment
[output_data] = Experiment_trajectory( R_angle1_init, R_angle2_init, L_angle1_init, L_angle2_init,...
                                       R_pts_foot, L_pts_foot, num_cycles,...
                                       traj_time, pre_buffer_time, post_buffer_time,...
                                       gains, duty_max);

%% Extract data
t = output_data(:,1);
x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
y = output_data(:,13); % actual foot position in Y
   
xdes = -output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
ydes = output_data(:,17); % desired foot position in Y

%% Plot foot vs desired
figure(3); clf;
subplot(211); hold on
plot(t,xdes,'r-'); plot(t,x);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'});

subplot(212); hold on
plot(t,ydes,'r-'); plot(t,y);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'});

figure(4); clf; hold on
plot(xdes,ydes,'r-'); plot(x,y,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});
