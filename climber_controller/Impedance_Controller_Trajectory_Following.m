% Climber Robot Controller
%
%% SET YOUR INPUTS HERE

% Bezier curve control points
%const_point = [-0.096; -0.133]; %[x;y] or [q1,q2] constant coordinate (x,q1,q2 coordinates should be opposite sign due to direction motors are mounted)
%pts_foot = repmat(const_point,1,8);
       
%pts_foot = [   -0.1033   -0.1033   -0.1033   -0.0668    0.0515    0.1157    0.1157    0.1157
%   -0.1916   -0.1916   -0.1916   -0.1653   -0.1712   -0.1828   -0.1828   -0.1828]; % YOUR BEZIER PTS HERE
% pts_foot =   [-0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960
%    -0.1400   -0.1271   -0.1143   -0.1014   -0.0886   -0.0757   -0.0629   -0.0500];
% pts_foot = [-0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960
% % -0.1400   -0.0767   -0.0133    0.05000   0.0500   -0.0500   -0.1500   -0.2500];

% pts_foot = [-0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960
% -0.1400   -0.0700        -0.04   -0.0700   -0.1400   -0.2100   -0.2500   -0.1400];
% pts_foot(2,2:7) = pts_foot(2,2:7)*1.0;

initial_foot_pts = [ -0.096 ; -0.14];
delta_up = 0.15;
delta_down = 0.15;
%Set up Bezier curve for right foot points
a = linspace(initial_foot_pts(2), initial_foot_pts(2) + delta_up,3);
a = horzcat( a , flip(a(1:length(a)-1)) );
b = linspace(initial_foot_pts(2), initial_foot_pts(2) - delta_down,3);
b = horzcat( b , flip(b(1:length(b)-1)) );
pts_foot = [ initial_foot_pts(1)*ones(1,9) ;  horzcat(a,b(2:length(b)))];
pts_foot = horzcat(pts_foot(:,1:4),pts_foot(:,6:9));

% pts_foot = [-0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960   -0.0960
% -0.1400   -0.0700      0  -0.0700   -0.1400   -0.2100   -0.2500   -0.1400];

% const_point = [-0.096; -0.133]; 
% pts_foot = repmat(const_point,1,8);

% Initial leg angles for encoder resets (negative of q1,q2 in lab handout due to direction motors are mounted)
angle1_init = 0;
angle2_init = -pi/2; 
angle1_initL = 0;
angle2_initL = pi/2; %% Flipped bc of motor orientation

% Total experiment time is buffer,trajectory,buffer
phase_flag        = 0;   % 0 == in-phase, 1 == out-of-phase
num_cycles        = 8;
traj_time         = 2.5;
pre_buffer_time   = 1; % this should be 0 for constant points, 2 for Bezier trajectories
post_buffer_time  = 0.2;

% Gains for impedance controller
% If a gain is not being used in your Mbed code, set it to zero
% For joint space control, use K_xx for K1, K_yy for K2, D_xx for D1, D_yy for D2
gains.K_xx = 65; %60
gains.K_yy = 75; %70
gains.K_xy = 0;

gains.D_xx = 1.0;
gains.D_yy = 1.0;
gains.D_xy = 0.05;

% Maximum duty cycle commanded by controller (should always be <=1.0)
duty_max   = 1;

%% Run Experiment
[output_data] = Experiment_trajectory( angle1_init, angle2_init, angle1_initL, angle2_initL, pts_foot,...
                                       traj_time, pre_buffer_time, post_buffer_time,...
                                       gains, duty_max, num_cycles, phase_flag);

%% Extract data
t = output_data(:,1);
x = -output_data(:,12); % actual foot position in X (negative due to direction motors are mounted)
y = output_data(:,13); % actual foot position in Y
xL = -output_data(:,30); % actual foot position in X (negative due to direction motors are mounted)
yL = output_data(:,31); % actual foot position in Y
   
xdes = -output_data(:,16); % desired foot position in X (negative due to direction motors are mounted)
ydes = output_data(:,17); % desired foot position in Y
xdesL = -output_data(:,34); % desired foot position in X (negative due to direction motors are mounted)
ydesL = output_data(:,35); % desired foot position in Y

current = [output_data(:,3) , output_data(:,8) , output_data(:,21) , output_data(:,26)]; 
           %[current1R , current2R , current1L , current2L]

%% Save workspace
if phase_flag
    gait = "out_of_phase";
else
    gait = "in_phase";
end

filepath = "/Users/mschoder/Dropbox (MIT)/2020_Fall/2_740_BIR/project_code/testing_data/";
filename = strcat( filepath , gait , "traj" , num2str(traj_time) , "s_NumCycles_" , num2str(num_cycles) , '.mat');
save(filename)

%% Plot foot vs desired
figure(5); clf;  %% RIGHT LEG - Desired vs Actual
subplot(211); hold on
title('Right Leg')
plot(t,xdes,'r-'); plot(t,x);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'});

subplot(212); hold on
plot(t,ydes,'r-'); plot(t,y);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'});

figure(6); clf;  %% LEFT LEG - Desired vs Actual
subplot(211); hold on
title('Left Leg');
plot(t,xdesL,'r-'); plot(t,xL);
xlabel('Time (s)'); ylabel('X (m)'); legend({'Desired','Actual'});

subplot(212); hold on
plot(t,ydesL,'r-'); plot(t,yL);
xlabel('Time (s)'); ylabel('Y (m)'); legend({'Desired','Actual'});

figure(7); clf; hold on %% RIGHT LEG -- FOOT POS
plot(xdes,ydes,'r-'); plot(x,y,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});
title('Right Leg')

figure(8); clf; hold on %% LEFT LEG -- FOOT POS
plot(xdesL,ydesL,'r-'); plot(xL,yL,'k');
xlabel('X (m)'); ylabel('Y (m)'); legend({'Desired','Actual'});
title('Left Leg')

% for Cartesian constant points, un-comment this to see the virtual potential well on figure 4
% [X, Y] = meshgrid(linspace(-0.25,0.25,50),linspace(-0.25,0.1,50));
% eX = X - const_point(1); 
% eY = Y - const_point(2); 
% V = 0.5*gains.K_xx*eX.*eX + 0.5*gains.K_yy*eY.*eY + gains.K_xy*eX.*eY;
% axis([-0.25, 0.25, -0.25, 0.1]);
% contour(X,Y,V,15,'LineWidth',1.5);

