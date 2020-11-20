function output_data = Experiment_trajectory( angle1_init, angle2_init, angle1_initL, angle2_initL,...
    pts_foot, traj_time, pre_buffer_time, post_buffer_time, gains, duty_max, num_cycles, phase_flag)
    
    % Figure for plotting motor data _ RIGHT
    figure(1);  clf;
    a1 = subplot(421);
    title('Right Side Motor Data')
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');
    
    a2 = subplot(423);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)');
    
    a3 = subplot(425);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    subplot(425);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(427);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Cycle 1');
    
    a5 = subplot(422);
    title('Left Side Motor Data')
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)');
    
    a6 = subplot(424);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity 2 (rad/s)');
    
    a7 = subplot(426);
    h23 = plot([0],[0]);
    h23.XData = []; h23.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(426);
    h24 = plot([0],[0],'r');
    h24.XData = []; h24.YData = [];
    hold off;
    
    a8 = subplot(428);
    h25 = plot([0],[0]);
    h25.XData = []; h25.YData = [];
    ylabel('Duty Cycle 2');
    
    %%% FIGURE for Plottting LEFT MOTOR data
    figure(2);  clf;    
    a1L = subplot(421);
    title('Left Side Motor Data')
    h1L = plot([0],[0]);
    h1L.XData = []; h1L.YData = [];
    ylabel('Angle 1 (rad)');
    
    a2L = subplot(423);
    h2L = plot([0],[0]);
    h2L.XData = []; h2L.YData = [];
    ylabel('Velocity 1 (rad/s)');
    
    a3L = subplot(425);
    h3L = plot([0],[0]);
    h3L.XData = []; h3L.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    
    subplot(425);
    h4L = plot([0],[0],'r');
    h4L.XData = []; h4L.YData = [];
    hold off;
    
    a4L = subplot(427);
    h5L = plot([0],[0]);
    h5L.XData = []; h5L.YData = [];
    ylabel('Duty Cycle 1');
    

    a5L = subplot(422);
    title('Left Side Motor Data')
    h21L = plot([0],[0]);
    h21L.XData = []; h21L.YData = [];
    ylabel('Angle 2 (rad)');
    
    a6L = subplot(424);
    h22L = plot([0],[0]);
    h22L.XData = []; h22L.YData = [];
    ylabel('Velocity 2 (rad/s)');
    
    a7L = subplot(426);
    h23L = plot([0],[0]);
    h23L.XData = []; h23L.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(426);
    h24L = plot([0],[0],'r');
    h24L.XData = []; h24L.YData = [];
    hold off;
    
    a8L = subplot(428);
    h25L = plot([0],[0]);
    h25L.XData = []; h25L.YData = [];
    ylabel('Duty Cycle 2');
    
    % Figure for plotting state of the leg - RIGHT
    figure(3)
    clf
    hold on
    axis equal
    axis([-.25 .25 -.25 .1]);
   
    h_OB = plot([0],[0],'LineWidth',2);
    h_AC = plot([0],[0],'LineWidth',2);
    h_BD = plot([0],[0],'LineWidth',2);
    h_CE = plot([0],[0],'LineWidth',2);
    
    h_foot= plot([0],[0],'k');
    h_des = plot([0],[0],'k--');
    h_des.XData=[];
    h_des.YData=[];
    h_foot.XData=[];
    h_foot.YData=[];
    
    % Figure for plotting state of the leg - LEFT
    figure(4)
    clf
    hold on
    axis equal
    axis([-.25 .25 -.25 .1]);
   
    h_OBL = plot([0],[0],'LineWidth',2);
    h_ACL = plot([0],[0],'LineWidth',2);
    h_BDL = plot([0],[0],'LineWidth',2);
    h_CEL = plot([0],[0],'LineWidth',2);
    
    h_footL= plot([0],[0],'k');
    h_desL = plot([0],[0],'k--');
    h_desL.XData=[];
    h_desL.YData=[];
    h_footL.XData=[];
    h_footL.YData=[];
    
    % Define leg length parameters
    l_OA = 0.011; 
    l_OB = 0.042; 
    l_AC = 0.096; 
    l_DE = 0.091;

    p   = [l_OA l_OB l_AC l_DE]';
    
    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data --------- RIGHT LEG ------------
        t = new_data(:,1);          % time
        pos1 = new_data(:,2);       % position
        vel1 = new_data(:,3);       % velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command
        
        pos2 = new_data(:,7);       % position
        vel2 = new_data(:,8);       % velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command
        
        x = -new_data(:,12);        % actual foot position (negative due to direction motors are mounted)
        y = new_data(:,13);         % actual foot position
        xdes = -new_data(:,16);     % desired foot position (negative due to direction motors are mounted)
        ydes = new_data(:,17);      % desired foot position         
        
        % ------ LEFT LEG -------- %
        pos1L = new_data(:,20);       % position
        vel1L = new_data(:,21);       % velocity
        cur1L = new_data(:,22);       % current
        dcur1L = new_data(:,23);      % desired current
        duty1L = new_data(:,24);      % command
        
        pos2L = new_data(:,25);       % position
        vel2L = new_data(:,26);       % velocity
        cur2L = new_data(:,27);       % current
        dcur2L = new_data(:,28);     % desired current
        duty2L = new_data(:,29);     % command
        
        xL = -new_data(:,30);        % actual foot position (negative due to direction motors are mounted)
        yL = new_data(:,31);         % actual foot position
        xdesL = -new_data(:,34);     % desired foot position (negative due to direction motors are mounted)
        ydesL = new_data(:,35);      % desired foot position        
        
        N = length(pos1);
        
        % Update motor data plots - RIGHT
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = -pos1; % switch sign on all plotted values due to direction motors are mounted
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = -vel1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = -cur1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = -dcur1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = -duty1;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = -pos2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = -vel2;
        h23.XData(end+1:end+N) = t;   
        h23.YData(end+1:end+N) = -cur2;
        h24.XData(end+1:end+N) = t;   
        h24.YData(end+1:end+N) = -dcur2;
        h25.XData(end+1:end+N) = t;   
        h25.YData(end+1:end+N) = -duty2;
        
        % Update motor data plots - LEFT
        h1L.XData(end+1:end+N) = t;   
        h1L.YData(end+1:end+N) = -pos1L; % switch sign on all plotted values due to direction motors are mounted
        h2L.XData(end+1:end+N) = t;   
        h2L.YData(end+1:end+N) = -vel1L;
        h3L.XData(end+1:end+N) = t;   
        h3L.YData(end+1:end+N) = -cur1L;
        h4L.XData(end+1:end+N) = t;   
        h4L.YData(end+1:end+N) = -dcur1L;
        h5L.XData(end+1:end+N) = t;   
        h5L.YData(end+1:end+N) = -duty1L;
        
        h21L.XData(end+1:end+N) = t;   
        h21L.YData(end+1:end+N) = -pos2L;
        h22L.XData(end+1:end+N) = t;   
        h22L.YData(end+1:end+N) = -vel2L;
        h23L.XData(end+1:end+N) = t;   
        h23L.YData(end+1:end+N) = -cur2L;
        h24L.XData(end+1:end+N) = t;   
        h24L.YData(end+1:end+N) = -dcur2L;
        h25L.XData(end+1:end+N) = t;   
        h25L.YData(end+1:end+N) = -duty2L;
        
        % Calculate leg state and update plots
        z = [pos1(end) pos2(end) vel1(end) vel2(end)]';
        keypoints = keypoints_leg(z,p);
        
        zL = [pos1L(end) pos2L(end) vel1L(end) vel2L(end)]';
        keypointsL = keypoints_leg(zL,p);
        
        % TODO: add code here to show Jacobian and/or mass matrix as
        % ellipses, controller force vectors and desired force vectors?
        
        rA = keypoints(:,1); 
        rB = keypoints(:,2);
        rC = keypoints(:,3);
        rD = keypoints(:,4);
        rE = keypoints(:,5);
        
        rAL = keypointsL(:,1); 
        rBL = keypointsL(:,2);
        rCL = keypointsL(:,3);
        rDL = keypointsL(:,4);
        rEL = keypointsL(:,5);

        set(h_OB,'XData',[0 rB(1)],'YData',[0 rB(2)]);
        set(h_AC,'XData',[rA(1) rC(1)],'YData',[rA(2) rC(2)]);
        set(h_BD,'XData',[rB(1) rD(1)],'YData',[rB(2) rD(2)]);
        set(h_CE,'XData',[rC(1) rE(1)],'YData',[rC(2) rE(2)]);
        
        set(h_OBL,'XData',[0 rBL(1)],'YData',[0 rBL(2)]);
        set(h_ACL,'XData',[rAL(1) rCL(1)],'YData',[rAL(2) rCL(2)]);
        set(h_BDL,'XData',[rBL(1) rDL(1)],'YData',[rBL(2) rDL(2)]);
        set(h_CEL,'XData',[rCL(1) rEL(1)],'YData',[rCL(2) rEL(2)]);
        
        h_foot.XData(end+1:end+N) = x;
        h_foot.YData(end+1:end+N) = y;
        h_des.XData(end+1:end+N) = xdes;
        h_des.YData(end+1:end+N) = ydes;
        
        h_footL.XData(end+1:end+N) = xL;
        h_footL.YData(end+1:end+N) = yL;
        h_desL.XData(end+1:end+N) = xdesL;
        h_desL.YData(end+1:end+N) = ydesL;
        
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
    start_period                = pre_buffer_time;    % In seconds 
    end_period                  = post_buffer_time;   % In seconds
    
    K_xx                     = gains.K_xx; % Stiffness
    K_yy                     = gains.K_yy; % Stiffness
    K_xy                     = gains.K_xy; % Stiffness

    D_xx                     = gains.D_xx; % Damping
    D_yy                     = gains.D_yy; % Damping
    D_xy                     = gains.D_xy; % Damping
    
    % Specify inputs
    input = [start_period traj_time end_period];
    input = [input angle1_init angle2_init angle1_initL angle2_initL];
    input = [input K_xx K_yy K_xy D_xx D_yy D_xy];
    input = [input duty_max num_cycles];
    input = [input pts_foot(:)' phase_flag]; % final size of input should be 15 + 16 = 32
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 37;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2 a3 a4],'x')
    linkaxes([a1L a2L a3L a4L],'x')
    
end