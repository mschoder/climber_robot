function [tout, zout, uout, indices] = hybrid_simulation_1motor(z0,ctrl,p,tspan)
%Inputs:
% z0 - the initial state
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories [y, th1, th2, phi, ... derivs]
% uout - vector of all control trajectories
% indices - vector of indices indicating when each phases ended
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended

    % Initialize time sequence
    t0 = tspan(1); tend = tspan(end);   % set initial and final times
    dt = 0.001;
    num_step = floor((tend-t0)/dt);
    tout = linspace(t0, tend, num_step);
    zout(:,1) = z0;                     % initial conditions
    uout = zeros(2,1);                  % initialize control torques
    iphase_list = 1;
    
    % Discrete step compuation
    for i = 1:num_step-1
        t = tout(i);
        iphase = iphase_list(i);
        
        % Velocity update with dynamics
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,iphase);
        zout(:,i+1) = zout(:,i) + dz*dt;
 
        % contact constraint force
%         zout(5:8,i+1) = discrete_impact_contact(zout(:,i+1), p);
        
        % Position update
        zout(1:4,i+1) = zout(1:4,i) + zout(5:8,i+1)*dt;
        uout(:,i+1) = u; 
        
        % Describe phase (1 = control, 2 = jump, 3 = descend)'
        eps = 0.015;
        if(zout(1,i+1) > z0(1)+eps && zout(5,i+1)+eps > 0 && iphase == 1) % jump started
            iphase = 2;
        elseif(zout(5,i+1) < 0 && iphase == 2) % max height reached; descending
            iphase = 3;
        end
        iphase = 1; %% TODO -- blocking
        iphase_list(i+1) = iphase;
    end
    
    % Find the timestep where iphase changes
    j=1;
    indices = 0;
    for i = 1:num_step-1
        if (iphase_list(i+1) == iphase_list(i))
            indices(j) = indices(j)+1;
        else
            j = j+1;
            indices(j) = 0;
        end
    end
end

%% Discrete Contact
function qdot = discrete_impact_contact(z,p)
    qdot = z(5:8);
    rHy = z(1);
    vHy = z(5);
    kps = keypoints_climber(z,p);
    dkps = dkeypoints_climber(z,p);
    rA = kps(:,1);
    drA = dkps(:,1);
    
    M = A_climber(z,p);
    Jh = jacobian_hand_climber(z,p);
    Jhx = Jh(1,:);
    
%     % horizontal impulse force on hand
%     lambda_zh = 1/(Jhx * (M \ Jhx.'));
%     F_Ax = lambda_zh*(0 - drA(1));
%     qdot = qdot + M \ Jhx.' * F_Ax;
    

    
    % Comment out to turn off hand constraint below
%     qdot = qdot + M \ Jhx.' * F_Ax;

    
    % vertical force on foot
%     if(vHy < 0)     % condition: if y-vel is negative
% %     if(vHy < 0 && rHy < 0)  % ground only contact
%       Jf  = [1, 0, 0, 0];
%       lambda_z = 1/(Jf * (M \ Jf.'));
%       F_z = lambda_z*(0 - vHy);
%       qdot = qdot + M \ Jf.'* F_z; 
%     end
    
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,p,ctrl,iphase);  % get controls at this instant
    u = [-0.05; -0.10];
%     u = [0;0];
    
    A = A_climber(z,p);                 % get full A matrix
    b = b_climber(z,u,[0; 0],p);        % get full b vector (z,u,Fc,p)
    
    QTau_Fh = 0;
    
%     % horizontal spring damper on hand
%     kps = keypoints_climber(z,p);
%     dkps = dkeypoints_climber(z,p);
%     rA = kps(:,1);
%     drA = dkps(:,1);
%     M = A_climber(z,p);
%     Jh = jacobian_hand_climber(z,p);
%     Jhx = Jh(1,:);
%     Kappa = 20; 
%     Dampa = .2;
%     xerr = 0 - rA(1);
%     dxerr = 0 - drA(1);
%     F_Ax = Kappa * xerr + Dampa * dxerr;
%     QTau_Fh = F_Ax * Jhx.';
%     
    QTau_Fy = 0;
    % Vertical s-d on foot
    if(z(5) < 0 && z(1) < 0)  % if vel foot is negative
      Jf  = [1, 0, 0, 0];
      Kf = 2000;
      Df = 5;
      fy_des = 0;
      rfoot = z(1);
      drfoot = z(5);
      yerr = fy_des - rfoot;
      dyerr = 0 - drfoot;
      Fy_foot = Kf * yerr + Df * dyerr;
      QTau_Fy = Fy_foot * Jf';
      QTau_Fy = max(0, QTau_Fy);

    end
    
    % update joint limit torques 
    QTauc = joint_limit_torque(z,p);
    QTauc = [0; 0; 0; 0];
    
    QTauc = QTauc + QTau_Fh + QTau_Fy;
    
    qdd = A\(b + QTauc);      % solve system for accelerations (and possibly forces)
    dz(1:4,1) = z(5:8);       % assign velocities to time derivative of state vector
    dz(5:8,1) = qdd(1:4);     % assign accelerations to time derivative of state vector

end

%% Control
function u = control_laws(t,z,p,ctrl,iphase)

%     % operational space control for motor 2
%     K_x = 150.; % Spring stiffness X
%     K_y = 150.; % Spring stiffness Y
%     D_x = 10.;  % Damping X
%     D_y = 10.;  % Damping Y
%     
%     % desired position of foot is a vertical line, moving down at constant
%     % speed
%     t = 0.001
%     rHd = [0; 0.1*t; 0];  % all in operational space
%     vHd = [0; 0.1; 0];  
%     aHd = [0; 0; 0];
%     
%     % actual position and velocity
%     kps = keypoints_climber(z,p);
%     dkps = dkeypoints_climber(z,p);
%     rH = kps(:,end);
%     vH = dkps(:,end);
%     
%     % Compute virtual force
%     f  = [0; K_y * (rHd(2) - rH(2) ) + D_y * (vHd(2) - vH(2) ); 0];
%     
%     % Task space compensation and feed-forward control
%     J = jacobian_foot_climber(z,p);
%     Jdot = jacobian_dot_foot_climber(z,p);
%     M = A_climber(z,p);
%     V = corr_climber(z,p);
%     G = grav_climber(z,p);
%     qdot = z(5:8);
%     Lam  = pinv(J*pinv(M)*J');
%     Mu   = Lam*J*pinv(M)*V - Lam*Jdot*qdot;
%     Rho  = Lam*J*pinv(M)*G;
%     f_extend = Lam * (aHd + f) + Mu + Rho;
%     
%     % Q3 - variants on above control law -- comment out unused
% %     f_extend = Lam * (aEd(1:2) + f) + Mu;
% %     f_extend = Lam * (aEd(1:2) + f) + Rho;
% %     f_extend = Lam * (f) + Mu + Rho;
% 
%     % Map to joint torques  
%     tau = J' * f_extend;
%     
%     u = -[u1; 0] + tau;
%%%%%%%%%%%%%%%%%%%%%%%%%
%     % PD control for motor 2 - trajectory
%     th2 = z(3,:);
%     dth2 = z(7,:);
%     k = 10;
%     b = .05;
%     
%     tf = 1.0;
%     th2_start = -100/360*2*pi;   % angle range
%     th2_end = -30/360*2*pi;
%     if (t < tf)
%        th2_des = t/tf * (th2_end - th2_start) + th2_start;
%     else
%        th2_des = -30/360*2*pi; % end angle desired
%     end
% 
%     u2 = k*(th2_des - th2) + b*(0 - dth2);

%     if (iphase == 1 && t < ctrl.tf)
%         u = -[BezierCurve(ctrl.T1, t/ctrl.tf);
%              BezierCurve(ctrl.T2, t/ctrl.tf)];
%         fprintf('ubezier: %f\n', u);
%   end
%    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    % PD Control for motor 1 - treat as a spring
    th1  = z(2,:);
    dth1 = z(6,:);
    th1_des = 0/360*2*pi;
    k = 10;                % stiffness (N/rad)
    b = .1;                % damping (N/(rad/s))
    u1 = k*(th1_des - th1) + b*(0 - dth1);
    
    % PD Control for motor 2 - simple spring
    th2  = z(3,:);
    dth2 = z(7,:);
    th2_des = -80/360*2*pi;
    k = 10;                % stiffness (N/rad)
    b = .1;                % damping (N/(rad/s))
    u2 = k*(th2_des - th2) + b*(0 - dth2);
    
%     u = [u1; u2];
    u = [-u1; -u2];
    
    fprintf('t: %f\n', t);
    fprintf('th1: %f ,th1_des: %f, delta: %f\n', th1, th1_des, th1_des - th1);
    fprintf('th2: %f, th2_des: %f, delta: %f\n', th2, th2_des, th2_des - th2);
    fprintf('dth1: %f, dth2: %f\n', dth1, dth2);
    fprintf('phase: %d\n', iphase);
    fprintf('Control U: %f\n', u)

end

%% Joint Limit Constraint
% Constraint s.t. q...
function qdot = joint_limit_constraint(z,p)
    q2_llim = -75/360*2*pi;
    q2_ulim = 45/360*2*pi;
    q3_llim = -135/360*2*pi;
    q3_ulim = -30/360*2*pi;
    q4_llim = -60/360*2*pi;
    q2 = z(2);
    q3 = z(3);
    q4 = z(4);
    qdot = z(5:8);
    
    K = 5;
    D = 0.02;
    
    Jq2 = [0 1 0 0];
    Jq3 = [0 0 1 0];
    Jq4 = [0 0 0 1];
    M = A_climber(z,p);
    
%     if (q2 < q2_llim)
%         disp(q2)
%     end
%     
%     if (q2 < q2_llim && qdot(2) < 0) % q2 lower limit
%         Tq2 = M(2,2)*(0 - Jq2 * qdot);
%         accel = M \ Jq2' * Tq2;
%         qdot = qdot + accel;
%     end
%     
%     if (q2 > q2_ulim && qdot(2) > 0) % q2 upper limit
%         Tq2 = M(2,2)*(0 - Jq2 * qdot);
%         accel = M \ Jq2' * Tq2;
%         qdot = qdot + accel;
%     end
%     
%     if (q3 < q3_llim && qdot(3) < 0) % q3 lower limit
%         Tq3 = M(3,3)*(0 - Jq3 * qdot);
%         accel = M \ Jq3' * Tq3;
%         qdot = qdot + accel;
%     end
%  
%     
%     if (q3 > q3_ulim && qdot(3) > 0) % q3 upper limit
%        Tq3 = M(3,3)*(0 - Jq3 * qdot);
%        accel = M \ Jq3' * Tq3;
%        qdot = qdot + accel;
%     end
    
%     if (q4 < q4_llim && qdot(4) < 0) % llim on gamma
%        Tq4 = M(4,4)*(0 - Jq4 * qdot);
%        accel = M \ Jq4' * Tq4;
%        qdot = qdot + accel;
%     end
%         
    
end

function QTauc = joint_limit_torque(z,p)
    Kappa = 20.0;
    Dampa = 0.05;
    
    q2 = z(2);
    q3 = z(3);
    q4 = z(4);
    qdot = z(5:8);
    
    % known to work
    q2_llim = -30/360*2*pi;  % Theta 1
    q2_ulim = 15/360*2*pi;
    q3_llim = -130/360*2*pi; % Theta 2 %130
    q3_ulim = -30/360*2*pi;
    q4_llim = -60/360*2*pi;  % Gamma
    q4_ulim = 15/360*2*pi;

% % More relaxed
%     q2_llim = -75/360*2*pi;  % Theta 1
%     q2_ulim = 55/360*2*pi;
%     q3_llim = -145/360*2*pi; % Theta 2
%     q3_ulim = -25/360*2*pi;
%     q4_llim = -45/360*2*pi;  % Gamma
%     q4_ulim = 35/360*2*pi;
    
    Tauc2 = 0; Tauc3 = 0; Tauc4 = 0;
    
    if (q2 < q2_llim)
        Tauc2 = max(0, -Kappa * (q2 - q2_llim) - Dampa * qdot(2));
    end   
    
    if (q2 > q2_ulim)
        Tauc2 = min(0, -Kappa * (q2 - q2_ulim) - Dampa * qdot(2));
    end
    
    if (q3 < q3_llim)
        Tauc3 = max(0, -Kappa * (q3 - q3_llim) - Dampa * qdot(3));
    end   
    
    if (q3 > q3_ulim)
        Tauc3 = min(0, -Kappa * (q3 - q3_ulim) - Dampa * qdot(3));
    end
    
    Kappa_g = 20;
    if (q4 < q4_llim)
        Tauc4 = max(0, -Kappa_g * (q4 - q4_llim) - Dampa * qdot(4));
    end
    
    if (q4 > q4_ulim)
        Tauc4 = min(0, -Kappa_g * (q4 - q4_ulim) - Dampa * qdot(4));
    end
    
    QTauc = [0; Tauc2; Tauc3; Tauc4];
    fprintf('QTauc: %f \n', QTauc);
end






