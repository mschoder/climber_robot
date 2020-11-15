function [tout, zout, uout, indices] = hybrid_simulation(z0,ctrl,p,tspan)
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
        zout(5:8,i+1) = discrete_impact_contact(zout(:,i+1), p);
        
%         % Update joint limit constraints - impulse
%         qdot = joint_limit_constraint(zout(:,i+1), p);
%         zout(5:8,i+1) = qdot;
        
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
    
    % horizontal spring damper on hand
    Kappa = 0.5;
    Dampa = 0.03;
    xerr = 0 - rA(1);
    dxerr = 0 - drA(1);
    F_Ax = Kappa * xerr + Dampa * dxerr;
%     disp(xerr)
%     disp(dxerr)
%     disp(F_Ax)
    qdot = qdot + M \ Jhx.' * F_Ax;
    
    % vertical force on foot
    if(vHy < 0)     % condition: if y-vel is negative
      Jf  = [1, 0, 0, 0];
      lambda_z = 1/(Jf * (M \ Jf.'));
      F_z = lambda_z*(0 - vHy);
      qdot = qdot + M \ Jf.'* F_z;
    end
    
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,ctrl,iphase);  % get controls at this instant
%     u = [0; 0];
%     u = [-0.8; -0.8];
    
    A = A_climber(z,p);                 % get full A matrix
    b = b_climber(z,u,[0; 0],p);        % get full b vector (z,u,Fc,p)
    
    % update joint limit torques 
    QTauc = [0; 0; 0; 0];
    QTauc = joint_limit_torque(z,p);
    
    qdd = A\(b + QTauc);      % solve system for accelerations (and possibly forces)
    dz(1:4,1) = z(5:8);       % assign velocities to time derivative of state vector
    dz(5:8,1) = qdd(1:4);     % assign accelerations to time derivative of state vector

end

%% Control
function u = control_laws(t,z,ctrl,iphase)

%         % PD Control
%         th1  = z(2,:);           % leg angles
%         th2  = z(3,:);
%         dth1 = z(6,:);           % leg angular velocities
%         dth2 = z(7,:);
% 
%         th1_des = -pi/8;          % desired leg angles
%         th2_des = -pi/4;
%         k = 20;                   % stiffness (N/rad)
%         b = .5;                  % damping (N/(rad/s))
% 
%         u = [-k*(th1-th1_des) - b*dth1;
%              -k*(th2-th2_des) - b*dth2]; % apply PD control
%         u = -u;

    if iphase == 1
        u = [BezierCurve(ctrl.T1, t/ctrl.tf);
             BezierCurve(ctrl.T2, t/ctrl.tf)];
    else
        
        % PD Control in flight
        th1  = z(2,:);           % leg angles
        th2  = z(3,:);
        dth1 = z(6,:);           % leg angular velocities
        dth2 = z(7,:);

        th1_des = -pi/8;         % desired leg angles
        th2_des = -pi/4;
        k = 20;                  % stiffness (N/rad)
        b = .5;                  % damping (N/(rad/s))

        u = [-k*(th1-th1_des) - b*dth1;
             -k*(th2-th2_des) - b*dth2]; % apply PD control
    end
    u = -u; %% Hack -- fixes sign issue for now

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
    Kappa = 5.0;
    Dampa = 0.05;
    
    q2 = z(2);
    q3 = z(3);
    q4 = z(4);
    qdot = z(5:8);
    
    % known to work
    q2_llim = -75/360*2*pi;  % Theta 1
    q2_ulim = 45/360*2*pi;
    q3_llim = -135/360*2*pi; % Theta 2
    q3_ulim = -30/360*2*pi;
    q4_llim = -30/360*2*pi;  % Gamma
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
    
    if (q4 < q4_llim)
        Tauc4 = max(0, -Kappa * (q4 - q4_llim) - Dampa * qdot(4));
    end
    
    if (q4 > q4_ulim)
        Tauc4 = min(0, -Kappa * (q4 - q4_ulim) - Dampa * qdot(4));
    end
    
    QTauc = [0; Tauc2; Tauc3; Tauc4];
end






