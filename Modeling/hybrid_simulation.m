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
        
        [dz, u] = dynamics_continuous(t,zout(:,i),ctrl,p,iphase);
        zout(:,i+1) = zout(:,i) + dz*dt;
        
%         % Fix phi with hand constraint
%         phi_solved = phi_solved_climber(zout(:,i+1), p);
%         zout(4,i+1) = phi_solved(1);
%         zout(8,i+1) = phi_solved(2);
        
        % contact constraint force
        zout(5:8,i+1) = discrete_impact_contact(zout(:,i+1), p);
        
        % Update state with new derivatives
        zout(1:4,i+1) = zout(1:4,i) + zout(5:8,i+1)*dt;
        uout(:,i+1) = u; 
        
        % Describe phase (1 = control, 2 = jump, 3 = descend)
        if(zout(1,i+1) > 0.001 && zout(2,i+1) > 0 && iphase == 1) % jump started
            iphase = 2;
        % Change: was zout(1,i+1) but max height == when velocity goes neg
        % // zout(3,i+1)
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
    
    % horizontal force on hand
    Jh = [0, 0, 0, 1];
    M = A_climber(z,p);
    lambda_zh = 1/(Jh * (M \Jh.'));
    F_Ax = lambda_zh*(0 - drA(1));
    qdot = qdot + M \ Jh.' * F_Ax;
    
    % vertical force on foot
    if(rHy < 0 && vHy < 0)     % condition: if y-vel is negative
      Jf  = [1, 0, 0, 0];
%       Ainv = inv(M);
      lambda_z = 1/(Jf * (M \ Jf.'));
      F_z = lambda_z*(0 - vHy);
      qdot = qdot + M \ Jf.'* F_z;
    end
    
end

%% Continuous dynamics
function [dz, u] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,ctrl,iphase);  % get controls at this instant
    
    A = A_climber(z,p);                 % get full A matrix
    b = b_climber(z,u,[0; 0],p);             % get full b vector (z,u,Fc,p)
    
    x = A\b;                % solve system for accelerations (and possibly forces)
    dz(1:4,1) = z(5:8);     % assign velocities to time derivative of state vector
    dz(5:8,1) = x(1:4);     % assign accelerations to time derivative of state vector

end

%% Control
function u = control_laws(t,z,ctrl,iphase)

    if iphase == 1
        u = [BezierCurve(ctrl.T1, t/ctrl.tf);
             BezierCurve(ctrl.T2, t/ctrl.tf)];
        
    else
        
        % PD Control in flight
        th1  = z(2,:);           % leg angles
        th2  = z(3,:);
        dth1 = z(6,:);           % leg angular velocities
        dth2 = z(7,:);

        th1_des = pi/8;          % desired leg angles
        th2_des = pi/4;
        k = 5;                   % stiffness (N/rad)
        b = .5;                  % damping (N/(rad/s))

        u = [-k*(th1-th1_des) - b*dth1;
             -k*(th2-th2_des) - b*dth2]; % apply PD control
    end

end

%% Hand Constraint - Solve for phi
% function phi = get_phi(z,p)
%     % Solve for psi -- quadrilateral with 3 known sides and 2 known angles
%     % take second solution after verifying -- TODO, might be a cleaner way
%     % phi_solved = solve(rA(1) == 0, phi_sym, 'Real', true);  
%     % phi = simplify(phi_solved(2));
%     th1 = z(2);
%     th2 = z(3); 
%     phi = z(4);
%     
%     
% end