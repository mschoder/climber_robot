function [cineq, ceq] = constraints_min_energy_walk(x,z0,p,tf0)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().


    q2_llim = -70/360*2*pi;  % theta 1
    q2_ulim = 60/360*2*pi;
    q3_llim = -130/360*2*pi; % theta 2
    q3_ulim = -25/360*2*pi;            
    q4_llim = -45/360*2*pi;  % gamma
    q4_ulim = 35/360*2*pi;


    tf = x(1); ctrl.tf = x(2); ctrl.T1 = x(3:5); ctrl.T2 = x(6:8);
    
    [t, z, u, ind] = hybrid_simulation(z0, ctrl, p, [0, tf]);
    
    % Joint limit constraints (satisfied when < 0)
    min_th1 = q2_llim - min(z(2,:)); 
    max_th1 = max(z(2,:)) - q2_ulim;
    min_th2 = q3_llim - min(z(3,:));
    max_th2 = max(z(3,:)) - q3_ulim;
    min_gam = q4_llim - min(z(4,:));
    max_gam = max(z(4,:)) - q4_ulim;
    
    com = COM_climber(z, p);
    com0 = COM_climber(z0, p);
    y_com_max = max(com(2,:));
    
    ctrl_time = ctrl.tf - t(ind(1));           % (ctrl_tf < t_takeoff)
    min_height = 0.07 - (y_com_max - com0(2)); % min height diff height
    
    cineq = [min_th1, min_th2, max_th1, max_th2, min_gam, max_gam, min_height, ctrl_time];

    % desired ending pose
    th1end_des = 10/360*2*pi;
    th2end_des = -25/360*2*pi;
    tmpz = [0, th1end_des, th2end_des, 0, 0, 0, 0, 0]';
    gamend_des = gam_solved_climber(tmpz, p);
    ceq(1) = z(2,end) - (th1end_des);  % start and end angles (pose) same
    ceq(2) = z(3,end) - (th2end_des);
    ceq(3) = z(4,end) - (gamend_des(1));
    
    ceq(4:6) = z(6:8,end);            % end angular velocities zero after step
    ceq(7) = tf - tf0;               % make tf state var irrelevant
    
    ceq(8) = ctrl.tf - t(ind(1));              % Case 1 (ctrl_tf == t_takeoff)
%     ceq(2) = 0.15 - (y_com_max - com0(2));   % (y_COM_max moves exact amt)

    
end