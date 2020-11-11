function [cineq, ceq] = constraints(x,z0,p)
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

    q2_llim = -75/360*2*pi;
    q2_ulim = 30/360*2*pi;
    q3_llim = -135/360*2*pi;
    q3_ulim = -30/360*2*pi;
    q4_llim = -30/360*2*pi;
    q4_ulim = 15/360*2*pi;

    tf = x(1); ctrl.tf = x(2); ctrl.T1 = x(3:5); ctrl.T2 = x(6:8);
    [t, z, u, ind] = hybrid_simulation(z0, ctrl, p, [0, tf]);
    
    % Joint limit constraints (satisfied when < 0)
    min_th1 = q2_llim - min(z(2,:)); 
    max_th1 = max(z(2,:)) - q2_ulim;
    min_th2 = q3_llim - min(z(3,:));
    max_th2 = max(z(3,:)) - q3_ulim;
    min_gam = q4_llim - min(z(4,:));
    max_gam = max(z(4,:)) - q4_ulim;
    
    
%     com = COM_climber(z, p);
%     y_com_max = max(com(2,:));
    
    cineq = [min_th1, min_th2, max_th1, max_th2, min_gam, max_gam];
    
    ceq(1) = ctrl.tf - t(ind(1));       % Case 1 (ctrl_tf == t_takeoff)
    
%     phi_solved = phi_solved_climber(z, p);
%     ceq(2) = phi_solved(1) - z(4);
%     ceq(2) = y_com_max - 0.4;           % Case 2 & 3 (y_COM_max == 0.4)

    
end