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

    tf = x(1); ctrl.tf = x(2); ctrl.T  = x(3:end);
    [t, z, u, ind] = hybrid_simulation(z0, ctrl, p, [0, tf]);
    min_theta = -1*min(z(2,:));
    max_theta = max(z(2,:)) - pi/2;
    com = COM_jumping_leg(z, p);
    y_com_max = max(com(2,:));
    
    cineq = [min_theta, max_theta];
    
    ceq(1) = ctrl.tf - t(ind(1));       % Case 1 (ctrl_tf == t_takeoff)
    ceq(2) = y_com_max - 0.4;           % Case 2 & 3 (y_COM_max == 0.4)

                                                            
% simply comment out any alternate constraints when not in use
    
end