function f = objective(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state [t, th, dy, dth]
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.
%
% Note: fmincon() requires a handle to an objective function that accepts 
% exactly one input, the decision variables 'x', and returns exactly one 
% output, the objective function value 'f'.  It is convenient for this 
% assignment to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().                
    
    tf = x(1); ctrl.tf = x(2); ctrl.T1 = x(3:5); ctrl.T2 = x(6:8);
    [t, z, u, ind] = hybrid_simulation(z0, ctrl, p, [0, tf]);
    com = COM_climber(z, p);
    
%     int_tau2 = z(5,end);
%     int_tau2 = sum(t.*u.^2);      % alternative implementation
    
    f = -max(com(2,:));                             % negative of COM max height
    
    % alternate objective functions:
%     f = tf;                                         % final time (minimize)
%     f = int_tau2;                                   % minimize T^2 integral
    
end