%Staggered or bounding gait
bounding = 1;

% TARGET POINTS
initial_foot_pts = [ -0.096 ; -0.14];
delta = .14; %Don't exceed 0.09

%Set up Bezier curve for right foot points
a = linspace(initial_foot_pts(2), initial_foot_pts(2) + delta,3);
a = horzcat( a , flip(a(1:length(a)-1)) );
b = linspace(initial_foot_pts(2), initial_foot_pts(2) - delta,3);
b = horzcat( b , flip(b(1:length(b)-1)) );
Right_pts = [ initial_foot_pts(1)*ones(1,8) ;  horzcat(a,b(2:length(b)-1))]

%Set up Bezier curve for left foot points
if bounding
    Left_pts = [-1*Right_pts(1,:); Right_pts(2,:) ]
else
    Left_pts = [-1*Right_pts(1,:) ; horzcat( Right_pts(2,1) , ...
        flip(Right_pts(2,2:length(Right_pts)))) ]
end