close all
clc
%% Bezier Curve Test

t = 0;
i = 1;

% Control Points (Please try many different ones)
ctrl_pt = [0, 2.5, 0];

% The time locations of each control points
ctrl_pt_x = linspace(0,1, length(ctrl_pt));

for d = 0:0.01:1 
    u(i) = BezierCurve(ctrl_pt, d);
    t(i) = d;
    i = i+1;
end

figure
plot(t, u)
hold on
plot(ctrl_pt_x, ctrl_pt, 'o');
hold off