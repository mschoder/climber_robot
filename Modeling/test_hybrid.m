

p = parameters();
z0 = [0, pi/8, pi/4, pi/12, 0, 0, 0, 0]';
gs = gam_solved_climber(z0, p);
z0(4) = gs(1);
z0(8) = gs(2);
tspan = [0 0.035];
ctrl.tf = 0.40;
ctrl.T1 = [0.2 1.8 2.0];
ctrl.T2 = [0.2 1.8 2.0];

[tout, zout, uout, indices] = hybrid_simulation(z0, ctrl, p, tspan);

%% Animate
speed = 0.005;
animate_test(tout, zout, p, speed);


function animate_test(t, z, p, speed)

%     axis equal
    pbaspect([1 1 1])
    
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',3);
    hold on
    v_rope = plot([0 0], [0 1], 'k-', 'LineWidth',5);
 
    % init base plot objs
    hAB = draw_member1([0 0], [0 0]);  % AB
    hBC = draw_member1([0 0], [0 0]);  % BC
    hCE = draw_member1([0 0], [0 0]);  % CE
    hDF = draw_member1([0 0], [0 0]);  % DF
    hEG = draw_member1([0 0], [0 0]);  % EG
    hFH = draw_member1([0 0], [0 0]);  % FH
    
    h_side = [hAB hBC hCE hDF hEG hFH];
    axis([-.2, 1.0, -.2, 1])
    
    tic
    while toc < t(end)/speed
        disp(toc)
        tsim = toc*speed;
        zint = interp1(t',z',tsim', 'linear')';
        draw_side(zint,p,h_side);
    end
    
end
   

function draw_side(z,p,h_side)
    kp = keypoints_climber(z, p);
%     [hAB, hBC, hCE, hDF, hEG, hFH] = h_side;
    update_member(h_side(1), kp(:,1), kp(:,2));
    update_member(h_side(2), kp(:,2), kp(:,3));
    update_member(h_side(3), kp(:,3), kp(:,5));
    update_member(h_side(4), kp(:,4), kp(:,6));
    update_member(h_side(5), kp(:,5), kp(:,7));
    update_member(h_side(6), kp(:,6), kp(:,8));
    drawnow
end

function update_member(h, pt1, pt2)
    h.XData = [pt1(1) pt2(1)];
    h.YData = [pt1(2) pt2(2)];
end

function h = draw_member1(pt1, pt2)

    h = plot([pt1(1) pt2(1)], [pt1(2) pt2(2)],...
                '-o',...
                'Color', 'green',...
                'LineWidth',8,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6); 

end

function draw_member2(pt1, pt2)

    plot([pt1(1) pt2(1)], [pt1(2) pt2(2)],...
                '-o',...
                'Color', 'cyan',...
                'LineWidth',8,...
                'MarkerEdgeColor','magenta',...
                'MarkerFaceColor','magenta',...
                'MarkerSize',6); 

end