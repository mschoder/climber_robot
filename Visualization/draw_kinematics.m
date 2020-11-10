function draw_kinematics()
% Draw simple model to validate kinematics
    p = parameters();
    z = [0.2, pi/8, pi/4, pi/12, 0, 0, 0, 0]';
    
    % Update phi with hand constraint
    gam_solved = gam_solved_climber(z, p);
    z(4) = gam_solved(1);
    z(8) = gam_solved(2);

    kp = keypoints_climber(z, p);
    
    
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',3);
    hold on
    v_rope = plot([0 0], [0 1], 'k-', 'LineWidth',5);
    
    draw_member1(kp(:,1), kp(:,2));  % AB
    draw_member1(kp(:,2), kp(:,3));  % BC
    draw_member1(kp(:,3), kp(:,5));  % CE
    draw_member1(kp(:,4), kp(:,6));  % DF
    draw_member1(kp(:,5), kp(:,7));  % EG
    draw_member1(kp(:,6), kp(:,8));  % FH
    
    axis equal
    axis([-.2, 1.0, -.2, 1])

end

function h = draw_member1(pt1, pt2)

    h = plot([pt1(1) pt2(1)], [pt1(2) pt2(2)],...
                '-o',...
                'Color', 'b',...
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



