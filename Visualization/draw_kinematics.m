function draw_kinematics(z, p)
% Draw simple model to validate kinematics

    kp = keypoints_climber(z, p);
    
    
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',3);
    hold on
    v_rope = plot([0 0], [0 1], 'k-', 'LineWidth',5);
    
    draw_member(kp(:,1), kp(:,2));  % AB
    draw_member(kp(:,2), kp(:,3));  % BC
    draw_member(kp(:,3), kp(:,5));  % CE
    draw_member(kp(:,4), kp(:,6));  % DF
    draw_member(kp(:,5), kp(:,7));  % EG
    draw_member(kp(:,6), kp(:,8));  % FH
    
    axis equal
    axis([-.2, 1.0, -.2, 1])
    
    
    


end

function draw_member(pt1, pt2)

    plot([pt1(1) pt2(1)], [pt1(2) pt2(2)],...
                '-o',...
                'LineWidth',8,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6); 

end



