function animate_side(t, z, p, speed)

%     axis equal
    pbaspect([1 1 1])
    
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',3);
    hold on
    v_rope = plot([0 0], [0 1], 'k-', 'LineWidth',5);
 
    % init base plot objs
    
    hBC = draw_member1([0 0], [0 0], 'b');  % BC
    hCE = draw_member1([0 0], [0 0], 'r');  % CE
    hDF = draw_member1([0 0], [0 0], 'g');  % DF
    hEG = draw_member1([0 0], [0 0], 'magenta');  % EG
    hFH = draw_member1([0 0], [0 0], 'k');  % FH
    hAB = draw_member1([0 0], [0 0], 'cyan');  % AB
    
    h_side = [hAB hBC hCE hDF hEG hFH];
    axis([-.4, 0.8, -0.8, 0.8])
    
    tic
    while toc < t(end)/speed
        tsim = toc*speed;
        zint = interp1(t',z',tsim', 'linear')';
        draw_side(zint,p,h_side);
    end
    
end
   

function draw_side(z,p,h_side)
    kp = keypoints_climber(z, p);
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

function h = draw_member1(pt1, pt2, color)

    h = plot([pt1(1) pt2(1)], [pt1(2) pt2(2)],...
                '-o',...
                'Color', color,...
                'LineWidth',6,...
                'MarkerEdgeColor','blue',...
                'MarkerFaceColor','blue',...
                'MarkerSize',4); 

end

function draw_member2(pt1, pt2)

    plot([pt1(1) pt2(1)], [pt1(2) pt2(2)],...
                '-o',...
                'Color', 'cyan',...
                'LineWidth',6,...
                'MarkerEdgeColor','magenta',...
                'MarkerFaceColor','magenta',...
                'MarkerSize',4); 

end
