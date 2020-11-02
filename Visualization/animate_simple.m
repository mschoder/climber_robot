function animate_simple(t,z,p, speed)



    axis([-.2 .2 -.2 1])
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',5);
    hold on
    h_leg    = plot([0],[0],'-o',...
                'LineWidth',3,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6); 
    
    p_com = plot([0],[0], '-o','MarkerFaceColor','b',...
                    'MarkerEdgeColor','b');
    com = COM_jumping_leg(z,p);

    tic                                             % start counter
    while toc < t(end)/speed                        % while there's real time left
        tsim = toc*speed;                           % determine the simulation time to draw
        zint = interp1(t',z',tsim', 'linear')';     % interpolate to get coordinates at that time
        com_int = interp1(t',com',tsim','linear');
        draw_lines(zint,p,h_leg,p_com,com_int);     
    end
    draw_lines(z(:,end),p,h_leg,p_com,com_int);
    
end

function draw_lines(z,p, h_leg, p_com, com_int)
    keypoints = keypoints_jumping_leg(z,p);
    h_leg.XData = keypoints(1,:);
    h_leg.YData = keypoints(2,:);
    
    p_com.XData = com_int(1);
    p_com.YData = com_int(2);
    
    drawnow
    axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
    axis([-.2 .2 -.2 1])
end