function [is_satisfied, res_index] = plotandcheck(input, rgb, linestyle, interval, plot_tube_body, Xg_cell, Xg_3d, avoid_set, goal_set)
    linewidth = 1.5;
    ts = 0.1;
    QTOL = 1e-8;
    Et  = [];
    Ex  = [];
    Ey  = [];
    N = length(input);
    
    % Compute tube boundary points.
    Ept_cell = {};
    for t = 1:N
        % Approx. Reach Tube.
        [q,Q] = double(input(t));
        Q     = Q + eye(2)*QTOL;
        Ept   = Ellpt3d(ts*(t-1),q,Q);
        Et    = [Et ; Ept(1,:)];
        Ex    = [Ex ; Ept(2,:)];
        Ey    = [Ey ; Ept(3,:)];
        Ept_cell{end+1} = Ept;
    end

    % Plot tube body.
    if plot_tube_body
        % Approx. Reach Tube.
        patchET = patch(surf2patch(Et,Ex,Ey,Ey*0));
        patchET.EdgeColor = rgb;
        patchET.FaceColor = rgb;
        patchET.EdgeAlpha = 0;
        patchET.FaceAlpha = 0.2;
    end

    % Plot tube boundary.
    figure
    hold on;
    for t = 1:N
        if mod(t,interval) == 1 || interval == 1
            % Ground Truth
            Ept_tmp = Ept_cell{t};
            patchE  = patch( Ept_tmp(1,:),Ept_tmp(2,:),Ept_tmp(3,:),rgb);
            patchE.EdgeColor = rgb;
            patchE.FaceAlpha = 0;
            patchE.LineWidth = linewidth;
            patchE.LineStyle = linestyle;
            
            % Reach-SDP Result
            if ~isempty(Xg_cell)
                Xg = Xg_cell{t};
                XgK = boundary(Xg(:,1),Xg(:,2),0.1);
                Xg = Xg(XgK,:)';
                Xgt_bd = [repmat(ts*(t-1),1,length(Xg)); Xg(1,:); Xg(2,:)];
                plot3(Xgt_bd(1,:),Xgt_bd(2,:),Xgt_bd(3,:),'b-',...
                    'LineWidth',linewidth)
            end
        end
    end
    grid on
    view(90,0)
    hold off;  
    savefig('./output/6d_plot.fig')
    
    is_satisfied = 0;
    goal_violation = 0;
    rule_violation = 0;
    res_index = 0;
    
    
    figure, hold on, view(3)        % Display the result
    set(gcf, 'Position', get(gcf, 'Position').*[0 0 1.5 1.5])
    DT = delaunayTriangulation(goal_set);
    [S.faces, S.vertices] = freeBoundary(DT);
    patch(S,'FaceColor','k','FaceAlpha',0.1)
    
    DT_avoid = delaunayTriangulation(avoid_set);
    [Sa.faces, Sa.vertices] = freeBoundary(DT_avoid);
    patch(Sa, 'FaceColor','b', 'FaceAlpha', 0.3)
    
    for t = 1:N
        Xg3d = Xg_3d{t};
        Xg3dK = boundary(Xg3d(:,1),Xg3d(:,2), Xg3d(:,3));
        Xg3dt = Xg3d(Xg3dK,:);
        points = Xg3dt;
        
        in1 = in_polyhedron(S, Xg3dt);
        avoid_in = in_polyhedron(Sa, Xg3dt);
        
        if numel(points(avoid_in)) ~= 0
            rule_violation = 1;
            plot3(points(:,1),points(:,2),points(:,3),'r')
        else
            rule_violation = 0;
            if numel(points(~in1)) ~= 0
                goal_violation = 1;
                plot3(points(:,1),points(:,2),points(:,3),'color', [0.9290 0.6940 0.1250])
            else
                goal_violation = 0;
                plot3(points( in1,1),points( in1,2),points( in1,3),'green')
            end
        end
        axis image 
        
        if rule_violation == 0 && goal_violation == 0
            disp('Reach-SDP Satisfied');
            is_satisfied = 1;
            res_index = t;
            return
        end
        %message = ['Verification Process N = ', num2str(t)];
        %disp(message);
        %m2 = ['rule_violate->',num2str(rule_violation), '; goal->', num2str(goal_violation)];
        %disp(m2);
    end
    hold off
    savefig('./output/Verification_6d_result.fig')
    
    if is_satisfied == 0
        disp('Reach-SDP Not Satisfied');
        return
    end
    
end

function Ept3d = Ellpt3d(t, q, Q)
    theta = 0: 0.01*pi: pi*2;
    v     = [ cos(theta) ; sin(theta) ];
    Ecpt  = sqrtm(Q+eye(2)*1e-6)*v;
    Ept   = Ecpt + repmat(q,1,length(theta));
    Ept3d = [repmat(t,1,length(theta));Ept ];
end