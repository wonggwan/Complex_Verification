function patchelltube(input, rgb, linestyle, interval, plot_tube_body, Xg_cell)
    linewidth = 1.5;
    ts = 0.1;

    QTOL = 1e-8;

    close all;
    figure(1)
    hold on;

    Et  = [];
    Ex  = [];
    Ey  = [];
    N = length(input);

    % Compute tube boundary points.
    Ept_cell = {};
    for t = 1:N
        %   Approx. Reach Tube.
        [q,Q] = double(input(t));
        Q     = Q + eye(2)*QTOL;
        Ept   = Ellpt3d(ts*(t-1),q,Q)
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
    for t = 1:N
        if mod(t,interval) == 1 || interval == 1
            % Approx. Reach Tube.
            % Ground Truth
            Ept_tmp = Ept_cell{t};
            patchE  = patch( Ept_tmp(1,:),Ept_tmp(2,:),Ept_tmp(3,:),rgb);
            patchE.EdgeColor = rgb;
            patchE.FaceAlpha = 0;
            patchE.LineWidth = linewidth;
            patchE.LineStyle = linestyle;

            % Exact Reach Tube.
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
end


function Ept3d = Ellpt3d(t, q, Q)
    theta = 0: 0.01*pi: pi*2;
    v     = [ cos(theta) ; sin(theta) ];
    Ecpt  = sqrtm(Q+eye(2)*1e-6)*v;
    Ept   = Ecpt + repmat(q,1,length(theta));
    Ept3d = [repmat(t,1,length(theta));Ept ];
end

