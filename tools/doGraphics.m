% Process and plot all of the loop graphics.

function doGraphics(rob, World, Graphics, Obstacles_mov, AxisDim)

        if ~isempty(World.scan_global)
            set(Graphics.Scan, 'xdata', World.scan_global(1,:), 'ydata', World.scan_global(2,:))
        else
            set(Graphics.Scan, 'xdata', [], 'ydata', [])
        end
        
        % Draw robot poses
        TrueRobTri = rob.computeTriangle('true');
        RobTri = rob.computeTriangle;
        set(Graphics.TrueRob, 'xdata', TrueRobTri(1, :), 'ydata', TrueRobTri(2, :));
        set(Graphics.Rob, 'xdata', RobTri(1, :), 'ydata', RobTri(2, :));

        % Draw landmarks
        lids = find(World.l(1,:));
        lx = World.x(World.l(1,lids));
        ly = World.x(World.l(2,lids));
        set(Graphics.Lmk, 'xdata', lx, 'ydata', ly);
        
        % Draw landmark covariance ellipses
        for lid = lids
            le = World.x(World.l(:,lid));
            LE = World.P(World.l(:,lid),World.l(:,lid));
            [X,Y] = cov2elli(le,LE,3,16);
            set(Graphics.Ellipse(lid),'xdata',X,'ydata',Y);
        end
        
        % Draw robot position covariance ellipse
        if World.t > 1
            re = World.x(World.r(1:2));
            RE = World.P(World.r(1:2),World.r(1:2));
            [X,Y] = cov2elli(re,RE,3,16);
            set(Graphics.RobCov,'xdata',X,'ydata',Y);
        end

        % Draw landmark sensor lines
        if ~isempty(Graphics.lmks_visible)
            vis_lmks_loci = zeros(2, 2 * length(Graphics.lmks_visible));
            for i = 1 : length(Graphics.lmks_visible)
                k = Graphics.lmks_visible(i);
                vis_lmks_loci(:, 2*i-1) = rob.R(1:2);
                %vis_lmks_loci(:, 2*i) = World.W(:, k);
                vis_lmks_loci(:, 2*i) = invScanPoint(rob.R, Graphics.lmks_all(:, k));
            end        
            set(Graphics.LmkLines, 'xdata', vis_lmks_loci(1, :), 'ydata', vis_lmks_loci(2, :)); % Reset lmk lines
        else
            set(Graphics.LmkLines, 'xdata', [], 'ydata', []);
        end
        
        % Plot moving obstacles
        for i = 1:length(Obstacles_mov)
            Obstacles_mov(i).plot;
        end
        
        axis([-AxisDim AxisDim -AxisDim AxisDim])

        drawnow;
end