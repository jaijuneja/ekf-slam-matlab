% Author: Jai Juneja
% Date: 13/02/2013
%
% Robot class contains properties and functions relating to the robot.
classdef Robot < handle
    properties
        % Robot properties
        wheelbase       % Width of wheel base (dist. between opposite wheels)
        length          % Length of robot
        R               % True pose [x y a]
        r               % Estimated pose [x y a]
        q               % True system noise
        u               % Control vector [dx da]
        maxspeed        % Maximum speed (m/s)
        min_turn_rad    % Turning radius (m)
        curr_wpt        % Current waypoint reached
    end
    
    methods
        
        function rob = Robot                        % Constructor method
            % Default values
            rob.wheelbase	=   1;
            rob.length      =   1.5;
            rob.R           =   [0, 0, 0]';
            rob.r           =   rob.R;
            rob.q           =   [0.02;pi/180];       % 2cm, 1 deg
            rob.u           =   [0.1; 0.02];
            rob.maxspeed    =   1.4;                % 1.4m/s ~ 5km/h
            rob.min_turn_rad=   0.5;
            rob.curr_wpt    =   1;
        end

        % Move the robot and obtain new pose and Jacobians.
        %
        % Inputs:
        %   rob             :   Robot object
        %   n               :   Noise (n = [nx na]')
        %   Optional:
        %       strRobType  :   Robot pose to move (true or estimated)
        %
        % Outputs:
        %   r_new           :   New position
        %   RNEW_r          :   Jacobian of r_new wrt. r
        %   RNEW_u          :   Jacobian of r_new wrt. u
        function [r_new, RNEW_r, RNEW_u] = move(rob, n, strRobType)
            
            if nargin < 3
                % Robot position (p = [x y]') and orientation (a):
                pose = rob.r; a = rob.r(3);
            elseif strcmp(strRobType, 'true')
                pose = rob.R; a = rob.R(3);
            else
                error('strRobType can only take the value "true"');
            end

            % Control input:
            dx = rob.u(1) + n(1);
            da = rob.u(2) + n(2);
            dp = [dx; 0];

            % Determine new orientation
            a_new = a + da;    
            % Ensure that angle is between -pi and pi
            a_new = getPiToPi(a_new);

            if nargout == 1
                % Translate robot to new position in global frame
                p_new = transToGlobal(pose, dp);

            else    % Get Jacobians

                % Translate robot to new position in global frame
                [p_new, PNEW_r, PNEW_dp] = transToGlobal(pose, dp);

                % Jacobians of r_a_new:
                ANEW_a = 1;
                ANEW_da = 1;

                % Hence, Jacobians of r_new:
                RNEW_r = [PNEW_r; 0 0 ANEW_a];

                % We define the perturbation such that it is in the same space
                % as control u. This means that there is uncertainty in how much
                % the robot advances and how much it turns. This means that the
                % Jacobians of RNEW with respect to u are the same as the Jacobian
                % of RNEW with respect to n. Less coding necessary, so a good
                % option.

                RNEW_u = [PNEW_dp(:, 1) zeros(2, 1); 0 ANEW_da];

            end

            r_new = [p_new; a_new];
            
        end

        % Return the vertices of a triangle of dimensions base*length, 
        % centred at (x, y) and with orientation a.
        %
        % Inputs:
        %   r = [x y a]'        :   local co-ordinate frame of robot
        %   base                :   size of triangle base
        %   length              :   length of triangle
        %
        % Outputs:
        %   p = [x1 x2 x3 x4;
        %        y1 y2 y3 y4]	:   vertices of resulting triangle
        function p = computeTriangle(rob, strRobType, r_pos)
            
            if nargin < 2
                pose = rob.r;
            elseif nargin < 3 && strcmp(strRobType, 'true')
                pose = rob.R;
            elseif nargin < 4
                pose = r_pos;
            else
                error('strRobType can only take the value "True"');
            end
            
            p = zeros(2, 4);

            p(:, 1) = [-rob.length/3; -rob.wheelbase/2];
            p(:, 2) = [2*rob.length/3; 0];
            p(:, 3) = [-rob.length/3; rob.wheelbase/2;];

            for i = 1:3
                p(:, i) = transToGlobal(pose, p(:, i));
            end

            % We include a fourth point that is equal to the first point in the
            % triangle. This allows us to 'close' the triangle when plotting it
            % using the line function.
            p(:, 4) = p(:, 1);

        end
        
        function steer(rob, Wpts)
            % Determine if current waypoint reached
            wpt = Wpts(:, rob.curr_wpt);
            % Distance from current waypoint
            dist = sqrt((wpt(1)-rob.R(1))^2 + (wpt(2)-rob.R(2))^2);
            if dist < rob.min_turn_rad
                if rob.curr_wpt < size(Wpts, 2)
                    rob.curr_wpt = rob.curr_wpt + 1;
                else
                    rob.curr_wpt = 1; % Go back to the start
                end
                wpt = Wpts(:, rob.curr_wpt);
            end
            % Change in robot orientation to face current waypoint
            da = getPiToPi(atan2(wpt(2)-rob.R(2), wpt(1)-rob.R(1))-rob.R(3));
            % Amount by which to perturb current robot control:
            dda = da - rob.u(2);
            du = [0; dda];
            rob.setControl(du);
        end

        % Perturb control vector by an amount du = [accel_x; accel_a].
        function u = setControl(rob, du)
            u = rob.u + du;
            % Make sure that angular control is between -pi and pi
            u(2) = getPiToPi(u(2));
            if abs(u(1)) > rob.maxspeed
                % Adjust speed so that max speed isn't exceeded
                u(1) = sign(u(1)) * rob.maxspeed;
            end
            turning_radius = u(1)/u(2);     % Using formula v = omega * r
            if abs(turning_radius) < rob.min_turn_rad
                % Adjust angular velocity so that min turning radius isn't
                % exceeded
            	u(2) = sign(u(2)) * abs(u(1)) / rob.min_turn_rad;
            end
            
            rob.u = u;
        end
    end
end