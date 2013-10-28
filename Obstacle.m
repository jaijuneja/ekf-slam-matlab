classdef Obstacle < handle
    properties (GetAccess = public)
        % Sensor properties
        vertices    % Vertices of object
        graphics
        velocity 
    end
    properties (GetAccess = protected)
        edges
    end
    
    methods
        
        function obs = Obstacle(vertices, velocity, varargin)	% Constructor method
            % Default values
            if nargin == 2
                obs.vertices = vertices;
                obs.velocity = velocity;
            else
                obs.vertices = [1, 1; 1, -1];
                obs.velocity = [0; 0];
            end
            if ~isempty(obs.vertices)
                obs.edges = obs.getEdges;
            end
        end
        
        % For a given sensor and robot pose, return the measurements of a
        % given obstacle
        % Output:
        %   meas_sen        :   vector of range-bearing sensor measurements
        function measurements = getMeasured(obs, sen, rob)
            % Determine which line segments of obstacle (if any) are in
            % range:
            n_segments = size(obs.vertices, 2)-1;
            n_visible_segments = 0;
            visible_segments = zeros(1, n_segments);
            for i = 1:n_segments
                closest_point = pointToLine([rob.R(1:2); 0], ...
                    [obs.vertices(:, i); 0], [obs.vertices(:, i+1); 0]);
                bearing1 = abs(bearingToPoint(rob.R, obs.vertices(:, i)));
                bearing2 = abs(bearingToPoint(rob.R, obs.vertices(:, i+1)));
                if closest_point < sen.range && ...
                        (bearing1 < sen.fov/2 || bearing2 < sen.fov/2)
                    visible_segments(i:i+1) = 1;
                    n_visible_segments = n_visible_segments + 1;
                end
            end
            
            visible_vertices = find(visible_segments);
            
            if isempty(visible_vertices)
                measurements = [];
            else
            
                threshold = 3 * norm(getInvMeasurement(sen.noise));

                % Generate vector of sensor laser lines in global frame
                sensor_lines = sen.generateEmptyScan;
                sensor_lines = invScanPoint(rob.R, sensor_lines);

                % Number of measurements per sensor sweep;
                n_measurements = size(sensor_lines, 2);

                % Initialise vector of zeros to store measurements
                meas_global = zeros(3, n_measurements * n_visible_segments);
                
                index = 0;
                for i = 1:n_visible_segments
                    if visible_vertices(i) == visible_vertices(i+1) - 1
                        obstacle_line = obs.vertices(:, visible_vertices(i):visible_vertices(i+1));
                        for j = 1:n_measurements
                            index = index + 1;
                            sensor_line = [rob.R(1:2) sensor_lines(:, j)];
                            meas_global_tmp = getIntersection(sensor_line, obstacle_line, threshold);
                            if ~isempty(meas_global_tmp)
                                meas_global(:, index) = [j; meas_global_tmp];
                            end
                        end
                    end
                end
                meas_global = meas_global(:, meas_global(1, :) ~= 0);
                measurements = zeros(size(meas_global));

                if ~isempty(meas_global)
                    % Transform to range-bearing measurement in local frame
                    measurements(2:3, :) = scanPoint(rob.R, meas_global(2:3, :));
                    measurements(1, :) = meas_global(1, :);
                else
                    measurements = [];
                end
            end
        end        
        
        function plot(obs, axisHandle)
            if isempty(obs.graphics)
                obs.graphics = line(...
                    'LineWidth', 2, ...
                    'parent',axisHandle, ...
                    'marker','x', ...
                    'xdata',[], ...
                    'ydata',[]);
            end
            if isempty(obs.vertices)
                set(obs.graphics, 'xdata', [], 'ydata', []);
            else
                set(obs.graphics, ...
                    'xdata', obs.vertices(1, :), 'ydata', obs.vertices(2, :));
            end
        end
        
        % Obstacles is an array of Obstacle objects
        function move(obs, world_size)
            vertices_old = obs.vertices;
            obs.vertices = [obs.vertices(1,:) + obs.velocity(1); ...
                obs.vertices(2,:) + obs.velocity(2)];
            obs.edges = obs.getEdges;
            
            if obs.edges(1, 1) < -world_size || obs.edges(1, 2) > world_size
                obs.velocity(1) = -obs.velocity(1); % Bounce off wall
            end
            if obs.edges(2, 1) < -world_size || obs.edges(2, 2) > world_size
                obs.velocity(2) = -obs.velocity(2);
            end
            
            obs.vertices = [vertices_old(1,:) + obs.velocity(1); ...
            vertices_old(2,:) + obs.velocity(2)];
            obs.edges = obs.getEdges;
        end
        
        function edges = getEdges(obs)
            edges = [min(obs.vertices(1,:)) max(obs.vertices(1,:)); ...
                min(obs.vertices(2,:)) max(obs.vertices(2,:))];       
        end
        
    end
end