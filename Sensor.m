% Author: Jai Juneja
% Date: 13/02/2013
%
% Sensor class contains properties and functions relating to the sensors.
classdef Sensor < handle
    properties
        % Sensor properties
        range           % Range of laser sensor (metres)
        range_min       % Minimum distance of sensor reading
        fov             % Angle over which sensor operates (field of view)
        noise           % Measurement noise (range and bearing)
        angular_res     % Angular resolution (separation between readings)
    end
    
    methods
        
        function sen = Sensor                       % Constructor method
            % Default values
            sen.range       =   5.6;
            sen.range_min   =   0.2;
            sen.fov         =   pi;
            sen.noise       =   [0.03; 1*pi/180];   % 3 cm, 1 deg
            sen.angular_res =   1 * pi/180;         % 1 degree
        end
        
        % Find all measurements that are out of the sensor's range and set
        % them to an impossible value so that they can be removed
        function [y, idx] = constrainMeasurement(sen, y)
            off_range = y(1, :) > sen.range | y(1, :) < sen.range_min;
            off_ang = y(2, :) > sen.fov/2 | ...
                y(2, :) < -sen.fov/2;
            y(:, off_range) = inf;
            y(:, off_ang) = inf;
            idx = find(y(1, :) < inf | y(2, :) < inf);
        end
        
        % Generate a scan where the full sweep gives the max range value
        function y = generateEmptyScan(sen)
            readings_a = -sen.fov/2 : sen.angular_res : sen.fov/2;
            numLasers = length(readings_a);
            y = [repmat(sen.range, 1, numLasers); readings_a];
        end

    end
end