% Author: Jai Juneja (adapted from video lecture series by Joan Sola)
% Date: 12/02/2013
%
% Given a range-bearing measurement y, obtain position p of observed
% landmark in the robot's frame.
%
% Inputs:
%   y = [d a]       :   Laser range and bearing measurement
%
% Outputs:
%   p_r = [p_x p_y]':   Cartesian position of observation in robot's frame
%   Optional:
%   P_y             :   Jacobian of p wrt. y

function [p_r, P_y] = getInvMeasurement(y)
    d = y(1, :);           % Range measurement
    a = y(2, :);           % Bearing measurement

    p_x = d .* cos(a);   % x-position of observation in robot's frame
    p_y = d .* sin(a);   % y-position of observation in robot's frame
    
    p_r = [p_x; p_y];
    
    if nargout > 1      % Compute Jacobian (only works for single measurement)
        P_y = [...
            cos(a)  -d*sin(a)
            sin(a)  d*cos(a)];
    end
end