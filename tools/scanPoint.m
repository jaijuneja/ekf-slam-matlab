% Author: Jai Juneja (adapted from SLAM course by Joan Sola)
% Date: 12/02/2013
%
% For a point p_g in the global frame, determine the range-bearing
% measurement y in the robot's local frame. This first requires a
% transformation of the point p_g to the robot's local frame to give p_r.
% This is then converted to a range and bearing vector with getMeasurement.
%
% Inputs:
%   r = [x y alpha]'    :   Robot frame
%   p_g = [pg_x pg_y]'  :   Point in global frame
%
% Outputs:
%   y = [d a]           :   Range-bearing sensor measurement
%   Optional:
%   Y_r                 :   Jacobian of y wrt. r
%   Y_pg                :   Jacobian of y wrt. p_g

function [y, Y_r, Y_pg] = scanPoint(r, p_g)
    
    if nargout == 1
        p_r = transToLocal(r, p_g);
        y = getMeasurement(p_r);    % Obtain range-bearing measurement y of p_r
    else
        % Compute Jacobians:
        [p_r, PR_r, PR_pg] = transToLocal(r, p_g);
        [y, Y_pr] = getMeasurement(p_r);
        
        % From the chain rule, we deduce:
        Y_r = Y_pr * PR_r;
        Y_pg = Y_pr * PR_pg;
    end
end