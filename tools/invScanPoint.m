% Author: Jai Juneja (adapted from SLAM course by Joan Sola)
% Date: 12/02/2013
%
% For a range-bearing measurement y in the robot's local frame, determine
% the corresponding point measured in the global frame. This first requires
% a conversion of the range-bearing measurement into a cartesian point p_r
% in the robot's frame. transToGlobal then transforms p_r to a point p_g in
% the global frame.
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

function [p_g, PG_r, PG_y] = invScanPoint(r, y)
    
    if nargout == 1
        p_r = getInvMeasurement(y);     % Convert range-bearing measurement to local point
        p_g = transToGlobal(r, p_r);
    else
        % Compute Jacobians:
        [p_r, PR_y] = getInvMeasurement(y);
        [p_g, PG_r, PG_pr] = transToGlobal(r, p_r);
        
        % From the chain rule, we deduce:
        PG_y = PG_pr * PR_y;
    end
end