% Author:   Jai Juneja (adapted from SLAM course by Joan Sola:
%           http://www.joansola.eu/JoanSola/eng/course.html)
% Date:     12/02/2013
%
% Tranform a point p_g from global co-ordinate frame to robot r's local
% frame.
%
% Inputs: 
%   r = [x y alpha]     :   Robot frame
%   p_g = [pg_x pg_y]   :   Point in global frame
%
% Outputs:
%   p_r = [pr_x pr_y]   :	Point in robot's frame
%   Optional:
%   PR_r                :   Jacobian of p_r wrt. robot frame
%   PR_pr               :   Jacobian of p_r wrt. p_g

function [p_r, PR_r, PR_pg] = transToLocal(r, p_g)

    r_pos = r(1:2);     % Robot's position
    r_ang = r(3);       % Robot's orientation
    
    % Transformation matrix:
    R = [cos(r_ang), sin(r_ang) ; ...
        -sin(r_ang), cos(r_ang)];
    
    % Hence point p_g tranformed to local co-ordinate frame is:
    p_r = [p_g(1, :) - r_pos(1); p_g(2, :) - r_pos(2)];
    p_r = R * p_r;

    if nargout > 1      % Compute Jacobians (only works for single point)
        pg_x = p_g(1); pg_y = p_g(2);
        r_x = r_pos(1); r_y = r_pos(2);

        PR_r = [ ...
            -cos(r_ang)	-sin(r_ang)	cos(r_ang)*(pg_y - r_y)-sin(r_ang)*(pg_x - r_x); ...
            sin(r_ang)	-cos(r_ang)	-cos(r_ang)*(pg_x - r_x)-sin(r_ang)*(pg_y - r_y)];
        
        PR_pg = R;
    end
end