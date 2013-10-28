% Author: Jai Juneja
% Date: 12/02/2013
% 
% Function transforms a point p_r from robot's local frame to the global
% frame.
%
% Inputs: 
%   r = [x y alpha]'	:   Robot frame
%   p_r                 :	Point in robot's frame (2*n matrix for n points)
%
% Outputs:
%   p_g                 :   Point in global frame (2*n matrix)
%   Optional:
%   PG_r                :   Jacobian of p_g wrt. robot frame
%   PG_pr               :   Jacobian of p_g wrt. p_r
function [p_g, PG_r, PG_pr] = transToGlobal(r, p_r)

    r_pos = r(1:2);     % Robot's position
    r_ang = r(3);       % Robot's orientation
        
    % Transformation matrix:
    R = [cos(r_ang), -sin(r_ang) ; ...
        sin(r_ang), cos(r_ang)];
    
    % Hence point p_r tranformed to global co-ordinate frame is:
    p_g = R * p_r;
    p_g = [p_g(1, :) + r_pos(1); p_g(2, :) + r_pos(2)];
    
    if nargout > 1      % Compute Jacobians
        pr_x = p_r(1); pr_y = p_r(2);
    
        PG_r = [...
            1,  0,  - pr_y*cos(r_ang) - pr_x*sin(r_ang); ...
            0,  1,  pr_x*cos(r_ang) - pr_y*sin(r_ang)];

        PG_pr = R;
    end
end