% Author: Jai Juneja
% Find the intersection of 2 lines. The intersection point must lie within
% the boundaries specified by the two lines. User may input optional
% threshold.
% Inputs:
%   L1          : Line 1 [x1 x2; y1 y2]
%   L2          : Line 2
%   threshold   : Treshold about area bounded by endpoints about lines 1
%                 and 2 in which point of intersection can lie.
% Outputs:
%   p           : Point of intersection [x; y] (= [0; 0] if not found)
function p = getIntersection(L1, L2, threshold)

    if nargin < 3
        threshold = 0;
    end
    
    % Compute several intermediate quantities
    Dx12 = L1(1,1)-L1(1,2);
    Dx34 = L2(1,1)-L2(1,2);
    Dy12 = L1(2,1)-L1(2,2);
    Dy34 = L2(2,1)-L2(2,2);
    Dx24 = L1(1,2)-L2(1,2);
    Dy24 = L1(2,2)-L2(2,2);

    A = [Dx12 -Dx34; Dy12 -Dy34];
    b = [-Dx24; -Dy24];
    % If lines are not almost parallel (which would give a large condition
    % number)
    if cond(A) < 1/eps
        % Solve for t and s parameters
        ts = A \ b;
        % Take weighted combinations of points on the line
        p = ts(1)*[L1(1,1); L1(2,1)] + (1-ts(1))*[L1(1,2); L1(2,2)];
        
        % If point lies outside of line boundaries, remove it
        if ~(p(1) <= max(L1(1,:)) + threshold && p(1) >= min(L1(1,:)) - threshold && ...
                p(2) <= max(L1(2,:)) + threshold && p(2) >= min(L1(2,:)) - threshold && ...
                p(1) <= max(L2(1,:)) + threshold && p(1) >= min(L2(1,:)) - threshold ...
                && p(2) <= max(L2(2,:)) + threshold && p(2) >= min(L2(2,:)) - threshold)            
            p = [];
        end
    else
        p = [];
    end
end