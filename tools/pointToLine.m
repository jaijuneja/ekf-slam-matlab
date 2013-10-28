% Author: Jai Juneja
% Date: 22/03/2013
%
% Find the shortest Euclidean distance from a point to a line.
% Inputs:
%   pt  :   point in space
%   v1  :   first point along line
%   v2  :   second point along line
%           pt, v1 and v2 must all be at least 3x1 vectors
% Outputs:
%   d   :   shortest distance from point pt to line
function d = pointToLine(pt, v1, v2)
    a = v1 - v2;
    b = pt - v2;
    d = norm(cross(a,b)) / norm(a);
end