% Author: Jai Juneja
% Date: 17/03/2013
% For a given set of laser readings, remove multiple readings from a single
% laser ray. All lasers fired at a given angle have the same index,
% provided in the first row of data. For a given index, the laser reading
% that is closest is passed and all others are deleted.
%
% Inputs:
%   data    :   Laser data (in local polar co-ordinates)
%                   [ndx1 ndx2 ndx3 ...
%                    r1   r2   r3   ...
%                    a1   a2   a3]
% Outputs:
%   data    :   Reduced data
function data = removeDuplicateLasers(data)
    % Laser index data is given in first row. Sort by laser index, then by
    % range
    data = sortrows(data', [1 2])';
    
    [~, ndx] = unique(data(1, :), 'first');
    
     data = data(:, ndx);
end