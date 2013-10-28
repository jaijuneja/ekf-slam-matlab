% Author: Jai Juneja
% Date: 18/03/2013
%
% Basic mapping function:
% 1. Correct scan points for the update step
% 2. Create a grid map of size defined in configuration
% 3. Count the number of sensor readings in each grid cell
% 4. Add these counts to the historical grid (containing sum of all past
%    counts)
% 5. Create grid map, where 'occupied' cells are those whose count is
%    in the highest x percentile, where x is defined in configuration
function World = doMapping_old(World, r_new, r_old, scan_data_corr, correl)
    % Determine pose correction during update step
    update = r_new - r_old;

    if ~(r_new == r_old)
        % Adjust latest scan data to account for pose correction
        World.scan_data = transformPoints(World.scan_data, update);

        scan_data_corr = World.scan_data;
        for i = 1:size(scan_data_corr, 2)
            scan_data_corr(:, i) = transToGlobal(r_new, scan_data_corr(:, i));
        end
        % Add corellated scan points to map
        scan_data_corr = scan_data_corr(:, correl(:, 2)'); % Don't need to add already associated points
    end

    % Map the corrected data
    World.scan_corr = scan_data_corr;
    gridmap_tmp = zeros(size(World.gridmap));
    scan_data_corr = interp1(World.map_vals, World.map_vals, scan_data_corr, 'nearest');

    if ~isempty(scan_data_corr)
        for i = 1:length(World.map_vals)
            for j = 1:length(World.map_vals)
                gridmap_tmp(i, j) = sum(scan_data_corr(1,:) == World.map_vals(j) & ...
                    scan_data_corr(2,:) == World.map_vals(i));
            end
        end
    end
    World.gridmap_counter = World.gridmap_counter + gridmap_tmp;
    World.gridmap = World.gridmap_counter > prctile(prctile ...
        (World.gridmap_counter,World.map_tolerance,1),World.map_tolerance,2);
    World.map = map2Plot(World.gridmap, World.map_vals);
    % plotGridMap(World.gridmap);
    figure(1);
    imagesc(flipud(~World.gridmap));
    axis equal
end