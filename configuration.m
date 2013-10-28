function [World, rob] = configuration(rob, sen_rf, Lmks, Wpts, AxisDim)
global RunTime;

    % If you want to make any changes to the default robot/sensor settings
    % then do that here - the changes will be passed back into the main
    % SLAM function.
    
    % Set robot at first waypoint
    rob.R(1:2) = Wpts(:,1);
    % Orient robot towards second waypoint
    rob.R(3) = bearingToPoint(rob.R, Wpts(:, 2)) + rob.R(3);
    % Initial pose estimate is perfect
    rob.r = rob.R;
    
    % These values are set by an engineer to match the system:
    World.Q = diag(rob.q .^ 2);     % System uncertainty
    World.M = diag(sen_rf.noise .^ 2); % Measurement uncertainty
    
    World.tend = RunTime;

    % Initialise landmarks and waypoints
    World.W = Lmks;
    World.Wpts = Wpts;
    
    % Initialise measurement vector
    World.y = zeros(2,size(World.W,2));

    % Initialise state and covariance vectors
    World.x = zeros(numel(rob.r)+numel(World.W), 1);    % State estimate
    World.P = zeros(numel(World.x),numel(World.x));     % Covariance matrix
    World.mapspace = 1:numel(World.x);                  % State map availability
    World.l = zeros(2, size(World.W,2));                % Landmark positions

    % Find anad reserve mapspace for robot pose
    World.r = find(World.mapspace,numel(rob.r));
    World.mapspace(World.r) = 0;
    World.x(World.r) = rob.R;     % Set initial pose estimate equal to truth
    World.P(World.r, World.r) = 0;  % Set initial pose covariance equal to 0
    
    % Initialise historical vectors
    World.R_hist = zeros(2, World.tend);
    World.r_hist = zeros(2, World.tend);
    World.Pr_hist = zeros(2, World.tend);
    World.error_hist = zeros(1, World.tend);
    World.scan_error_hist = zeros(2, World.tend);
    World.odo_error_hist = zeros(2, World.tend);
    World.turning_hist = zeros(1, World.tend);
    World.weight_scan_hist = zeros(1, World.tend);
    World.weight_odo_hist = zeros(1, World.tend);

    %%%%%%%%%%%%%%%%%%%%%%%%% MAPPING SETTINGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Map resolution (grid size)
    World.map_res = 0.5;
    % Scan correlation tolerance. This defines the minimum number of
    % correlations required for a scan match to be accepted
    World.scan_corr_tolerance = 20;
    % Descretised grid map values
    World.map_vals = -AxisDim+World.map_res:World.map_res:AxisDim-World.map_res;
    World.gridmap = zeros(AxisDim * 2 / World.map_res - 1);
    World.gridmap_counter = ones(size(World.gridmap)) * round(255/2);
    World.scan_data = [];
    World.scan_global = [];
    World.scan_true = [];
end