% Test ICP scan matching algorithm
numLoops = 100;
load corridor.mat;
rob = Robot;
rob.u = [2; 0];
sen = Sensor;
sen.range = 5;
S = zeros(1, numLoops);
Err = zeros(2, numLoops);

for k = 1:numLoops
    rob.R = [-2; -1; 0];
    rob.r = rob.R;

    % obs is an array of objects of type obstacle
    scan_data_1 = [];
    for i = 1:length(obs)
        scan_data_1_tmp = obs(i).getMeasured(sen, rob);
        scan_data_1 = [scan_data_1 scan_data_1_tmp];
    end
    scan_data_1 = removeDuplicateLasers(scan_data_1);
    scan_data_1 = scan_data_1(2:3, :);
    scan_data_1 = getInvMeasurement(scan_data_1);

    rob.R = rob.move(rob.q .* randn(2,1), 'true');
    
    scan_data_2 = [];
    for i = 1:length(obs)
        scan_data_2_tmp = obs(i).getMeasured(sen, rob);
        scan_data_2 = [scan_data_2 scan_data_2_tmp];
    end
    scan_data_2 = removeDuplicateLasers(scan_data_2);
    scan_data_2 = scan_data_2(2:3, :);
    scan_data_2 = getInvMeasurement(scan_data_2);

    [R, T, corr, icp_var] = doICP(scan_data_1, scan_data_2, 1, rob.u);

    da = getPiToPi(asin(R(2,1)));
    rob.r(1:2) = transToGlobal(rob.r, T);
    rob.r(3) = getPiToPi(rob.r(3) + da);

    scan_data_2 = transToGlobal(rob.r, scan_data_2);
    C = getICPCovariance(icp_var, scan_data_2(:, corr(:, 2)'));
    S(k) = trace(C);
    Err(:, k) = [pdist([rob.r(1:2)'; rob.R(1:2)']); abs(getPiToPi(rob.r(3) - rob.R(3)))];
end

S_avg = sum(S)/numLoops
S_stdev = std(S)
Err_avg = [sum(Err(1,:))/numLoops; (sum(Err(2,:))/numLoops)*180/pi]
Err_stdev = [std(Err(1,:)); std(Err(2,:))*180/pi]