% Test ICP scan matching algorithm

load corridor.mat;
rob = Robot;
rob.R = [-2; -1; 0];
rob.r = rob.R;
rob.u = [2; pi/1.5]; % Set robot control to move in straight line
sen = Sensor;
sen.range = 30;
sen.fov = 3*pi/2;
axis_size = 5.5;

% obs is an array of objects of type obstacle
scan_data_1 = [];
for i = 1:length(obs)
    scan_data_1_tmp = obs(i).getMeasured(sen, rob);
    scan_data_1 = [scan_data_1 scan_data_1_tmp];
end
scan_data_1 = removeDuplicateLasers(scan_data_1);
scan_data_1 = scan_data_1(2:3, :);
v = repmat(sen.noise, 1, size(scan_data_1, 2)) .* randn(2,size(scan_data_1, 2));
scan_data_1 = scan_data_1 + v;
scan_data_1 = getInvMeasurement(scan_data_1);
        
p1 = rob.computeTriangle('true');
p1_guess = rob.computeTriangle;


rob.R = rob.move(rob.q .* randn(2,1), 'true');
p2 = rob.computeTriangle('true');

scan_data_2 = [];
for i = 1:length(obs)
    scan_data_2_tmp = obs(i).getMeasured(sen, rob);
    scan_data_2 = [scan_data_2 scan_data_2_tmp];
end
scan_data_2 = removeDuplicateLasers(scan_data_2);
scan_data_2 = scan_data_2(2:3, :);
v = repmat(sen.noise, 1, size(scan_data_2, 2)) .* randn(2,size(scan_data_2, 2));
scan_data_2 = scan_data_2 + v;
scan_data_2 = getInvMeasurement(scan_data_2);

[R, T, corr, icp_var] = doICP(scan_data_1, scan_data_2, 1, rob.u);

da = getPiToPi(asin(R(2,1)));
rob.r(1:2) = transToGlobal(rob.r, T);
rob.r(3) = rob.r(3) + da;
p2_guess = rob.computeTriangle;

% Plot results
figure('color', 'white')
s1 = subplot(1,2,1);
h1 = plot(p1(1,:), p1(2,:), 'r-', p2(1,:), p2(2,:), 'b-', ...
    p2_guess(1,:), p2_guess(2,:), 'm--');
hold on
for i = 1:length(obs)
    obs(i).plot(s1);
end
axis square
axis([-axis_size axis_size -axis_size axis_size])
title(['Before and after wheelchair poses' char(10) 'relative to obstacles'])
legend('pose before', 'pose after', 'estimated pose after', 'obstacles')

% Rot = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)];
Rot = 1;
scan_data_2_rot = Rot*scan_data_2;
s2 = subplot(1,2,2);
h2 = plot(scan_data_1(1,:), scan_data_1(2,:), 'r+', ...
    scan_data_2_rot(1,:), scan_data_2_rot(2,:), 'b+');
axis square
axis([-axis_size axis_size -axis_size axis_size])
title(['Before and after scans' char(10) 'in local (Cartesian) frame'])
legend('scan before', 'scan after')

scan_data_2 = transToGlobal(rob.r, scan_data_2);
C = getICPCovariance(icp_var, scan_data_2(:, corr(:, 2)'));
S = 1/trace(C);
disp(['Saliency score: ' num2str(S)])