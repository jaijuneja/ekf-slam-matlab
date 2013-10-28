% Test ICP scan matching algorithm

load medFilterScan.mat;
rob = Robot;
rob.R = [0; 0; 0];
rob.r = rob.R;
rob.u = [2; pi/1.5]; % Set robot control to move in straight line
sen = Sensor;
sen.range = 30;
sen.fov = 270*pi/180;
axis_size = 7;
sen.noise = [0.3; 3*pi/180];

% obs is an array of objects of type obstacle
scan_data_1 = [];
for i = 1:length(obs)
    scan_data_1_tmp = obs(i).getMeasured(sen, rob);
    scan_data_1 = [scan_data_1 scan_data_1_tmp];
end
scan_data_1 = removeDuplicateLasers(scan_data_1);
scan_data_1 = scan_data_1(2:3, :);
% First add Gaussian white noise to scan
v = repmat(sen.noise, 1, size(scan_data_1, 2)) .* randn(2,size(scan_data_1, 2));
scan_data_1 = scan_data_1 + v;
scan_data_filt = scan_data_1;
scan_data_1 = getInvMeasurement(scan_data_1);

scan_data_filt(1,:) = medfilt1(scan_data_filt(1,:), 5);
scan_data_filt(2,:) = medfilt1(scan_data_filt(2,:), 5);
scan_data_filt = getInvMeasurement(scan_data_filt);

p1 = rob.computeTriangle('true');

% Plot results
figure('color', 'white')
s1 = subplot(1,2,1);
h1 = plot(p1(1,:), p1(2,:), 'b-', scan_data_1(1,:), scan_data_1(2,:), 'r+');
hold on
for i = 1:length(obs)
    obs(i).plot(s1);
end
axis square
axis([-axis_size axis_size -axis_size axis_size])
title('Raw noisy scan data')
legend('wheelchair pose', 'scan points', 'obstacles')

s2 = subplot(1,2,2);
h2 = plot(p1(1,:), p1(2,:), 'b-', scan_data_filt(1,:), scan_data_filt(2,:), 'r+');
obs2 = obs;
hold on
for i = 1:length(obs)
    ob2graphics = copyobj(obs(i).graphics,s2);
end
axis square
axis([-axis_size axis_size -axis_size axis_size])
title(['Scan data after' char(10) '1D median filtering'])
%legend('scan before', 'scan after')