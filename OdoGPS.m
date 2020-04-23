%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief Example of a simple 2D localization example
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

%% Assumptions
%  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
%  - The robot moves 2 meters each step
%  - The robot is on a grid, moving 2 meters each step

%% load Odometry
load('x4.mat')
load('y4.mat')
load('angle.mat')
load('time.mat')

%% load GPS
load('filtered_data.mat')

%% Preprocess data
count = 1;
new_x4 = zeros(size(GPS_data))';
new_y4 = zeros(size(GPS_data))';
new_time = zeros(size(GPS_data))';
new_angle = zeros(size(GPS_data))';

for t_temp = 1:length(GPS_data)
    while t(count)/1e6 < GPS_data(t_temp).Time
        count = count+1;
    end
    new_time(t_temp) = t(count)/1e6;
    new_x4(t_temp) = x4(count);
    new_y4(t_temp) = y4(count);
    new_angle(t_temp) = angle(count);
end


%% Create the graph (defined in pose2SLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;

%% Add a Gaussian prior on pose x_1
priorMean = Pose2(new_x4(1), new_y4(1), new_angle(1)); % prior mean is at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]); % 30cm std on x,y, 0.1 rad on theta
graph.add(PriorFactorPose2(1, priorMean, priorNoise)); % add directly to graph

%% Add two odometry factors
for i = 2:length(new_x4)
    odometry = Pose2(new_x4(i)-new_x4(i-1), new_y4(i)-new_y4(i-1), new_angle(i)-new_angle(i-1));
%     odometry = Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
    odometryNoise = noiseModel.Diagonal.Sigmas([100; 100; 0.3]); 
%     odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]); % 20cm std on x,y, 0.1 rad on theta
    graph.add(BetweenFactorPose2(i-1, i, odometry, odometryNoise));
%     graph.add(BetweenFactorPose2(2, 3, odometry, odometryNoise));
end

%% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
% initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
% initialEstimate.insert(2, Pose2(2.3, 0.1,-0.2));
% initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
angle2 = zeros(size(GPS_data));
angle2(1)=0;
for i = 2:length(GPS_data)
    diffy = GPS_data(i).Y - GPS_data(i-1).Y;
    diffx = GPS_data(i).X - GPS_data(i-1).X;
    angle2(i) = atan2(diffy, diffx);
end

for i = 1:length(GPS_data)
    initialEstimate.insert(i,Pose2(GPS_data(i).X, GPS_data(i).Y, angle2(i)));
end

initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n  '));

%% Plot trajectory and covariance ellipses
cla;


load("x_ground.mat")
load("y_ground.mat")
gtsam.plot2DTrajectory(result,'g');
hold on;
plot(x2,y2,'r');
xlabel("x[m]")
ylabel("y[m]")
title("Fuse Odometry with GPS vs Ground truth")
legend("GPS-Odometry","","ground-truth")
axis equal

% plot2DTrajectory(initialEstimate,'k*-');

axis equal
view(2)
