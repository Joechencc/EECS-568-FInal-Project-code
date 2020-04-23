clear all;
import gtsam.*;
disp('Example of application of ISAM2 for GPS-aided navigation on the KITTI VISION BENCHMARK SUITE (http://www.computervisiononline.com/dataset/kitti-vision-benchmark-suite)')

%% Read metadata and compute relative sensor pose transforms
% IMU metadata
disp('-- Reading sensor metadata')
IMU_metadata = importdata(findExampleDataFile('KittiEquivBiasedImu_metadata.txt'));
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
  IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);
if ~IMUinBody.equals(Pose3, 1e-5)
  error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
end

%% Read data
disp('-- Reading sensor data from file')
% % % IMU data
% IMU_data = importdata(findExampleDataFile('KittiEquivBiasedImu.txt'));
% IMU_data = cell2struct(num2cell(IMU_data.data), IMU_data.colheaders, 2);
% imum = cellfun(@(x) x', num2cell([ [IMU_data.accelX]' [IMU_data.accelY]' [IMU_data.accelZ]' [IMU_data.omegaX]' [IMU_data.omegaY]' [IMU_data.omegaZ]' ], 2), 'UniformOutput', false);
% [IMU_data.acc_omega] = deal(imum{:});

ms25 = readtable('ms25.csv');
t = table2array(ms25(:, 1));
t = t/1e6;
mag_x = table2array(ms25(:, 2));
mag_y = table2array(ms25(:, 3));
mag_z = table2array(ms25(:, 4));

accel_x = table2array(ms25(:, 5));
accel_y = table2array(ms25(:, 6));
accel_z = table2array(ms25(:, 7));

rotational_x = table2array(ms25(:, 8));
rotational_y = table2array(ms25(:, 9));
rotational_z = table2array(ms25(:, 10));

delta_t = zeros(size(t));
% table_t = zeros(size(t));

delta_t(1) = t(1);

table_t = struct([]);

% a = struct; table_t(1) = a; table_t(2) = a; table_t(3) = a;

for t_temp = 2:length(t)
    delta_t(t_temp) = (t(t_temp) - t(t_temp-1));
end

for t_temp = 1:length(t)
    table_t(t_temp,:) = struct;
end

for t_temp = 1:length(t)
    table_t(t_temp).Time = t(t_temp);
    table_t(t_temp).dt = delta_t(t_temp);
    table_t(t_temp).omegaX = -rotational_x(t_temp);
    table_t(t_temp).omegaY = rotational_y(t_temp);
    table_t(t_temp).omegaZ = -rotational_z(t_temp);
    table_t(t_temp).accelX = -accel_x(t_temp);
    table_t(t_temp).accelY = accel_y(t_temp);
    table_t(t_temp).accelZ = -accel_z(t_temp);
    
end

imum = cellfun(@(x) x', num2cell([ [table_t.accelX]' [table_t.accelY]' [table_t.accelZ]' [table_t.omegaX]' [table_t.omegaY]' [table_t.omegaZ]' ], 2), 'UniformOutput', false);
[table_t.acc_omega] = deal(imum{:});
IMU_data = table_t;

% for t_temp = 1:length(t)
%     for j = 1:3
%         if IMU_data(min(t_temp,length(IMU_data))).acc_omega == IMU_data(max(1,min(t_temp,length(IMU_data))-j)).acc_omega
%             IMU_data(t_temp) = []
%         else
%         end 
%     end
% end

clear imum

% GPS data
% GPS_data = importdata(findExampleDataFile('KittiGps_converted.txt'));
% GPS_data = cell2struct(num2cell(GPS_data.data), GPS_data.colheaders, 2);
% for i = 1:numel(GPS_data)
%     GPS_data(i).Position = gtsam.Point3(GPS_data(i).X, GPS_data(i).Y, GPS_data(i).Z);
% end

GPS = readtable('gps.csv');
t2 = table2array(GPS(:, 1));
t2 = t2/1e6;
latitude = table2array(GPS(:, 4));
longitude = table2array(GPS(:, 5));
altitude = table2array(GPS(:, 6));
GPS_data = struct([]);
for t_temp = 1:length(t2)
    GPS_data(t_temp,:) = struct;
end

for t_temp = 1:length(t2)
    GPS_data(t_temp).Time = t2(t_temp);
    GPS_data(t_temp).X = (latitude(t_temp) - latitude(1)) * 180 / pi * 111139 ;
    GPS_data(t_temp).Y = (longitude(t_temp) - longitude(1)) * 180 /pi * 111139;
    GPS_data(t_temp).Z = altitude(t_temp) - altitude(1);
end

% load('original_GPS.mat')
% GPS_data = GPS_data';

for i = 1:numel(GPS_data)
    GPS_data(i).Position = gtsam.Point3(GPS_data(i).X, GPS_data(i).Y, GPS_data(i).Z);
end

noiseModelGPS = noiseModel.Diagonal.Precisions([ [0;0;0]; 1.0/0.07 * [1;1;1] ]);
firstGPSPose = 2;
GPSskip = 10; % Skip this many GPS measurements each time

%% Process IMU data
new_IMU_data = struct([]);

count = 1;
for i = 1:length(GPS_data)
    while IMU_data(count).Time < GPS_data(i).Time
        count = count + 1;
    end
    new_IMU_data = [new_IMU_data, IMU_data(count)];
end

IMU_data = IMU_data';
%% Get initial conditions for the estimated trajectory
currentPoseGlobal = Pose3(Rot3, GPS_data(firstGPSPose).Position); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
g = [0;0;-9.8];
w_coriolis = [0;0;0];

%% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

%% Main loop:
% (1) we read the measurements
% (2) we create the corresponding factors in the graph
% (3) we solve the graph to obtain and optimal estimate of robot trajectory
IMUtimes = [IMU_data.Time];

disp('-- Starting main loop: inference is performed at each time step, but we plot trajectory every 10 steps')

for measurementIndex = firstGPSPose:length(GPS_data)
  
  % At each non=IMU measurement we initialize a new node in the graph
  currentPoseKey = symbol('x',measurementIndex);
  currentVelKey =  symbol('v',measurementIndex);
  currentBiasKey = symbol('b',measurementIndex);
  t = GPS_data(measurementIndex, 1).Time;
  
  if measurementIndex == firstGPSPose
    %% Create initial estimate and prior on initial pose, velocity, and biases
    newValues.insert(currentPoseKey, currentPoseGlobal);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, sigma_init_x));
    newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
    newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
  else
    t_previous = GPS_data(measurementIndex-1, 1).Time;
    %% Summarize IMU data between the previous GPS measurement and now
    IMUindices = find(IMUtimes >= t_previous & IMUtimes <= t);
    
    currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
      currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
      IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
    
    for imuIndex = IMUindices
      accMeas = [ IMU_data(imuIndex).accelX; IMU_data(imuIndex).accelY; IMU_data(imuIndex).accelZ ];
      omegaMeas = [ IMU_data(imuIndex).omegaX; IMU_data(imuIndex).omegaY; IMU_data(imuIndex).omegaZ ];
      deltaT = IMU_data(imuIndex).dt;
      currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
    end
    
    % Create IMU factor
    newFactors.add(ImuFactor( ...
      currentPoseKey-1, currentVelKey-1, ...
      currentPoseKey, currentVelKey, ...
      currentBiasKey, currentSummarizedMeasurement, g, w_coriolis));
    
    % Bias evolution as given in the IMU metadata
    newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
      noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * sigma_between_b)));

    % Create GPS factor
    GPSPose = Pose3(currentPoseGlobal.rotation, GPS_data(measurementIndex).Position);
    if mod(measurementIndex, GPSskip) == 0
      newFactors.add(PriorFactorPose3(currentPoseKey, GPSPose, noiseModelGPS));
    end

    % Add initial value
    newValues.insert(currentPoseKey, GPSPose);
    newValues.insert(currentVelKey, currentVelocityGlobal);
    newValues.insert(currentBiasKey, currentBias);
    
    % Update solver
    % =======================================================================
    % We accumulate 2*GPSskip GPS measurements before updating the solver at
    % first so that the heading becomes observable.
    if measurementIndex > firstGPSPose + 2*GPSskip
      isam.update(newFactors, newValues);
      newFactors = NonlinearFactorGraph;
      newValues = Values;
      
      if rem(measurementIndex,10)==0 % plot every 10 time steps
        cla;
        plot3DTrajectory(isam.calculateEstimate, 'g-');
        title('Estimated trajectory using ISAM2 (IMU+GPS)')
        xlabel('[m]')
        ylabel('[m]')
        zlabel('[m]')
        axis equal
        drawnow;
      end
      % =======================================================================
      currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
      currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
      currentBias = isam.calculateEstimate(currentBiasKey);
    end
  end
   
end % end main loop

disp('-- Reached end of sensor data')
