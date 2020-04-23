import gtsam.*;
%% Read data

disp('-- Reading sensor data from file')
%% IMU data

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

% a = struct;
% table_t(1) = a;
% table_t(2) = a;
% table_t(3) = a;

for t_temp = 2:length(t)
    delta_t(t_temp) = (t(t_temp) - t(t_temp-1));
end

for t_temp = 1:length(t)
    table_t(t_temp,:) = struct;
end

for t_temp = 1:length(t)
    table_t(t_temp).Time = t(t_temp);
    table_t(t_temp).dt = delta_t(t_temp);
    table_t(t_temp).accelX = accel_x(t_temp);
    table_t(t_temp).accelY = accel_y(t_temp);
    table_t(t_temp).accelZ = accel_z(t_temp);
    table_t(t_temp).omegaX = rotational_x(t_temp);
    table_t(t_temp).omegaY = rotational_y(t_temp);
    table_t(t_temp).omegaZ = rotational_z(t_temp);
end

% IMU_data = importdata(('KittiEquivBiasedImu.txt'));
% IMU_data = cell2struct(num2cell(IMU_data.data), IMU_data.colheaders, 2);
imum = cellfun(@(x) x', num2cell([ [table_t.accelX]' [table_t.accelY]' [table_t.accelZ]' [table_t.omegaX]' [table_t.omegaY]' [table_t.omegaZ]' ], 2), 'UniformOutput', false);
[table_t.acc_omega] = deal(imum{:});

formated_imu = [table_t.acc_omega]';
% csvwrite('imu_out.csv',filtered_imu)

%% Meta IMU

IMU_metadata = importdata('KittiEquivBiasedImu_metadata.txt');
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);

% IMUinBody_state = [IMU_metadata.BodyPtx, IMU_metadata.BodyPty, IMU_metadata.BodyPrz];
% H = posemat(IMUinBody_state);
% IMUinBody = expm(H);
% IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
%   IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);

% if ~IMUinBody.equals(Pose3, 1e-5)
%   error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
% end

%% GPS data
GPS_data = readtable('gps.csv');
t2 = table2array(GPS_data(:, 1));
t2 = t2/1e6;
lon = table2array(GPS_data(:, 4));
lag = table2array(GPS_data(:, 5));
alti = table2array(GPS_data(:, 6));

for t_temp = 1:length(t2)
    gps_t(t_temp,:) = struct;
end

for t_temp = 1:length(t2)
    gps_t(t_temp).Time = t2(t_temp);
    gps_t(t_temp).lon = lon(t_temp);
    gps_t(t_temp).lag = lag(t_temp);
    gps_t(t_temp).alti = alti(t_temp);
end
gps_t




%% Function
function H = posemat(state)
    x = state(1);
    y = state(2);
    h = state(3);
    % construct a SE(2) matrix element
    H = [...
        cos(h) -sin(h) x;
        sin(h)  cos(h) y;
             0       0 1];
end