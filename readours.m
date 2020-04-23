clear all;
%% IMU
load("GPS_data.mat")
GPS_data = new_GPS_data;
dt_temp = GPS_data(2).Time - GPS_data(1).Time;
init_vx = (GPS_data(2).X - GPS_data(1).X)/dt_temp;
init_vy = (GPS_data(2).Y - GPS_data(1).Y)/dt_temp;

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
    table_t(t_temp).accelX = accel_x(t_temp);
    table_t(t_temp).accelY = accel_y(t_temp);
    table_t(t_temp).accelZ = -accel_z(t_temp);
    table_t(t_temp).omegaX = rotational_x(t_temp);
    table_t(t_temp).omegaY = rotational_y(t_temp);
    table_t(t_temp).omegaZ = -rotational_z(t_temp);
end

imum = cellfun(@(x) x', num2cell([ [table_t.accelX]' [table_t.accelY]' [table_t.accelZ]' [table_t.omegaX]' [table_t.omegaY]' [table_t.omegaZ]' ], 2), 'UniformOutput', false);
[table_t.acc_omega] = deal(imum{:});
IMU_data = table_t;
t = [IMU_data.Time];
delta_t(1) = IMU_data(1).Time;


% for t_temp = 1:length(t)
%     for j = 1:3
%         if IMU_data(min(t_temp,length(IMU_data))).acc_omega == IMU_data(max(1,min(t_temp,length(IMU_data))-j)).acc_omega
%             IMU_data(t_temp) = []
%         else
%         end 
%     end
% end
endR_array(:,:,1) = eye(2);
R_array(:,:,1) = posemat(0.5);
V_array(:,1) = [0;0];
P_array(:,1) = [GPS_data(1).X, GPS_data(1).Y];
g_vector = [0;0];

for i = 2:length(IMU_data)
    dt = delta_t(i);
    R_array(:,:,i) = R_array(:,:,i-1) * posemat((IMU_data(i).omegaZ)*dt);
    x_component = IMU_data(i-1).accelX - 9.8* sin(IMU_data(i).omegaX)*cos(IMU_data(i).omegaZ);
    y_component = IMU_data(i-1).accelY - 9.8* sin(IMU_data(i).omegaX)*sin(IMU_data(i).omegaZ);
    
    ak = [x_component; y_component];
    V_array(:,i) = V_array(:,i-1) + (R_array(:,:,i) * ak + g_vector).* dt*0.1;
    P_array(:,i) = P_array(:,i-1) + V_array(:,i) * dt + 0.5*(R_array(:,:,i)*ak+g_vector)*dt^2;
end

plot(P_array(1,:), P_array(2,:),'b')
% 
% 
% 
% 
% %% GPS
GPS = readtable('gps.csv');
t2 = table2array(GPS(:, 1));
t2 = t2/1e6;
latitude = table2array(GPS(:, 4));
longitude = table2array(GPS(:, 5));
altitude = table2array(GPS(:, 6));
GPS_data = struct([]);

GPS_x_limit = [-900,100];
GPS_y_limit = [-2500,100];

for t_temp = 1:length(t2)
    GPS_data(t_temp,:) = struct;
end

for t_temp = 1:length(t2)
    GPS_data(t_temp).Time = t2(t_temp);
    GPS_data(t_temp).X = (latitude(t_temp) - latitude(1)) * 180 / pi * 111139 ;
    GPS_data(t_temp).Y = (longitude(t_temp) - longitude(1)) * 180 /pi * 111139;
    GPS_data(t_temp).Z = altitude(t_temp) - altitude(1);
end

% new_GPS_data = struct([]);
% for t_temp = 1:length(t2)
%     index = min(t_temp,length(GPS_data));
%     if ~(GPS_data(index).X > GPS_x_limit(2)) & ~(GPS_data(index).X < GPS_x_limit(1)) & ...
%         ~(GPS_data(index).Y > GPS_y_limit(2)) & ~(GPS_data(index).Y < GPS_y_limit(1))
%         new_GPS_data = [new_GPS_data,GPS_data(index)];
%     end
% end
% 
% 
% load("new_GPS_data.mat")
% for i = 1:numel(new_GPS_data)
%     new_GPS_data(i).Position = gtsam.Point3(new_GPS_data(i).X, new_GPS_data(i).Y, new_GPS_data(i).Z);
% end
% 
% 

%% filter GPS
% GPS_data = struct([]);
% for t_temp = 2:length(new_GPS_data)
%     loc1 = [new_GPS_data(t_temp).X, new_GPS_data(t_temp).Y, new_GPS_data(t_temp).Z];
%     loc2 = [new_GPS_data(t_temp-1).X, new_GPS_data(t_temp-1).Y, new_GPS_data(t_temp-1).Z];
%     if isequal(loc1,loc2)
%     else
%        GPS_data = [GPS_data, new_GPS_data(t_temp)];
%     end
% end


hold on;
plot([GPS_data.X],[GPS_data.Y],'m')
xlabel("x[m]")
ylabel("y[m]")
title("compare IMU data with GPS")

%% Function

function H = posemat(state)
    h = state/pi *180;
    % construct a SE(2) matrix element
    H = [...
        cos(h) -sin(h) 
        sin(h)  cos(h) ];
end