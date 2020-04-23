clear all;
IMU_data = importdata(('KittiEquivBiasedImu.txt'));
IMU_data = cell2struct(num2cell(IMU_data.data), IMU_data.colheaders, 2);
imum = cellfun(@(x) x', num2cell([ [IMU_data.accelX]' [IMU_data.accelY]' [IMU_data.accelZ]' [IMU_data.omegaX]' [IMU_data.omegaY]' [IMU_data.omegaZ]' ], 2), 'UniformOutput', false);
[IMU_data.acc_omega] = deal(imum{:});

GPS_data = importdata(('KittiGps_converted.txt'));
GPS_data = cell2struct(num2cell(GPS_data.data), GPS_data.colheaders, 2);

dt_temp = GPS_data(2).Time - GPS_data(1).Time;
init_vx = (GPS_data(2).X - GPS_data(1).X)/dt_temp;
init_vy = (GPS_data(2).Y - GPS_data(1).Y)/dt_temp;

t = [IMU_data.Time];
delta_t(1) = IMU_data(1).Time;

% a = struct; table_t(1) = a; table_t(2) = a; table_t(3) = a;

for t_temp = 2:length(IMU_data)
    delta_t(t_temp) = (t(t_temp) - t(t_temp-1));
    if t_temp ==2
        delta_t(t_temp) = delta_t(t_temp);
    end
end

% 
% for i = 1:numel(GPS_data)
%     GPS_data(i).Position = gtsam.Point3(GPS_data(i).X, GPS_data(i).Y, GPS_data(i).Z);
% end

R_array(:,:,1) = eye(2);
V_array(:,1) = [0;0];
P_array(:,1) = [GPS_data(1).X, GPS_data(1).Y];
g_vector = [0;0];

for i = 2:length(IMU_data)
    posemat(IMU_data(i).omegaZ);
    dt = delta_t(i);
    R_array(:,:,i) = R_array(:,:,i-1) * posemat((IMU_data(i).omegaZ));
%     R_array(:,:,i)
    ak = [IMU_data(i-1).accelX; IMU_data(i-1).accelY];
%     (R_array(:,:,i) * ak + g_vector).* dt^2
    V_array(:,i) = V_array(:,i-1) + (R_array(:,:,i) * ak + g_vector).* dt;
    P_array(:,i) = P_array(:,i-1) + V_array(:,i) * dt + 0.5*(R_array(:,:,i)*ak+g_vector)*dt^2;
end


plot([GPS_data.X],[GPS_data.Y],'m')
hold on;
plot(P_array(1,:), P_array(2,:),'b')
xlabel("x")
ylabel("y")
title("compare IMU data with GPS")

%%
function H = posemat(state)
    h = state/pi *180;
    % construct a SE(2) matrix element
    H = [...
        cos(h) -sin(h) 
        sin(h)  cos(h) ];
end