function [quat, mag_f, accel_f, arm_f] = load_imudata_fil(location,tag)
%% Load data
load(location,tag)
imu_size = size(robot.imu_data.header_timestamp,1);
mag_size = size(robot.mag_data.msg_time,1);

quat_w = robot.imu_data.orientation_quat_w;
quat_x = robot.imu_data.orientation_quat_x;
quat_y = robot.imu_data.orientation_quat_y;
quat_z = robot.imu_data.orientation_quat_z;
quat =[quat_w quat_x quat_y quat_z];

mag_x = robot.mag_data.magnetic_field_x;
mag_y = robot.mag_data.magnetic_field_y;
mag_z = robot.mag_data.magnetic_field_z;
mag = [mag_x mag_y mag_z];

accel_x = robot.imu_data.acceleration_x;
accel_y = robot.imu_data.acceleration_y;
accel_z = robot.imu_data.acceleration_z;
accel = [accel_x accel_y accel_z];

arm_x =robot.imu_data.angular_velocity_x;
arm_y =robot.imu_data.angular_velocity_y;
arm_z =robot.imu_data.angular_velocity_z;
arm = [arm_x arm_y arm_z];

%% Filtering RAW data
LPF = designfilt('lowpassfir','PassbandFrequency',0.15, ...
      'StopbandFrequency',0.25,'PassbandRipple',0.1, ...
      'StopbandAttenuation',65,'DesignMethod','kaiserwin');

HPF = designfilt('highpassfir','StopbandFrequency',0.25, ...
         'PassbandFrequency',0.35,'PassbandRipple',0.5, ...
         'StopbandAttenuation',65,'DesignMethod','kaiserwin');

mag_f = filtfilt(LPF,mag);
accel_f = filtfilt(LPF,accel);
arm_f = filtfilt(HPF,arm);

%% Plotting hist(x)
figure('name','Magnetometer Z_{axis}')
hist(mag_f(:,3),50)
title('Magnetometer Z_{axis} data')
figure('name','Accelerometer Z_{axis}')
hist(accel_f(:,3),50)
title('Accelerometer Z_{axis} data')
figure('name','Gyroscope Z_{axis}')
hist(arm_f(:,3),50)
title('Gyroscope Z_{axis} data')
%% Plotting filter
figure('name','Accelerometer')
subplot(3,1,1);
hold;
plot(1:imu_size, accel_x,'.')
plot(1:length(accel_f), accel_f(:,1),'.')
legend('RAW', 'Filtered')
title('Accelerometer X_{axis} data')
xlabel('data index');
ylabel('m^2/s');
subplot(3,1,2);
hold;
plot(1:imu_size, accel_y,'.')
plot(1:length(accel_f), accel_f(:,2),'.')
legend('RAW', 'Filtered')
title('Accelerometer Y_{axis} data')
xlabel('data index');
ylabel('m^2/s');
subplot(3,1,3);
hold;
plot(1:imu_size, accel_z,'.')
plot(1:length(accel_f), accel_f(:,3),'.')
legend('RAW', 'Filtered')
title('Accelerometer Z_{axis} data')
xlabel('data index');
ylabel('m^2/s');
legend('RAW', 'Filtered')
grid

figure('name','Magnetometer')
subplot(3,1,1);
hold;
plot(1:mag_size, mag_x,'.')
plot(1:length(mag_f), mag_f(:,1),'.')
title('Magnetometer X_{axis} data')
legend('Raw', 'Filtered')
xlabel('data index');
ylabel('Gauss');
subplot(3,1,2);
hold;
plot(1:mag_size, mag_y,'.')
plot(1:length(mag_f), mag_f(:,2),'.')
title('Magnetometer Y_{axis} data')
legend('Raw', 'Filtered')
xlabel('data index');
ylabel('Gauss');
subplot(3,1,3);
hold;
plot(1:mag_size, mag_z,'.')
plot(1:length(mag_f), mag_f(:,3),'.')
title('Magnetometer Z_{axis} data')
legend('Raw', 'Filtered')
xlabel('data index');
ylabel('Gauss');
grid

figure('name','Gyroscope')
subplot(3,1,1);
hold;
plot(1:imu_size, arm_x,'.')
plot(1:length(arm_f), arm_f(:,1),'.')
title('Gyroscope X_{axis} data')
legend('Raw', 'Filtered')
xlabel('data index');
ylabel('rad/s');
subplot(3,1,2);
hold;
plot(1:imu_size, arm_y,'.')
plot(1:length(arm_f), arm_f(:,2),'.')
title('Gyroscope Y_{axis} data')
legend('Raw', 'Filtered')
xlabel('data index');
ylabel('rad/s');
subplot(3,1,3);
hold;
plot(1:imu_size, arm_z,'.')
plot(1:length(arm_f), arm_f(:,3),'.')
title('Gyroscope Z_{axis} data')
legend('Raw', 'Filtered')
xlabel('data index');
ylabel('rad/s');
grid


%% Gyro Calibration
r_x =range(arm_f(:,1))
m_x =mean(arm_f(:,1))
med_x =median(arm_f(:,1))
var_x = var(arm_f(:,1))

r_y =range(arm_f(:,2))
m_y =mean(arm_f(:,2))
med_y =median(arm_f(:,2))

var_y = var(arm_f(:,2))
r_z =range(arm_f(:,3))
m_z =mean(arm_f(:,3))
med_z =median(arm_f(:,3))
var_z = var(arm_f(:,3))

%% Mag Calibration
r_x =range(mag_f(:,1))
m_x =mean(mag_f(:,1))
med_x =median(mag_f(:,1))
var_x = var(mag_f(:,1))
r_y =range(mag_f(:,2))
m_y =mean(mag_f(:,2))
med_y =median(mag_f(:,2))
var_y = var(mag_f(:,2))
r_z =range(mag_f(:,3))
m_z =mean(mag_f(:,3))
med_z =median(mag_f(:,3))
var_z = var(mag_f(:,3))

%% Accel Calibration
r_x =range(accel_f(:,1))
m_x =mean(accel_f(:,1))
med_x =median(accel_f(:,1))

var_x = var(accel_f(:,1))
r_y =range(accel_f(:,2))
m_y =mean(accel_f(:,2))
med_y =median(accel_f(:,2))

var_y = var(accel_f(:,2))
r_z =range(accel_f(:,3))
m_z =mean(accel_f(:,3))
med_z =median(accel_f(:,3))
var_z = var(accel_f(:,3))

end
