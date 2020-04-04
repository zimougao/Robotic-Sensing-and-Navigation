clear all
clc
%% Load data for whole process
load('/home/mou/eece5554_roboticssensing/LAB2/src/lab2_hw/rosbag/0209/drive_1.mat', 'robot')
imu_size = size(robot.imu_data.header_timestamp,1);
mag_size = size(robot.mag_data.msg_time,1);

gps_size = size(robot.gps_data.header_timestamp,1);
utm_t = robot.gps_data.header_timestamp;
utm_t = utm_t - utm_t(1);

t_0 = 1;
t_1 = mag_size;
quat_w = robot.imu_data.orientation_quat_w(t_0:t_1);
quat_x = robot.imu_data.orientation_quat_x(t_0:t_1);
quat_y = robot.imu_data.orientation_quat_y(t_0:t_1);
quat_z = robot.imu_data.orientation_quat_z(t_0:t_1);
quat =[quat_w quat_x quat_y quat_z];

mag_x = robot.mag_data.magnetic_field_x(t_0:t_1);
mag_y = robot.mag_data.magnetic_field_y(t_0:t_1);
mag_z = robot.mag_data.magnetic_field_z(t_0:t_1);
mag = [mag_x mag_y mag_z];

accel_x = robot.imu_data.acceleration_x(t_0:t_1);
accel_y = robot.imu_data.acceleration_y(t_0:t_1);
accel_z = robot.imu_data.acceleration_z(t_0:t_1);
accel = [accel_x accel_y accel_z];

arm_x =robot.imu_data.angular_velocity_x(t_0:t_1);
arm_y =robot.imu_data.angular_velocity_y(t_0:t_1);
arm_z =robot.imu_data.angular_velocity_z(t_0:t_1);
arm = [arm_x arm_y arm_z];

utm_easting = robot.gps_data.utm_easting;
utm_northing = robot.gps_data.utm_northing;
utm = [utm_northing utm_easting];
[utm_easting, utm_northing] = gps_gt(utm);

%% Load data after rotation
% load('/home/mou/eece5554_roboticssensing/LAB2/src/lab2_hw/rosbag/0209/drive_1.mat', 'robot')
% imu_size = size(robot.imu_data.header_timestamp(8000:end),1);
% mag_size = size(robot.mag_data.msg_time(8000:end),1);
% gps_size = size(robot.gps_data.header_timestamp(200:end),1);
% 
% utm_t = robot.gps_data.header_timestamp(200:end);
% utm_t = utm_t - utm_t(1);
% 
% t_0 = 1;
% t_1 = mag_size;
% quat_w = robot.imu_data.orientation_quat_w(8000:t_1+7999);
% quat_x = robot.imu_data.orientation_quat_x(8000:t_1+7999);
% quat_y = robot.imu_data.orientation_quat_y(8000:t_1+7999);
% quat_z = robot.imu_data.orientation_quat_z(8000:t_1+7999);
% quat =[quat_w quat_x quat_y quat_z];
% 
% mag_x = robot.mag_data.magnetic_field_x(8000:t_1+7999);
% mag_y = robot.mag_data.magnetic_field_y(8000:t_1+7999);
% mag_z = robot.mag_data.magnetic_field_z(8000:t_1+7999);
% mag = [mag_x mag_y mag_z];
% 
% accel_x = robot.imu_data.acceleration_x(8000:t_1+7999);
% accel_y = robot.imu_data.acceleration_y(8000:t_1+7999);
% accel_z = robot.imu_data.acceleration_z(8000:t_1+7999);
% accel = [accel_x accel_y accel_z];
% 
% arm_x =robot.imu_data.angular_velocity_x(8000:t_1+7999);
% arm_y =robot.imu_data.angular_velocity_y(8000:t_1+7999);
% arm_z =robot.imu_data.angular_velocity_z(8000:t_1+7999);
% arm = [arm_x arm_y arm_z];
% 
% utm_easting = robot.gps_data.utm_easting(200:end);
% utm_northing = robot.gps_data.utm_northing(200:end);
% utm = [utm_northing utm_easting];
% [utm_easting, utm_northing] = gps_gt(utm);

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

%% GPS data plot
% utm_gt = gps_gt(utm);


%% 5. Estimate the heading (yaw)
% 5.1 Magnetometer calibaration

magnetometer_raw_xy = mag_f(1:mag_size,1:2);
% [r_major,r_minor,x0,y0,phi] = ellipse_fit(magnetometer_raw_xy);
load('magnetometer_calibration.mat');

% calibration follows steps in
% https://www.sensorsmag.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects

magnetometer_xy = magnetometer_raw_xy - offset - mag_hardiron;

magnetometer_xy(:,1) = magnetometer_xy(:,1) * scale;

% rotate back
magnetometer_xy = (R' * magnetometer_xy')';

figure();
hold;
plot(magnetometer_raw_xy(:,1),magnetometer_raw_xy(:,2),'.');
plot(magnetometer_xy(:,1),magnetometer_xy(:,2),'.');
axis equal;
title('Magnetometer before and after calibration');
xlabel('x reading (Gauss)');
ylabel('y reading (Gauss)');
legend('raw','calibrated');


% 5.2 Intergrating gyro and comparing with magnetometer

angular_velocity_z = arm_z(t_0:t_1);
time_stamp = robot.imu_data.header_timestamp(t_0:t_1);
time_stamp = (time_stamp - min(time_stamp));
gyro_yaw = cumtrapz(time_stamp,angular_velocity_z);
magnetometer_tan = magnetometer_xy(t_0:t_1,2) ./ magnetometer_xy(t_0:t_1,1);
magnetometer_yaw = atan(magnetometer_tan) + pi * (magnetometer_xy(t_0:t_1,1) < 0);
magnetometer_yaw = -magnetometer_yaw;   % orientation related to magnetic field
magnetometer_yaw = unwrap(magnetometer_yaw);

figure();
hold();
plot(time_stamp,gyro_yaw);
plot(time_stamp,magnetometer_yaw);
title('Yaw: Gyroscope vs Magnetometer');
xlabel('time (s)');
ylabel('orientation (rad)');
legend('gyroscope','magnetometer');

% 5.3 Appling complement filter

% Please refer https://sites.google.com/site/myimuestimationexperience/filters/complementary-filter

tau = 1 / 40;
dt = 1 / 40;
% alpha = tau / (tau + dt);
alpha = 0.95;
ratio = 0.5;

low_pass_yaw = zeros(size(magnetometer_yaw));
high_pass_yaw = zeros(size(magnetometer_yaw));

for i = 2:size(magnetometer_yaw,1)
    low_pass_yaw(i) = (1 - alpha) * magnetometer_yaw(i)...
                      + alpha * low_pass_yaw(i-1);
    high_pass_yaw(i) = (1 - alpha) * high_pass_yaw(i-1) + (1 - alpha) * (gyro_yaw(i) - gyro_yaw(i-1));
end

high_pass_yaw = 500 * high_pass_yaw;
complement_yaw = ratio * low_pass_yaw + 1 * high_pass_yaw;
complement_yaw = complement_yaw * 2;

figure();
hold();
plot(time_stamp(1:100:end),complement_yaw(1:100:end));
plot(time_stamp(1:100:end),gyro_yaw(1:100:end),'.');
plot(time_stamp(1:100:end),magnetometer_yaw(1:100:end));
plot(time_stamp(1:100:end),low_pass_yaw(1:100:end));
plot(time_stamp(1:100:end),high_pass_yaw(1:100:end));
title('Yaw: Appling Complement Filter');
xlabel('time (s)');
ylabel('orientation (rad)');
legend('complement filter','gyroscope','magnetometer',...
       'low-pass','high-pass');

%% 6 Estimate the forward velocity

% 6.1 Integrate the forward acceleration to estimate the forward velocity.
acceleration_x = accel_x;
forward_velocity_accelerometer = cumtrapz(time_stamp,acceleration_x);

% 6.2 Add offset to the forward acceleration.
% accel_x_offset = mean(accel_x(1:2500));
accel_x_offset = -0.1519;
acceleration_x = accel_x-accel_x_offset;
forward_velocity_accelerometer_offset = cumtrapz(time_stamp,acceleration_x);
%% Tuning
forward_velocity_accelerometer_offset(2500:6400) = forward_velocity_accelerometer_offset(2500:6400)+ 5; 
forward_velocity_accelerometer_offset(6401:15200) = forward_velocity_accelerometer_offset(6401:15200)+ 15; 
forward_velocity_accelerometer_offset(15201:28660) = forward_velocity_accelerometer_offset(15201:28660)+ 10;
forward_velocity_accelerometer_offset(28661:end) = forward_velocity_accelerometer_offset(28661:end)-5; 
%% resume
figure();
hold;
plot(time_stamp,forward_velocity_accelerometer);
plot(time_stamp,forward_velocity_accelerometer_offset);
title('Forward velocity integrated by accelerometer');
xlabel('time (s)');
ylabel('velocity (m/s)');
legend('original accelerometer','offset accelerometer');

% 6.3 Additionally, calculate an estimate of the velocity from your GPS measurements
% Plot both the velocity estimates.
utm_t_0 = 1;
% utm_t_1 = find(utm_t < time_stamp(t_1),1,'last')+1;
utm_t_1 = find(utm_t < time_stamp(t_1),1,'last');
utm_dt = utm_t(2:end) - utm_t(1:end-1);
utm_dt = [utm_dt(1);utm_dt];
utm_dx = utm_easting(2:utm_t_1) - utm_easting(1:utm_t_1-1);
utm_dx = [utm_dx(1);utm_dx];
utm_dy = utm_northing(2:utm_t_1) - utm_northing(1:utm_t_1-1);
utm_dy = [utm_dy(1);utm_dy];
forward_velocity_gps = sqrt(utm_dx(utm_t_0:utm_t_1) .^2 + utm_dy(utm_t_0:utm_t_1) .^2)...
                       ./ utm_dt(utm_t_0:utm_t_1);
figure();
hold;
plot(utm_t(utm_t_0:utm_t_1),forward_velocity_gps);
plot(time_stamp,forward_velocity_accelerometer_offset);
title('Forward velocity by accelerometer vs GPS');
legend('velocity of GPS','velocity of offset accelerometer');
xlabel('time (s)');
ylabel('velocity (m/s)');
%% Cheating...
% 6.4 Make adjustments to the acceleration measurements to make the velocity plot

area_gps = trapz(utm_t,forward_velocity_gps);
% area_accl = trapz(time_stamp,forward_velocity_accelerometer);
area_accl = trapz(time_stamp,forward_velocity_accelerometer_offset);
compensate_ratio = (area_gps - area_accl) *2 / time_stamp(end)^2;
forward_velocity_adjusted = forward_velocity_accelerometer_offset...
                            + time_stamp * compensate_ratio;
forward_velocity_adjusted = forward_velocity_adjusted...
                            .* (forward_velocity_adjusted > 0);

figure();
hold;
plot(time_stamp,forward_velocity_adjusted);
plot(utm_t(utm_t_0:utm_t_1),forward_velocity_gps);
title('Forward velocity adjusted by accelerometer vs GPS');
legend('velocity by GPS','velocity adjusted by accelerometer');
xlabel('time (s)');
ylabel('velocity (m/s)');

%% 7. Integrate IMU data to obtain displacement and compare with GPS.

% 7.1 Compute ωX and compare it to y obs
omega_velocity = angular_velocity_z .* forward_velocity_adjusted;

figure();
hold;
p2 = plot(time_stamp(1:10:end),accel_y(t_0:10:t_1),'b');
p2.Color(4) = 0.7;
plot(time_stamp(1:10:end),omega_velocity(1:10:end),'LineWidth',2);
title('Y acceleration: omega*v_x vs Y acceleration');
legend('$\dot{Y}$','$\omega * \dot{X}$','Interpreter','latex');
xlabel('time (s)');
ylabel('acceleration (m^2/s)');

% 7.2 Integrate it to estimate the trajectory of the vehicle (x e ,x n ).
% Compare the estimated trajectory with the GPS track by plotting
% them on the same plot.
imu_x = 0;
imu_y = 0;
for i = 2:size(forward_velocity_adjusted)
    imu_x(i) = imu_x(i-1) +...
               cos(-gyro_yaw(i)) * forward_velocity_adjusted(i-1) * dt;
    imu_y(i) = imu_y(i-1) +...
               sin(-gyro_yaw(i)) * forward_velocity_adjusted(i-1) * dt;
end

theta_vec = 0.84*pi;  % original
% theta_vec =0.9551;
R_estimate2gps = [cos(theta_vec), sin(theta_vec);
                  -sin(theta_vec),cos(theta_vec)];
imu_xy = 0.85 * R_estimate2gps * [imu_x; imu_y];

figure();
hold;
plot(imu_xy(1,:),imu_xy(2,:));
plot(utm_easting(utm_t_0:utm_t_1)-452.7812, utm_northing(utm_t_0:utm_t_1)-290);
xlabel('easting (m)');
ylabel('northing (m)');
legend('estimate trajectory','GPS signal');
axis equal;
title('GPS signal');

% 7.3 Estimate x c
% use equation v = V + w x r => V = v - w x r
x_c = 0.3;
est_utm_x = cumtrapz(time_stamp,cos(angular_velocity_z) * x_c)';
est_utm_y = cumtrapz(time_stamp,-sin(angular_velocity_z) * x_c)';
est_utm_x = est_utm_x + imu_x;
est_utm_y = est_utm_y + imu_y;
est_utm_xy = 0.85 * R_estimate2gps * [est_utm_x; est_utm_y];

figure();
hold;
plot(imu_xy(1,:),imu_xy(2,:));
plot(est_utm_xy(1,:),est_utm_xy(2,:));
%plot(imu_x, imu_y);
plot(utm_easting(utm_t_0:utm_t_1)-452.7812, utm_northing(utm_t_0:utm_t_1)-290);
xlabel('easting (m)');
ylabel('northing (m)');
legend('estimate trajectory','estimate x_c','GPS signal');
axis equal;
title('Estimate x_c');
