clear all
clc
%% Load data
load('/home/mou/eece5554_roboticssensing/LAB2/src/lab2_hw/rosbag/0209/drive_1.mat', 'robot')
imu_size = size(robot.imu_data.header_timestamp,1);
mag_size = size(robot.mag_data.msg_time,1);
gps_size = size(robot.gps_data.header_timestamp,1);

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

utm_easting = robot.gps_data.utm_easting;
utm_northing = robot.gps_data.utm_northing;
utm = [utm_northing utm_easting];

%% Filtering RAW data
LPF = designfilt('lowpassfir','PassbandFrequency',0.15, ...
      'StopbandFrequency',0.25,'PassbandRipple',0.1, ...
      'StopbandAttenuation',65,'DesignMethod','kaiserwin');

HPF = designfilt('highpassfir','StopbandFrequency',0.25, ...
         'PassbandFrequency',0.35,'PassbandRipple',0.5, ...
         'StopbandAttenuation',65,'DesignMethod','kaiserwin');

mag_f = mag;
accel_f = accel;
arm_f = arm;

%% GPS data plot
% utm_gt = gps_gt(utm);

%% Magnet data plot

mag_xr_raw= mag_f(3500:7000,1);            % rotation magnetometer data
mag_yr_raw= mag_f(3500:7000,2);
mag_arfa = (max(mag_xr_raw)+min(mag_xr_raw))/2;
mag_beta = (max(mag_yr_raw)+min(mag_yr_raw))/2;
mag_hardiron = [mag_arfa mag_beta];
mag_xr = mag_xr_raw - mag_arfa;
mag_yr = mag_yr_raw - mag_beta;
mag_r = [mag_xr mag_yr];

%% ellipse_fit
[r_major,r_minor,x0,y0,phi] = ellipse_fit(mag_r);

% offset
offset = [x0,y0];
magnetometer_xy = mag_r - offset;

% rotation
R = [cos(phi),  sin(phi);
     -sin(phi), cos(phi)];
magnetometer_xy = (R * magnetometer_xy')';

% scale
scale = r_minor / r_major;
magnetometer_xy(:,1) = magnetometer_xy(:,1) * scale;

% rotate back
magnetometer_xy = (R' * magnetometer_xy')';

figure();
hold;
plot(mag_xr_raw,mag_yr_raw,'.');
plot(magnetometer_xy(:,1),magnetometer_xy(:,2),'.');
axis equal;
title('Magnetometer before and after calibration');
xlabel('x reading (Gauss)');
ylabel('y reading (Gauss)');
legend('raw','calibrated');

save('./magnetometer_calibration','offset','R','scale','mag_hardiron');

%% Yaw 

yaw_magr =atan2(-magnetometer_xy(:,2),magnetometer_xy(:,1));    
yaw_mag =yaw_magr/pi*180;

arm_zr = arm_f(3500:7000,3);
arm_zr = arm(3500:7000,3);
yaw1 = (1/40)*cumtrapz(arm_zr);
yaw_integ = wrapToPi(yaw1);
yaw_integration = yaw_integ/pi*180;

% 
yawf = 0.98*yaw1+0.02*yaw_magr;
yawfil = wrapToPi(yawf);
yawfilter = yawfil/pi*180;

%% Plotting 
figure()
hold;

plot(yaw_integration)
title('yaw')
xlabel('time')
plot(yawfilter)
plot(yaw_mag)
legend('integration','filter','magnetometer')