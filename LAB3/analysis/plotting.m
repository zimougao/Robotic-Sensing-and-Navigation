utm_x = stationaryISECrtkdatatrans.utmeasting;
utm_y = stationaryISECrtkdatatrans.utmnorthing;
alt = stationaryISECrtkdatatrans.altitude;
min_x = min(utm_x);
min_y = min(utm_y);
min_alt = min(alt);

utm_x = utm_x-min_x;
utm_y = utm_y-min_y;
alt = alt-min_alt;

figure()
subplot(3,1,1);
histogram(utm_x);
title('utmeasting');
subplot(3,1,2);
histogram(utm_y);
title('utmnorthing');
subplot(3,1,3);
histogram(alt);
title('altitude');
% figure();
% subplot(3,1,1);
% hold;
% plot(utm_x,'.');
% plot(utm_y,'.');
% plot(alt,'.');
% title('utm & altitude vs time');
% xlabel('time (s)');
% ylabel('utm_data (m)');
% legend('utmeasting','utmnorthing','altitude');
% 
% subplot(3,1,2);
% plot(utm_x,utm_y,'.');
% title('ISEC stationary 2D');
% xlabel('utmeasting (m)');
% ylabel('utmnorthing (m)');
% legend('utm signal');
% 
% subplot(3,1,3);
% plot3(utm_x,utm_y,alt,'.');
% title('ISEC stationary 3D');
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('z (m)');
% legend('RTK GPS signal');