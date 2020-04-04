function [utm_northing utm_easting] = gps_gt(data)
utm_e = data(:,1);
utm_n = data(:,2);
a = min(utm_e);
b = min(utm_n);

figure('name','GPS GT')
for i =1:length(utm_e)
    new1(i) = utm_e(i) -a;
    new2(i) = utm_n(i) -b;
    plot(new1(i), new2(i), '.')
    hold on
end
legend('utm')
title('GPS data')
xlabel('easting')
ylabel('northing')
grid
utm_easting = new1';
utm_northing = new2';
end