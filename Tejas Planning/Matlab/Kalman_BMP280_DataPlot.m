
close all;
T = readtable('data_bmp.csv');
[x, y] = size(KalmanBMP280);
t1 = 1:1:x;

height = KalmanBMP280{:,1};
est_alt = KalmanBMP280{:,2};
pascal = KalmanBMP280{:,3};
figure(1)
plot(t1, height, t1, est_alt)
title('Kalman Filter BMP280')


%New Data - Rapid changes in height
[x, y] = size(KBMP_2);
t2 = 1:1:x;

height1 = KBMP_2{:,1};
est_alt1 = KBMP_2{:,2};
pascal1 = KBMP_2{:,3};

figure(2)
plot(t2, height1, t2, est_alt1)
title('Basic Kalman Filter BMP280')
legend('Baseline Altitude','Basic Kalman Est Alt')