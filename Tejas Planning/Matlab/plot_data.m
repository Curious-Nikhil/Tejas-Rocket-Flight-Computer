close all;

T = readtable('data_bmp.csv');

height = T{:,1};
pascal = T{:,2};

subplot(1,2,1)
plot(height, 'r')
xlabel('Readings')
ylabel('Height')
title('BMP280 Height Plot');

%Low Pass Filter
[b, a] = butter(4, 0.01, 'low');
b_h = filter(b, a, height);
subplot(1,2,2)
plot(b_h, 'g')
xlabel('Readings')
ylabel('Pressure')
title('BMP280 Pressure Plot');