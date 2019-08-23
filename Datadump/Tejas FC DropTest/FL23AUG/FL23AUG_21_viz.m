close all
FL23AUG21
%Apoggee Check

%Figure 1
figure
yyaxis left
plot(FL23AUG21.Time, FL23AUG21.alt, FL23AUG21.Time, FL23AUG21.KMF, '--k')

hold on
plot(FL23AUG21.Time(384), FL23AUG21.alt(384), 'r*', FL23AUG21.Time(396), FL23AUG21.alt(396), 'k*')

text(FL23AUG21.Time(384), FL23AUG21.alt(384),'\leftarrow Alt 4.79, 543m')
text(FL23AUG21.Time(396), FL23AUG21.KMF(396),'\leftarrow KMF 4.827, 538.56')
ylabel("Height(m)")
yyaxis right
plot(FL23AUG21.Time, FL23AUG21.Apo, FL23AUG21.Time, FL23AUG21.Apo_KMF,'-c')

title("Apogee Check - DiffHeight")
xlabel("time(ms)")
ylabel("Differnece Height(m) - (Current - Previous)")
grid on
legend('Altitude - RAW','Altitude - KMF','MAX RAW ALT','KMF MAX ALt','Apogee - RAW', 'Apogee - KMF')



%Figure 2
figure

yyaxis left
plot(FL23AUG21.Time, FL23AUG21.alt, FL23AUG21.Time, FL23AUG21.KMF, '--k')

hold on
plot(FL23AUG21.Time(384), FL23AUG21.alt(384), 'r*', FL23AUG21.Time(396), FL23AUG21.alt(396), 'k*')

text(FL23AUG21.Time(384), FL23AUG21.alt(384),'\leftarrow Alt 4.79, 543m')
text(FL23AUG21.Time(396), FL23AUG21.KMF(396),'\leftarrow KMF 4.827, 538.56')

xlabel("Time");
ylabel("Height");
title("Kalman Filter");

yyaxis right
plot(FL23AUG21.Time, FL23AUG21.ax, 'r')
hold on
plot(FL23AUG21.Time, FL23AUG21.ay, 'g')
plot(FL23AUG21.Time, FL23AUG21.az, 'b')
hold off

ylabel("Accelerometer -x, y, z")
