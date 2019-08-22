close all

%Apoggee Check

%Figure 1
figure
yyaxis left
plot(FL21AUG.Time, FL21AUG.alt, FL21AUG.Time, FL21AUG.KMF, '--k')

hold on
plot(FL21AUG.Time(384), FL21AUG.alt(384), 'r*', FL21AUG.Time(396), FL21AUG.alt(396), 'k*')

text(FL21AUG.Time(384), FL21AUG.alt(384),'\leftarrow Alt 4.79, 543m')
text(FL21AUG.Time(396), FL21AUG.KMF(396),'\leftarrow KMF 4.827, 538.56')
ylabel("Height(m)")
yyaxis right
plot(FL21AUG.Time, FL21AUG.Apo, FL21AUG.Time, FL21AUG.Apo_KMF,'-c')

title("Apogee Check - DiffHeight")
xlabel("time(ms)")
ylabel("Differnece Height(m) - (Current - Previous)")
grid on
legend('Altitude - RAW','Altitude - KMF','MAX RAW ALT','KMF MAX ALt','Apogee - RAW', 'Apogee - KMF')



%Figure 2
figure

yyaxis left
plot(FL21AUG.Time, FL21AUG.alt, FL21AUG.Time, FL21AUG.KMF, '--k')

hold on
plot(FL21AUG.Time(384), FL21AUG.alt(384), 'r*', FL21AUG.Time(396), FL21AUG.alt(396), 'k*')

text(FL21AUG.Time(384), FL21AUG.alt(384),'\leftarrow Alt 4.79, 543m')
text(FL21AUG.Time(396), FL21AUG.KMF(396),'\leftarrow KMF 4.827, 538.56')

xlabel("Time");
ylabel("Height");
title("Kalman Filter");

yyaxis right
plot(FL21AUG.Time, FL21AUG.ax, 'r')
hold on
plot(FL21AUG.Time, FL21AUG.ay, 'g')
plot(FL21AUG.Time, FL21AUG.az, 'b')
hold off

ylabel("Accelerometer -x, y, z")
