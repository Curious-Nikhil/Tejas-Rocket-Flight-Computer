%Apogee Check with Real Data:

J = readtable('Apo0FL.csv');

figure
plot(J.t, J.KMF, J.t, J.alt)
title('Apogee Drop Test 2')
yyaxis right
plot(J.t, J.Apo);

plotbrowser('on')

