close all;
clear all;

T = readtable('M8_test.csv');

figure
plot(T.Time, T.Thrust/1000);
xlabel('Time (s)')
ylabel('Thrust (kg)')
title('M8 Rocket Motor Test')

T_Impulse = (sum(T.Thrust)/1000)*0.95*9.8; %Ns 
S_Imp = T_Impulse/0.050; %50grams Prop

S_Impulse = S_Imp/9.8;

F_avg = T_Impulse/(T.Time(38)); %N

