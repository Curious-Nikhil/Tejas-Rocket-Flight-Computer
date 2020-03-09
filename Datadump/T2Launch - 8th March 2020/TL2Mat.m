clear all
% close all

T = readtable('TL2.csv');


figure 
yyaxis left

plot(T.time, T.KAGL, T.time, T.AGL)

yyaxis right
plot(T.time,T.roll, T.time, T.pitch,'k')


figure
plot3(T.AGL, T.roll, T.pitch)
xlabel('KAGL')
ylabel('roll')
zlabel('pitch')
%Roll Program Hit Detector
count = 0;
for i=1:length(T.time)
    if (-T.pitch(i) < 30)
        count = count +1;
        disp(T.time(i))
        break;
    end
end

disp(count)


