clear all
% close all

T = readtable('TL2.csv');

%Roll Program Hit Detector
count = 0;
pitchabort = 45;
for i=1:length(T.time)
    if (abs(T.pitch(i)) < pitchabort || abs(T.pitch(i)) > pitchabort+90)
        count = count +1;
        disp(T.time(i))
        break;
    end
end

disp(count)


figure 
yyaxis left
hold on

plot(T.time, T.KAGL, T.time, T.AGL)
text(T.time(i), T.KAGL(i), 'Pitch Abort')
hold off
yyaxis right
plot(T.time,T.roll,'g', T.time, T.pitch,'k')


figure
plot3(T.AGL, T.roll, T.pitch)
xlabel('KAGL')
ylabel('roll')
zlabel('pitch')
