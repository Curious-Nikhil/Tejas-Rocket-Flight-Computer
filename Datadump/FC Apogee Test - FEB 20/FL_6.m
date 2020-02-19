% @Nikhil Mishra 
%  TEJAS Apogee Simulator
clear all;
close all;
% clc;

p1c = 0;
p2c = 0;
p3c = 0;
pass1 = false;
delay = (30*0.1); %virtual shift for 0.5 second of drop
fallr = -0.1; %metres
adrop = 267; %Actual Drop
apop = 0; %Apogee Pass I
apop2 = 0; %Apogee Pass II
apot = 0; %Apogee Last Trigger

% --------------------------
T = readtable('6FL.csv');
% Z = readtable('1FL.csv');

% The Apo Program
for i=1:length(T.diff)
    if(T.diff(i) <= fallr)
        p1c = p1c + 1;
        apop = i;
        i = i + delay;
        
        if(T.diff(i) <= fallr)
            p2c = p2c + 1;
            apop2 = i;
            i = i + delay;
            
            if(T.diff(i) <= fallr) 
                p3c = p3c + 1;
                apot = i;
                disp('APOGGE TRIGGER');
                break;
            end
        end
    end
end

% CMD disps
disp(T.diff(apot));
disp(apot);
disp(p1c);
disp(p2c);
disp(p3c);
disp('Total Delay');
disp(T.t(apot) - T.t(apop));

% Plots:
figure 
yyaxis left
plot(T.t, T.KMF, T.t, T.alt);
xlabel('Time')
ylabel('Height(m)');
title('TEJAS Apogee Demo Test');

annotation('textbox', [0.2, 0.05, 0.1, 0.1], 'String', "Apo Delay: " + (T.t(apot) - T.t(apop))/1000 +'s')
annotation('textbox', [0.2, 0.1, 0.1, 0.1], 'String', "Total Delay: " + (T.t(apot) - T.t(adrop))/1000 +'s')
annotation('textbox', [0.2, 0.15, 0.1, 0.1], 'String', "Fall Rate: " + fallr +'m')
annotation('textbox', [0.2, 0.2, 0.1, 0.1], 'String', "ProDelay: " + delay*1000/30 +'ms')

delay = 30*0.2; %virtual shift for 0.5 second of drop
fallr = -0.1; %metres
hold on
plot(T.t(apot), T.KMF(apot),'b*');
plot(T.t(apop), T.KMF(apop),'r*');
plot(T.t(apop2), T.KMF(apop2),'g*');
plot(T.t(adrop), T.KMF(adrop),'k*');

text(T.t(apot), T.KMF(apot), 'APO FIRED')
text(T.t(apop), T.KMF(apop), 'AP-I')
text(T.t(apop2), T.KMF(apop2), 'AP-II')
text(T.t(245), T.KMF(245), 'Actual Drop')

hold off
yyaxis right
% plot(T.t, T.diff)
plot(T.t, T.ax);
ylabel('AX');
