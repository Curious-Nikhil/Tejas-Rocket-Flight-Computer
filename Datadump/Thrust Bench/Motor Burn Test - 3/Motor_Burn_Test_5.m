close all;


% plot(thrust.times, thrust.weight)
% title("Motor Burn Test #3")
% xlabel("Time")
% ylabel("Force")

f = (thrust.weight/1000)*9.81;

h = plot(thrust.times,f);
title("Motor Burn Test #3")
xlabel("Time (s)")
ylabel("Force (N)")

h.MarkerIndices = 5:5:length(thrust.times);
