slope_times = [0, 10, 20, 50];
slope       = [1,  3,  5,  7];  %you need to recheck the initial slope
x = 0:10:100;
times = linspace(slope_times(1), slope_times(end));
m = interp1(slope_times, slope, times);
for k = 1 : length(times)
  y = m(k) * x;
  plot(x, y);
  title( sprintf('t = %.1f', times(k)) );
  hold all
  pause( 0.5 );
end
hold off