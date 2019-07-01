%Ref Flight Data

T = readtable('flight_rawdata.csv');

t = T{:,1};
pascal = T{:,2};
alt = T{:,3};
v = T{:,3};

plot(t, alt);