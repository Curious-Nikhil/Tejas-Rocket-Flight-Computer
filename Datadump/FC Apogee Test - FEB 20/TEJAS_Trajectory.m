% Rocket Flight Calcs
% http://www.rocketmime.com/rockets/rckt_eqn.html#Method
% @Nikhil Mishra

clear all;
close all;

M = 0.350; %Kg
rho = 1.2;
Cd = 0.75;
g = 9.8;
T = 8.39;
r = (4.8/2)/100; %m
A = pi*r^2;
k = 0.5*rho*Cd*A; %Wind Resistance Factor
T_Imp = 31.52*0.9; %10% Error
Bt = 3;
F = M*0.98;

q = sqrt((T - M*g)/k);
x = 2*k*q/M;

v = q*(1 - exp(-x*Bt))/(1+exp(-x*Bt));

yb = (-M/(2*k))*log((T - M*g - k*v^2)/(T - M*g));

yc = (M/(2*k))*log((M*g + k*v^2)/(M*g));

H = yb + yc;

qa = sqrt(M*g / k);
qb = sqrt(g*k/M);
ta = atan(v/qa)/qb;

Flight_t = Bt + ta;

disp('Max V')
disp(v)
disp('Apogee')
disp(H)
disp('Flight Time')
disp(Flight_t)

% boost phase by adding
% the empty weight of the rocket,
% the weight of the empty motor casing, and
% half the weight of the propellant.
% During the coast phase I use
% 
% the empty weight of the rocket, and
% the weight of the empty motor casing,
% THAT'S IT, since the propellant's all burned up at this point.