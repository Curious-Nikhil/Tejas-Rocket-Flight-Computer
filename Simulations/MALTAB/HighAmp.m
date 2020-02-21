

V = 4; %v
R = 100;
C = 5*10^-6;
t = 3*(10^-6);

I_t = (V/R)*exp(-t/(R*C));

disp('I = ')
disp(I_t)



%https://careertrend.com/how-8780713-increase-amperage-capacitors-diodes.html