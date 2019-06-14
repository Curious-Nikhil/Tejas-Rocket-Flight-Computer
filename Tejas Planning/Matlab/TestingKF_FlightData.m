close all;

%columns
t = SRP4Flight7{:,1}; %Time
p = SRP4Flight7{:,2};
a = SRP4Flight7{:,3}; %Alitude
v = SRP4Flight7{:,4};
% figure(1)
% plot(t, a)

% Make up some parameters to get sample data
numElements = 2338; % Whatever...
Mag =5;
% Add noise only between elements 1000 and 2000
noiseSignal = Mag*rand(numElements, 1);


%Adding Noise
a_noise = a + noiseSignal;

figure(1)
plot(t,a,t,a_noise, 'r')

%Applying 1 - Var Kalman Filter
%UpdateEstimate
%config - (1, 1, 0.01);
err_measure = 1;
err_estimate = 1;
q = 0.01;

for i=1:100
    k_gain = err_estimate/(err_estimate + err_measure);

    current_estimate = last_estimate + k_gain * (mea - last_esitmate);

    err_estimate = (1 - k_gain)*err_estimate + fabs(last_estimate - current_estimate) * q;

    last_estimate = current_estimate;
end

plot(t, last_estimate)
