function rocket_sim(t_length,t_accmax,hold_a_max,t_g,a_peak)
close all; % Close existing figures
dt = 10^-3; % Time steps of 1ms
% Default parameters that match conditions of measured rocket data
t_length = 15;
t_accmax = 0.05;
hold_a_max = 0.35;
t_g = 0.5;
a_peak = 160;
t = linspace(0,t_length,t_length*(1/dt)); % Create Time vector
g = -9.81; % Acceleration due to gravity
Acc_t = zeros(1,length(t)); % Initialize Acceleration vector
Acc_t(1:round(t_accmax*(dt^-1))) = ... % Linear rise to peak acceleration
linspace(g,a_peak,round(t_accmax*(dt^-1)));
Acc_t(round(t_accmax*(dt^-1))+1:round(hold_a_max*(dt^-1))) = a_peak;
% Hold peak acceleration for set amount of time
Acc_t(round(hold_a_max*(dt^-1))+1:round(t_g*(dt^-1)))=...
linspace(a_peak,g,round((t_g-hold_a_max)*(dt^-1))); % Linear decay to g
Acc_t(round(t_g*(dt^-1))+1:end) = g; % Hold acceleration of g for remainder
% of flight
figure;
plot(t,Acc_t) % Show plot of simulated acceleration
title('Simulated Acceleration of Rocket')
state_vecs = zeros(3,length(t)); % Initialize matrix for position,
% velocity and acceleration states
trans_mat = [1 dt (1/2)*(dt^2); 0 1 dt; 0 0 1]; % Initialize state
% derivation matrix
prv_states = zeros(3,length(t)); % Matrix for previous states
prv_states(3,:) = Acc_t; % Set acceleration vector in previous states
for i=1:length(t)-1
state_vecs(1,i) = trans_mat(1,:)*prv_states(:,i); % Derive position
state_vecs(2,i) = trans_mat(2,:)*prv_states(:,i); % Derive velocity
state_vecs(3,i) = trans_mat(3,:)*prv_states(:,i); % Derive acceleration
prv_states(1,i+1) = state_vecs(1,i); % Use current states to derive
% future values
prv_states(2,i+1) = state_vecs(2,i);
end
s_comp = state_vecs(1,:); % Position vector
v_comp = state_vecs(2,:); % Velocity vector
a_comp = state_vecs(3,:); % Acceleration vector
figure;
plot(t,s_comp) % Show plot of computed position
title('Computed Position')
figure;
plot(t,v_comp) % Show plot of computed velocity
title('Computed Velocity')
figure;
plot(t,a_comp) % Show plot of computed acceleration
title('Computed Acceleration')
[apogee,t_index] = max(s_comp); % Determine and display actual apogee
disp('Actual Apogee occurs at t =')
disp(t(t_index))


% Measurement noise
% Position corrupted with Gaussian noise with standard deviation = 5.985
s_n = state_vecs(1,:)+(5.985.*randn(1,length(t)));
% Velocity corrupted with Gaussian noise with standard deviation = 2
v_n = state_vecs(2,:)+(2.*randn(1,length(t)));
% Position corrupted with Gaussian noise with standard deviation = 0.0346
a_n = state_vecs(3,:)+(0.0346.*randn(1,length(t)));
figure;
plot(t,s_n) % Plot noisy position
title('Noisy Position')
figure;
plot(t,v_n) % Plot noisy velocity
title('Noisy Velocity')
figure;
plot(t,a_n) % Plot noisy acceleration
title('Noisy Acceleration')
%Quantization
%Time Quantized to 0.05 seconds
%Amplitude quantization is 3 meters
t_q = t(1:50:end);
s_n_q = floor(s_n(1,1:50:end)./3).*3;
v_n_q = floor(v_n(1,1:50:end)./3).*3;
a_n_q = floor(a_n(1,1:50:end)./3).*3;
figure;
plot(t_q,s_n_q) % Plot noisy and quantized position
title('Noisy and Quantized Position')
figure;
plot(t_q,v_n_q) % Plot noisy and quantized velocity
title('Noisy and Quantized Velocity')
figure;
plot(t_q,a_n_q) % Plot noisy and quantized acceleration
title('Noisy and Quantized Acceleration')