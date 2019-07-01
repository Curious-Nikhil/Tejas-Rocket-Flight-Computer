
% Calculates the Kalman gain
H = [1 0 0; 0 0 1]; % maps x (state variables) to z (sensor data)
R = [35.8229 0; 0 .0012]; % measurement noise covariance
Q = [0 0 0; 0 0 0; 0 0 1]; % process noise covariance matrix
T = .05; % time step
A = [1 T 1/2 * T^2; 0 1 T; 0 0 1]; % maps previous state to next state
% these three equations recursively define k (matrix of kalman gains)
% and P (error covariance matrix)

P = eye(3); % initial guess for p

for i = 1:20
    K = P*H'/(H*P*H' + R); % Kalman gains
P = (eye(3) - K *H)*P;
P = A*P*A' + Q;
end

display(K)
display(H)
display(P)

% T = readtable('data_bmp.csv');
% 
% height = T{:,1};
% pascal = T{:,2};

% % implements Kalman filter on altitude and accelerometer data. Required vectors are alt and accel, 
% t = .05:.05:15;
% 
% estimate = zeros(3,length(t));
% estimate(:,1) = [height(1); 0; accel(1)];
% for i = 2:length(t)
% estimate(:,i) = A*estimate(:,i-1);
% estimate(:,i) = estimate(:,i) + K*([alt(i);accel(i)] - H *estimate(:,i));
% end

