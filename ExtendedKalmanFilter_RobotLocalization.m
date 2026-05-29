%Extended matrix kalman filter v1.0
%BEE499 Ghirmai Spring 2026

%x is the true state. The actual position or value of the object that we
%cant see.
%y(n) is sensor noise. This is not a perfect value of the true value.
%xest is the filter's estimate with example physics formula and sensor noise
%p(n) is the prediction error. This means how much we trust the current
%estimate.
%k(n) kalman gain helps to offset the sensor to a more accurate value.

%this EKF is used for robot localization in 2 dimensions. 

clear; clc; close all;


%given values
N = 1000;      % amount of cycles to run in the simulation
T = 0.1;       % time step in seconds
v = 1.0;       % robot forward velocity (m/s)
w_turn = 0.1;  % robot turn rate (rad/s)

%given noise variances
var_pos = 0.01;   % process noise for X and Y
var_theta = 0.01; % process noise for heading
var_range = 0.5;  % sensor noise for distance
var_bear = 0.05;  % sensor noise for angle

%process noise matrix 3x3 
Q = [var_pos, 0, 0;
     0, var_pos, 0;
     0, 0, var_theta];

%measurment noise matrix 2x2
R = [var_range, 0;
     0, var_bear];

%3x3 identity matrix
I = eye(3); 

%true state vector [x-position; y-position; theta]
x = zeros(3, N);   
%Start at [1; 1] so we don't divide by zero in the Jacobian
x(:, 1) = [1; 
            1; 
            0];   

xest = zeros(3, N);    
xest(:, 1) = [1; 1; 0];

%initial uncertainty
P = eye(3) * 10; 

%sensor measurements [range; bearing]
y = zeros(2, N); 

%simulation loop
for n = 2:N
    
    %generate random noise for all 3 rows of the true state vector x.
    w_noise = [sqrt(var_pos)*randn; 
               sqrt(var_pos)*randn; 
               sqrt(var_theta)*randn];
    
    x(1, n) = x(1, n-1) + v * cos(x(3, n-1)) * T + w_noise(1);
    x(2, n) = x(2, n-1) + v * sin(x(3, n-1)) * T + w_noise(2);
    x(3, n) = x(3, n-1) + w_turn * T + w_noise(3);
    
    true_range = sqrt(x(1, n)^2 + x(2, n)^2);
    true_bearing = atan2(x(2, n), x(1, n));
    
    y(1, n) = true_range + sqrt(var_range) * randn;
    y(2, n) = true_bearing + sqrt(var_bear) * randn;
    
    %store previous estimates 
    px = xest(1, n-1);
    py = xest(2, n-1);
    theta = xest(3, n-1);
    
    %Step 1: prediction time update using non-linear physics
    xest(1, n) = px + (v * cos(theta) * T);
    xest(2, n) = py + (v * sin(theta) * T);
    xest(3, n) = theta + (w_turn * T);
    

    %NEW jacobian matrix of physics
    F = [1, 0, -v * sin(theta) * T;
         0, 1,  v * cos(theta) * T;
         0, 0,  1];
         
    %NEW jacobian matrix of sensor
    r2 = px^2 + py^2; 
    r = sqrt(r2);
    
    %NEW 
    H = [ px/r,    py/r,   0;
         -py/r2,   px/r2,  0];
         
    
    %NEW update uncertainty using Jacobian F
    P = F * P * F' + Q;
    
    %NEW expected sensor measurements
    h_x = [r; atan2(py, px)];
    
    %NEW calculate Kalman Gain using Jacobian H
    K = (P * H') / (H * P * H' + R);
    
    %NEW Step 2: measurement update (correction)
    xest(:, n) = xest(:, n) + K * (y(:, n) - h_x);
    
    %update our uncertainty
    P = (I - K * H) * P;
end

figure;
plot(x(1,:), x(2,:), 'r'); 
hold on;
plot(xest(1,:), xest(2,:), 'b');
grid on;
title('EKF robot localization');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
legend('true path', 'EKF estimate');
