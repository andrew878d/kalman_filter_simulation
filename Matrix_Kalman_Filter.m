%Matrix kalman filter v1.0
%BEE499 Ghirmai Spring 2026

%x is the true state. The actual position or value of the object that we
%cant see.
%y(n) is sensor noise. This is not a perfect value of the true value.
%xest is the filter's estimate with example physics formula and sensor noise
%p(n) is the prediction error. This means how much we trust the current
%estimate.
%k(n) kalman gain helps to offset the sensor to a more accurate value.

clear; clc; 

%given values:
N=1000; %amount of cycles to run in the simulation
T = 0.1; %time step in seconds

A = [1 T;
    0 1];

C = [1, 0];

a=0.8; 
c=0.6;

var_w = 0.25;
var_v = 0.25;

%given that (sigma_v)^2 and (sigma_w)^2 = 0.25
sigma_w = 0.5;
sigma_v = 0.5;

%x matrix has 2 rows position, velocity and N columns
x = zeros(2, N);
%y matrix has 1 row (measured position) and N columns
y = zeros(1, N);

%given initial values of x
x(:, 1) = [0;
           1];

%simulation loop
for n = 2:N

   %simulation generates a random process noise:
   w = [0.5*T^2; T] * sqrt(var_w) * randn;

   %calculate next state based off the noise we just got
   x(:, n) = A * x(:, n-1) + w;

   %measurement update step in same loop
   %simulation generates a random measurement noise:
   %this is like a real sensor measurment, but we randomize it here
   v_noise = sqrt(var_v) * randn;
   y(n) = C * x(:, n) + v_noise;

end

figure;

%instead of plotting over n, plot over actual time
time = (0: N-1) * T;

%plot position
subplot(2,1,1);
% Plotting Truth as a solid line and Measurements as black dots
plot(time, x(1,:), 'r'); 
hold on;
plot(time, y, 'k.'); 
grid on;
title('position truth vs sensor noise');
ylabel('position (meters)');
legend('true position', 'measured y');

%plot velocity
subplot(2,1,2);
plot(time, x(2,:), 'g');
grid on;
title('true velocity');
ylabel('velocity (m/s)');
xlabel('time (seconds)');
legend('true velocity');
