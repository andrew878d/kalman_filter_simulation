%kalman filter v1.2
%BEE499 Ghirmai Spring 2026

%x is the true state. The actual position or value of the object that we
%cant see.
%y(n) is sensor noise. This is not a perfect value of the true value.
%xest is the filter's estimate with ex:physics formula and sensor noise
%p(n) is the prediction error. This means how much we trust the current
%estimate.
%k(n) kalman gain helps to offset the sensor to a more accurate value.

%given values:
N=500; %amount of cycles to run in the simulation
a=0.8; 
c=0.6;
w_variance=0.1; %process noise variance
v_variance=0.2; %measurement noise variance

%create arrays of all 0s to store x and y values over time
x = zeros(1, N);
y = zeros(1, N);

%need to randomly create x(1) as the initial value
%randn generates a random number with a mean of 0 and variance of 1
x(1) = randn;

%simulation loop
for n = 2:N

   %simulation generates a random process noise:
   xnoise = sqrt(w_variance) * randn;
   %calculate next state based off the noise we just got
   x(n) = a * x(n-1) + xnoise;

   %simulation generates a random measurement noise:
   %this is like a real sensor measurment, but we randomize it here
   ynoise = sqrt(v_variance) * randn;
   y(n) = c * x(n) + ynoise;

end

%Calculate x estimate step:

%create an empty array of zeros for xest
xest = zeros(1,N);

%first xest value is randomized
xest(1) = randn;

%prediction ? how does this initial value affect the model ?
p(1) = 10*randn;

%this is the filter loop: estimates the truth using only sensor
for n=2:N

   %prediction step (time update)
   %xest is predicted using only a
   xest(n) = a * xest(n-1);
   %p(n) given by formula
   p(n) = a^2 * p(n-1) + w_variance;

   %calculate kalman gain
   k(n) = (c * p(n)) / ((c^2 * p(n)) + v_variance);

   %measurement update step (correction step)
   %correct the prediction
   xest(n) = xest(n) + k(n)* (y(n) - c * xest(n));

   %then calculate from formula
   p(n)=(1 - c * k(n)) * p(n);
 
end

%after x and xest loops are ran,
%need to get the mean squared estimate of the error
%between x and xest  to tell how well it did

%x is the truth, xest is the estimate
mse_filter = mean((x - xest) .^ 2);

%and get MSE for the sensor noise
%from y = c*x + v, the sensor noise is v = y/c
mse_sensor = mean((x - (y/c)) .^ 2);

fprintf('MSE of filter: %f. \n', mse_filter);
fprintf('MSE of sensor: %f. \n', mse_sensor);

figure;
plot(1:N, x, 'r', 1:N, xest,'b')
grid on;
xlabel('time (n)');
ylabel('Value');
legend('true state (x)', 'Filter estimate (xest)');

