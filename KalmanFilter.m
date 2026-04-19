%kalman filter v1.1

%given values:
N=500; %amount of cycles to run in the simulation
a=0.8;
c=0.6;
w_variance=0.1;
v_variance=0.2;

%create arrays of all 0s to store x and y values over time
x = zeros(1, N);
y = zeros(1, N);

%need to randomly create x(1) as the initial value
%randn generates a random number with a mean of 0 and variance of 1
x(1) = randn;

%simulation loop
for n = 2:N

   % ? Why do this step ?
   xnoise = sqrt(w_variance) * randn;
   x(n) = a * x(n-1) + xnoise;

   %this is like a real sensor measurment, but we randomize it here
   ynoise = sqrt(v_variance) * randn;
   y(n) = c * x(n) + ynoise;

end

%Calculate x estimate step:

%create an empty array of zeros for xest
xest = zeros(1,N);

%first xest value is randomized
xest(1) = randn;

%prediction
p(1) = 10*randn;

for n=2:N

   %prediction step (time update)
   xest(n) = a * xest(n-1);
   p(n) = a^2 * p(n-1) + w_variance;

   %calculate kalman gain
   k(n) = (c * p(n)) / ((c^2 * p(n)) + v_variance);

   %measurement update step (correction step)
   xest(n) = xest(n) + k(n)* (y(n) - c * xest(n));

   %then calculate
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

