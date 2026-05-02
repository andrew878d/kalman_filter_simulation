%kalman filter v1.4
%BEE499 Ghirmai Spring 2026

%x is the true state. The actual position or value of the object that we
%cant see.
%y(n) is sensor noise. This is not a perfect value of the true value.
%xest is the filter's estimate with example physics formula and sensor noise
%p(n) is the prediction error. This means how much we trust the current
%estimate.
%k(n) kalman gain helps to offset the sensor to a more accurate value.

clear; clc; close all;

%given values:
N=500; %amount of cycles to run in the simulation
a=0.8; 
c=0.6;
w_variance=0.1; %process noise variance

%For v_variance is calculated later in terms of Psignal
%v_variance=0.1; %measurement noise variance. 

%snr array for values 0,2,4....20.
SNR_vector = 0:2:20;
mse_array = zeros(1, length(SNR_vector));

%Note: signal to noise ratio (dB) = 10 * log (power of signal / power of
%noise

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
  
end

%for every SNR value, we need to get the mse value:
for i = 1:length(SNR_vector)
   

    %Power = Voltage^2 / R. In this project we assume R=1. 
    Psignal = mean((c*x).^2);

    v_variance = Psignal*10^(-SNR_vector(i)/10);
  
    %simulation generates a random measurement noise:
    %this is like a real sensor measurment, but we randomize it here
    ynoise = sqrt(v_variance) * randn(1,N);
    y = c * x + ynoise;


    %Calculate x estimate step:
    
    %create an empty array of zeros for xest
    xest = zeros(1,N);
    
    %first xest value is randomized
    xest(1) = randn;

    k = zeros(1, N);
    p = zeros(1, N);
    
    %prediction ? how does this initial value affect the model ?
    p(1) = 10*randn;

    
    %this is the filter loop: estimates the truth using only sensor
    %will be nested inside the SNR loop
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
    
    %also save this unique mse to snr in the array
    mse_array(i) = mse_filter;

    %save xest at SNR = 0 and 20 to be plotted later:
    if SNR_vector(i) == 0
        xest_snr0 = xest;
    end
    if SNR_vector(i) == 20
        xest_snr20 = xest;
    end
   
end


final_table = table(SNR_vector', mse_array', 'VariableNames', {'SNR (db)', 'MSE'});
disp(final_table);

figure;

%plot x and xest at snr=0
subplot(2, 2, 1);
plot(1:N, x, 'r', 1:N, xest_snr0,'b')
grid on;
legend('true state (x)', 'Filter estimate (xest)');
title('X vs Xest at SNR=0');

%plot x and xest at snr=20
subplot(2, 2, 2);
plot(1:N, x, 'r', 1:N, xest_snr20,'b')
grid on;
legend('true state (x)', 'Filter estimate (xest)');
title('X vs Xest at SNR=20');

%plot MSE and SNR values
subplot(2, 2, 3);
plot(SNR_vector, mse_array, 'o-b')
grid on;
xlabel('SNR (db)'); ylabel('mean squared error (MSE)'); legend('SNR');
title('SNR and MSE values over time');


%Test experiment: increase sensor noise over time

y_test = zeros(1, N);
xest_test = zeros(1, N);
p_test = zeros(1, N);

%need to create initial prediction value
p_test(1) = 1;

for n = 2:N

    %generate new sensor variance and sensor noise
    v_var_test = v_variance * (n/50); %this means the v_var is growing
    y_noise_test = sqrt(v_var_test) * randn;

    %generate measurement based on true state x
    %(this would be a real life sensor reading)
    y_test(n) = c * x(n) + y_noise_test;

    %prediction step (time update)
    xest_test(n) = a * xest_test(n-1);
    
    %update prediction error
    p_test(n) = a^2 * p_test(n-1) + w_variance;

    %generate new kalman gain
    k_test(n) = (c * p_test(n)) / ((c^2 * p_test(n)) + v_var_test);

    %correction step:
    xest_test(n) = xest_test(n) + k_test(n) * (y_test(n) - c * xest_test(n));
    p_test(n) = (1 - c * k_test(n)) * p_test(n);
    
end

subplot(2, 2, 4);
plot(1:N, x, 'r', 1:N, xest_test,'b')
grid on;
xlabel('time (n)');
legend('true state (x)', 'Filter estimate (xest)');
title('experiment: increasing sensor noise over time');
