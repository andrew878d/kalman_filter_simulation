%kalman filter

%given values:
n=500;
a=0.8;
c=0.6;
w_variance=0.1;
v_variance=0.2;

%create arrays of all 0s to store x and y values over time
x = zeros(1, n);
y = zeros(1, n);

%need to randomly create x(1) as the initial value
%randn generates a random number with a mean of 0 and variance of 1
x(1) = randn;

%simulation loop
for n = 2:n

    % Step 1 Prediction time update
    %x(n) = ax(n-1) + wn
    
    xnoise = sqrt(w_variance) * randn;
    x(n) = a * x(n-1) + xnoise;

    % Step 2 Measurement update
    % y(n) = cx(n) + vn

    ynoise = sqrt(v_variance) * randn;
    y(n) = c * x(n) + ynoise;

end


figure;
plot(1:n, x, 'r');
hold on;
plot(1:n, y, 'b');
grid on;
xlabel('n time step');
ylabel('Value')'



