function [phi] = transitionMatrix(x_hat_plus)

% resolution time in seconds
T = 0.005;
% time constants for angular rates
tau_rx = 1;
tau_ry = 1;
tau_rz = 1;

% states
x1 = x_hat_plus(1); % p
x2 = x_hat_plus(2); % q
x3 = x_hat_plus(3); % r
x4 = x_hat_plus(4); % qx
x5 = x_hat_plus(5); % qy
x6 = x_hat_plus(6); % qz
x7 = x_hat_plus(7); % qw

% Norm of the quaternion -- hopefully usually 1
n2 = sqrt(x4^2 + x5^2 + x6^2 + x7^2);

% f1 = -x1/tau_rx;
% f2 = -x1/tau_ry;
% f3 = -x1/tau_rz;
f4 = 0.5*( x3*x5 - x2*x6 + x1*x7)/n2;
f5 = 0.5*(-x3*x4 + x1*x6 + x2*x7)/n2;
f6 = 0.5*( x2*x4 - x1*x5 + x3*x7)/n2;
f7 = 0.5*(-x1*x4 - x2*x5 - x3*x6)/n2;

% Derivatives for linearization
df1_x1 = 1 - T/tau_rx;
df2_x2 = 1 - T/tau_ry;
df3_x3 = 1 - T/tau_rz;

df4_x1 = 0.5*( x7/n2);
df4_x2 = 0.5*(-x6/n2);
df4_x3 = 0.5*( x5/n2);
df4_x4 = 1 + 0.5*(-f4*x4/n2^3);
df4_x5 = 0.5*( x3/n2 - f4*x5/n2^3);
df4_x6 = 0.5*(-x2/n2 - f4*x6/n2^3);
df4_x7 = 0.5*( x1/n2 - f4*x7/n2^3);

df5_x1 = 0.5*( x6/n2);
df5_x2 = 0.5*( x7/n2);
df5_x3 = 0.5*(-x4/n2);
df5_x4 = 0.5*(-x3/n2 - f5*x4/n2^3);
df5_x5 = 1 + 0.5*(-f5*x5/n2^3);
df5_x6 = 0.5* ( x1/n2 - f5*x6/n2^3);
df5_x7 = 0.5*( x2/n2 - f5*x7/n2^3);

df6_x1 = 0.5*(-x5/n2);
df6_x2 = 0.5*( x4/n2);
df6_x3 = 0.5*( x7/n2);
df6_x4 = 0.5*( x2/n2 - f6*x4/n2^3);
df6_x5 = 0.5*(-x1/n2 - f6*x5/n2^3);
df6_x6 = 1 + 0.5*(-f6*x6/n2^3);
df6_x7 = 0.5* ( x3/n2 - f6*x7/n2^3);

df7_x1 = 0.5*(-x4/n2);
df7_x2 = 0.5*(-x5/n2);
df7_x3 = 0.5*(-x6/n2);
df7_x4 = 0.5*(-x1/n2 - f7*x4/n2^3);
df7_x5 = 0.5*(-x2/n2 - f7*x5/n2^3);
df7_x6 = 0.5*(-f7*x7/n2^3);
df7_x7 = 1 + 0.5*(-x3/n2 - f7*x6/n2^3);

phi = [df1_x1 0      0      0      0      0      0;
       0      df2_x2 0      0      0      0      0;
       0      0      df3_x3 0      0      0      0;
       df4_x1 df4_x2 df4_x3 df4_x4 df4_x5 df4_x6 df4_x7;
       df5_x1 df5_x2 df5_x3 df5_x4 df5_x5 df5_x6 df5_x7;
       df6_x1 df6_x2 df6_x3 df6_x4 df6_x5 df6_x6 df6_x7;
       df7_x1 df7_x2 df7_x3 df7_x4 df7_x5 df7_x6 df7_x7];
