function xprime = processModel(t,x)

% time constants for angular rates
tau_rx = 1;
tau_ry = 1;
tau_rz = 1;

n = 2*sqrt(x(4)^2+x(5)^2+x(6)^2+x(6)^2);
xprime = [-x(1)/tau_rx;
          -x(2)/tau_ry;
          -x(3)/tau_rz;
          ( x(3)*x(5)-x(2)*x(6)+x(1)*x(7))/n;
          (-x(3)*x(4)+x(1)*x(6)+x(2)*x(7))/n;
          ( x(2)*x(4)-x(1)*x(5)+x(3)*x(7))/n;
          (-x(1)*x(4)-x(2)*x(5)-x(3)*x(6))/n];
