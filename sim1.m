T = 1:100;
b = 0.1;
k = 0.1;
m = 2;
Fg = -m*9.81;

A = [0, 1; -k/m, -b/m];
B = [0; 1/m];
C = [0, 1];
D = [0];

sys = ss(A, B, C, D);
sysd = c2d(sys, 1);
L = acker(sysd.A',sysd.C', [0, 0])';

S = zeros(2, 1);
Y = zeros(1, 1);
S_hat = zeros(2, 1);
Y_hat = zeros(1, 1);
for t = T(2:end)
	S(:, end+1) = sysd.A*S(:, end) + sysd.B*Fg;
    Y(t) = C*S(:, t);
    S_hat(:, end+1) = sysd.A*S_hat(:, end) + sysd.B*Fg + L*(Y(t)-C*S_hat(:, end));
    Y_hat(t) = C * S_hat(:, end);
end
x = S(1, :);
dx = S(2, :);
ddx = [diff(dx), 0];

x_hat = S_hat(1, :);
dx_hat = S_hat(2, :);

% FS - this is what we get from sensor
FS = ddx*m + 0.4*m*randn(size(ddx)) - m*9.81;
plot(T, [x;dx;ddx;FS;x_hat;dx_hat], '-');
legend('x', 'dx', 'ddx', 'FS', 'xhat', 'dxhat');

% try to analize what model it is: observator