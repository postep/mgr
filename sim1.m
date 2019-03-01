f = 1;
T = 1:200;
b = 0.5;
k = 0.4;
m = 6;
Fg = -m*9.81;

A = [0, 1; -k/m, -b/m];
B = [0; 1/m];
C = [1, 1];
D = [0];

L = acker(A',C', [0, 0])';

sys = ss(A, B, C, D);
sysd = c2d(sys, f);
S = zeros(2, 1);
for t = T(2:end)
	S(:, end+1) = sysd.A*S(:, end) + sysd.B*Fg;
end
x = S(1, :);
dx = S(2, :);
ddx = [diff(dx), 0];

% FS - this is what we get from sensor
FS = ddx*m + 0.4*m*randn(size(ddx)) - m*9.81;
plot(T, [x;dx;ddx;FS], '-');
legend('x', 'dx', 'ddx', 'FS');

% try to analize what model it is: MSE
Z = S(:,2:end);
X = S(:, 1:end-1);

P = Z*X'*(X*X')^-1
sysd.A


fc = 30; % Cut off frequency
fs = 1000; % Sampling rate

[b,a] = butter(6,fc/(fs/2)); % Butterworth filter of order 6
x = filter(b,a,FS); % Will be the filtered signal
plot(x);
hold on;
plot(FS);
plot(movmean(FS, 100));
hold off;