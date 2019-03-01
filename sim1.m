clear variables;
T = 1:5*100;
b = 1;
k = 3;
m = 10;

A = [0, 1; -k/m, -b/m];
B = [0; 1];
C = [0, 1];
D = [0];

sys = ss(A, B, C, D);
sysd = c2d(sys, 100);
L = acker(sysd.A',sysd.C', [0, 0])';

S = zeros(2, 1);
Y = zeros(1, 1);
S_hat = zeros(2, 1);
Y_hat = zeros(1, 1);
M_hat = zeros(size(T));
MFS_hat = zeros(size(T));
M_fuzzy = zeros(size(T));
M_fuzzy_raw = zeros(size(T));
W = zeros(size(T));
gf= 0;
for t = T(2:end)
	x = S(1, :);
	dx = S(2, :);
	ddx = [0, diff(dx)];
	FS = ddx*m + 0.4*m*randn(size(ddx)) - m*9.81;
	FS = movmean(FS,500);
	MFS_hat(t-1) = -FS(t-1)/9.81;
	M_hat(t-1) = (-k*x(t-1)-b*dx(t-1))/(ddx(t-1)-(9.81-gf/m));
    a = 5;
    c = 1;
	M_fuzzy_raw(t-1) = dsigmf(ddx(t-1), [a, -c, a, c])*MFS_hat(t-1) + (1-dsigmf(ddx(t-1), [a, -c, a, c]))*M_hat(t-1);
    W(t-1) = dsigmf(ddx(t-1), [a, -c, a, c]);
	M_fuzzy(1:t-1) = movmean(M_fuzzy_raw(1:t-1), 10);
	gf = M_fuzzy(t-1)*9.81;
	S(:, end+1) = sysd.A*S(:, end) + sysd.B*(9.81-gf/m);
    Y(t) = C*S(:, t);
    Y_hat(t) = C * S_hat(:, end);
end
x = S(1, :);
dx = S(2, :);
ddx = [0, diff(dx)];

x_hat = S_hat(1, :);
dx_hat = S_hat(2, :);

% x = zeros(size(T))
M_hat = zeros(size(T));
plot(T, [x; MFS_hat;M_hat;M_fuzzy], '-');
legend('x', 'MFShat', 'Mhat', 'Mfuzzy');
