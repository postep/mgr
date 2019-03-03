clear variables;
f = 1;
T = 1:20*f;
b = 1;
k = 3;
m = 10;
g = -9.81;


a = 5;
c = 1;
	

A = [0, 1; -k/m, -b/m];
B = [0; 1];
C = [0, 1];
D = [0];

sys = ss(A, B, C, D);
sysd = c2d(sys, 1/f);

S = zeros(2, 2);
x = zeros(size(T));
dx = zeros(size(T));
ddx = zeros(size(T));
FS = zeros(size(T));
M_pos_hat = zeros(size(T));
M_fs_hat = zeros(size(T));
M_fuzzy = zeros(size(T));
M_fuzzy_filter = zeros(size(T));
W = zeros(size(T));
U = zeros(size(T));
for t = T(3:end)
	F = 0;
	U(t) = g+F/m;
	S(:, t) = sysd.A*S(:, t-1) + sysd.B*U(t);
	x(t) = S(1, t) + 0.0*m*randn();
	dx(t) = S(2, t) + 0.0*m*randn();
	ddx(t) = (S(2, t) - S(2, t-1)) + 0.0*m*randn();
	FS(t) = ddx(t)*m + 0.02*m*randn() + m*g;
	FS_mean = movmean(FS(1:t), 10*f);
	M_fs_hat(t) = FS_mean(t)/g;
	M_pos_hat(t) = (-(1/f)^2*k*x(t-2) -b*(1/f)*x(t-1) -b*(1/f)*x(t-2))/(x(t)-2*x(t-1)-x(t-2)-(1/f)*U(t)-(1/f)*U(t-1));
	W(t) = dsigmf(ddx(t), [a, -c, a, c]);
	M_fuzzy(t) = W(t)*M_fs_hat(t) + (1-W(t))*M_pos_hat(t);
	M_fuzzy_filter = movmean(M_fuzzy(1:t), 10*f);
	
end;
	
t = length(T);
Time = 0:1/f:t/f;
Time = Time(1:end-1);

f = figure(1);
plot(Time, [x; dx; ddx; FS], '-');
legend('x', 'dx', 'ddx', 'FS');
xlabel('t');


f = figure(2);
plot(Time, [M_fs_hat; M_pos_hat; M_fuzzy; W], '-');
legend('M\_fs\_hat', 'M\_pos\_hat', 'M\_fuzzy', 'W');
xlabel('t');

f = figure(3);
w = -10:0.1:10;
plot(w, dsigmf(w, [a, -c, a, c]), '-');
xlabel('ddx');
ylabel('W');

