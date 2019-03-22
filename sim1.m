clear variables;
f = 100;
T = 1:20*f;
b = 1;
k = 100;
m = 3;
g = -9.81;


a = 0.5*f;
c = 7/f;
	

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
M_pos = zeros(size(T));
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
	x(t) = S(1, t) + 0.00009*randn();
	dx(t) = S(2, t);
	ddx(t) = (S(2, t) - S(2, t-1));
	FS(t) = ddx(t)*m + 0.02*m*randn() + m*g;
	FS_mean = movmean(FS(1:t), 10*f);
	M_fs_hat(t) = FS_mean(t)/g;
    Tf = 1/f;
	M_pos(t) = (-k*(x(t))-b*(x(t)-x(t-1))/Tf)/((x(t)-2*x(t-1)+x(t-2))/Tf/Tf -g);
    POS_mean = movmean(M_pos(1:t), 10*f);
    M_pos_hat(t) = POS_mean(t);
	W(t) = dsigmf(ddx(t), [a, -c, a, c]);
	M_fuzzy(t) = W(t)*M_fs_hat(t) + (1-W(t))*M_pos_hat(t);
	M_fuzzy_filter = movmean(M_fuzzy(1:t), 10*f);
	
end;
	
t = length(T);
Time = 0:1/f:t/f;
Time = Time(1:end-1);

fig = figure(1);
subplot(2, 1, 1)
plot(Time, [x; dx; ddx], '-');
legend('x', 'dx', 'ddx');
xlabel('t');

subplot(2, 1, 2)
plot(Time, [FS], '-');
legend('FS');
axis([0 inf, -35 -25])
xlabel('t');

fig = figure(2);
plot(Time, [M_fs_hat; M_pos_hat; M_fuzzy; W], '-');
legend('M\_fs\_hat', 'M\_pos\_hat', 'M\_fuzzy', 'W');
xlabel('t');

fig = figure(3);
w = -100/f:0.1/f:100/f;
plot(w, dsigmf(w, [a, -c, a, c]), '-');
xlabel('ddx');
ylabel('W');

