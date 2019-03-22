clear variables;
f = 100;
T = 1:20*f;
b = 1;
k = 20;
m = 3;
g = -9.81;


a = 1;
c = 7;
	

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
M_sys_hat = zeros(size(T));
M_sys = zeros(size(T));
for t = T(3:end)
	F = -M_fuzzy(t-1)*g;
% 	F=0;
	noise_level = 0.01;
	U(t) = g+F/m;
	S(:, t) = sysd.A*S(:, t-1) + sysd.B*U(t);
	x(t) = S(1, t) + noise_level*randn();
	dx(t) = S(2, t) + noise_level*randn();
	ddx(t) = (S(2, t) - S(2, t-1))*f;
	FS(t) = ddx(t)*m + 0.02*m*randn() + m*g;
	FS_mean = movmean(FS(1:t), 10*f);
	M_fs_hat(t) = FS_mean(t)/g;
    Tf = 1/f;
	M_pos(t) = (-k*(x(t))-b*(x(t)-x(t-1))/Tf)/((x(t)-2*x(t-1)+x(t-2))/Tf/Tf -g);
    POS_mean = movmean(M_pos(1:t), 10*f);
    M_pos_hat(t) = POS_mean(t);
	
	optim_steps = 100;
	if(t > optim_steps)
		B_hat = [0;Tf];
		A_hat = (S(:, t-optim_steps+1:t) - B_hat*U(t-optim_steps:t-1))*pinv(S(:, t-optim_steps:t-1));
		sys_hat = ss(A_hat, sysd.B, sysd.C, sysd.D, Tf);
		sysc_hat = d2c(sys_hat);
		A_hat = sysc_hat.A;
		m1 = -k/A_hat(2,1);
		m2 = -b/A_hat(2, 2);
		M_sys_hat(t) = (m1 + m2)/2;
	end;
	
	W(t) = dsigmf(ddx(t), [a, -c, a, c]);
	M_fuzzy(t) = W(t)*M_fs_hat(t) + (1-W(t))*M_sys_hat(t);
	
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
plot(Time, [M_fs_hat; M_sys_hat; M_fuzzy; W], '-');
legend('M\_fs\_hat', 'M\_sys', 'M\_fuzzy', 'W');
xlabel('t');

fig = figure(3);
w = -10:0.1/f:10;
plot(w, dsigmf(w, [a, -c, a, c]), '-');
xlabel('ddx');
ylabel('W');

