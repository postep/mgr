clear variables;
f = 10;
T = 1:30*f;
b = 1;
k = 20;
m = 3;
r = 1;
I = m*r*r;
g = -9.81;
name = 'test'

a = 0.4;
c = 2;
	
optim_steps = 10;
noise_level = 0.01;

A = [0, 1; -k/I, -b/I];
B = [0; 1];
C = [0, 1];
D = [0];

sys = ss(A, B, C, D);
sysd = c2d(sys, 1/f);


S = zeros(2, 2);
q = zeros(size(T));
dq = zeros(size(T));
ddq = zeros(size(T));
FS_x = zeros(size(T));
FS_y = zeros(size(T));
FS_t = zeros(size(T));
M_pos = zeros(size(T));
M_pos_hat = zeros(size(T));
M_fs_hat = zeros(size(T));
M_fuzzy = zeros(size(T));
M_fuzzy_filter = zeros(size(T));
W = zeros(size(T));
U = zeros(size(T));
I_sys = zeros(size(T));
M_sys_hat = zeros(size(T));
F_ods = zeros(size(T));
FS_F = zeros(size(T));
M_ods = zeros(size(T));
M_ods_hat = zeros(size(T));
r_ods_hat = zeros(size(T));
r_ods = zeros(size(T));
i = 0;
for t = T(3:end)
	%symulacja urzadzenia
    Ext_T = -M_fuzzy(t-1)*g;
% 	if (t < 1000)
% 		F = -3.1*g;
% 	end;
 	Ext_T = 0;
	U(t) = m*g*cos(q(t))/I+Ext_T;
	S(:, t) = sysd.A*S(:, t-1) + sysd.B*U(t);
	q(t) = S(1, t);
	dq(t) = (S(1, t)-S(1, t-1))*f;
	ddq(t) = (S(1, t) - 2*S(1, t-1) + S(1, t-2))*f;
    F_ods(t) = m*ddq(t)^2*r;
	FS_y(t) = m*g + F_ods(t)*sin(q(t)) + noise_level*2*m*randn();
    FS_x(t) = F_ods(t)*cos(q(t)) + noise_level*2*m*randn();
    FS_t(t) = I*ddq(t);
	
    
    %algorytm naiwny
    FS_F(t) = FS_y(t);
    FS_mean(1:t) = movmean(FS_F(1:t), 10);
	M_fs_hat(t) = FS_mean(t)/g;
    
    %algorytm fts ods
    diff = 10;
    if t>diff
%         Fy1 = FS_y(t-diff);
%         Fy2 = FS_y(t);
%         q1 = q(t-diff);
%         q2 = q(t);
%         dq1 = dq(t-diff);
%         dq2 = dq(t);
%         r_ods(t) = (Fy1-Fy2)*g/(Fy2*dq1^2*sin(q1) -Fy1*dq2^2*sin(q2));
%         M_ods(t) = Fy1/(g + dq1^2*r_ods(t)*sin(q1));
		
		last_q = q(t-diff:t);
		last_dq = dq(t-diff:t);
		last_Fy = FS_y(t-diff:t);
 		Fy = @(E) E(1)*(g+(last_dq.^2)*E(2).*sin(last_q)) -last_Fy;
		e0 = [0, 0];
		e = lsqnonlin(Fy,e0);
		
		r_ods(t) = e(2);
		M_ods(t) = e(1);
     end
    
    % algorytm macierzy stanu
    Tf = 1/f;
	if(t > optim_steps)
		B_hat = [0;Tf];
		A_hat = (S(:, t-optim_steps+1:t) - B_hat*U(t-optim_steps:t-1))*pinv(S(:, t-optim_steps:t-1));
		sys_hat = ss(A_hat, sysd.B, sysd.C, sysd.D, Tf);
		sysc_hat = d2c(sys_hat);
		A_hat = sysc_hat.A;
		I1 = -k/A_hat(2,1);
		I2 = -b/A_hat(2, 2);
		I_sys(t) = (I1 + I2)/2;
	end;
	temp =  movmean(I_sys(1:t), 10);
	I_sys_hat(t) = temp(end);
    disc_ddq = (q(t)-2*q(t-1)+q(t-2))/Tf/Tf;
    disc_dq = (q(t)-q(t-1))/Tf;
    M_sys_hat(t) = (I_sys_hat(t)*ddq(t) - k*q(t) - b*dq(t) - I_sys_hat(t)*U(t))/g/cos(q(t));
	
	W(t) = dsigmf(ddq(t), [a, -c, a, c]);
% 	W(t) = 1;
	M_fuzzy(t) = W(t)*M_fs_hat(t) + (1-W(t))*I_sys_hat(t);
end;
M_ods = movmean(M_ods, 10);
r_ods = movmean(r_ods, 10);
% r_ods = medfilt1(r_ods, 3);
t = length(T);
Time = 0:1/f:t/f;
Time = Time(1:end-1);


fig = figure(1);
subplot(2, 1, 1)
plot(Time, [q; dq; ddq], '-');
legend('q', 'dq', 'ddq');
xlabel('t');

subplot(2, 1, 2)
plot(Time, [FS_x; FS_y; FS_t], '-');
legend('Fx', 'Fy', 'T');
xlabel('t');
orient(fig,'landscape');
print(fig,['img/', name, '_sys.pdf'],'-dpdf', '-fillpage');

fig = figure(2);
plot(Time, [M_fs_hat; I_sys_hat; M_ods], '-');
legend('M\_fs', 'I\_sys', 'M\_ods');
% plot(Time(10:end), [M_sys_hat(10:end)], '-');
% axis([0 inf, 0 4])
% legend('M\_sys\_hat');

xlabel('t');
orient(fig,'landscape');
print(fig,['img/', name, '_mass.pdf'],'-dpdf', '-fillpage');

% fig = figure(3);
% w = -10:0.1/f:10;
% plot(w, dsigmf(w, [a, -c, a, c]), '-');
% xlabel('ddx');
% ylabel('W');
% orient(fig,'landscape');
% print(fig,['img/', name, '_w.pdf'],'-dpdf', '-fillpage');
