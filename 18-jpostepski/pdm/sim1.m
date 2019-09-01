clear variables;
f = 10;
T = 1:30*f;
Tp = 2*f;
Tk = 3*f;
b = 2;
k = 16;
m = 2;
r = 1.5;
I = m*r*r;
g = -9.81;
name = 'test'

a = 0.4;
c = 2;
	
optim_steps_sys = 20;
optim_steps_ods = 10;
noise_level = 0.02;

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
FS_F = zeros(size(T));
FS_x = zeros(size(T));
FS_y = zeros(size(T));
FS_m = [FS_x; FS_y];
FS_t = zeros(size(T));
U = zeros(size(T));
M_fs_hat = zeros(size(T));
I_sys = zeros(size(T));
F_ods = zeros(size(T));
M_ods = zeros(size(T));
R_ods = zeros(size(T));
I_ods = zeros(size(T));
I_fs = zeros(size(T));
Us = zeros(size(T));
Us_exp = zeros(size(T));
Akt = zeros(size(T));
for t = T(3:end)
	%symulacja urzadzenia
    Akt(t) = min(max((t-Tp)/(Tk-Tp), 0), 1);
    if abs(I_ods(t-1)) > 0.1
%         Us(t) = -M_ods(t-1)*g*R_ods(t-1)*cos(q(t-1))/I_ods(t-1);
    end;
    Us(t) = Us(t)*Akt(t);
	U(t) = m*g*r*cos(q(t-1))/I + Us(t);
    S(:, t) = sysd.A*S(:, t-1) + sysd.B*U(t);
	q(t) = S(1, t);
    dq(t) = S(2, t);
	ddq(t) = (S(2, t)-S(2, t-1))*f;
    
    Us_exp(t) = -m*g*r*cos(q(t))/I;
	
    F_ods(t) = m*r*ddq(t)^2;
	FS_y(t) = m*g + F_ods(t)*sin(q(t)) + noise_level*2*m*randn();
    FS_x(t) = F_ods(t)*cos(q(t)) + noise_level*2*m*randn();
    Fs_m(1,t) = FS_x(t);
    FS_m(2,t) = FS_y(t);
    FS_t(t) = I*ddq(t) + noise_level*m*randn();
    
    I_fs(t) = FS_t(t)/ddq(t);
	
    
    %algorytm naiwny
    FS_F(t) = FS_y(t);
    FS_mean(1:t) = movmean(FS_F(1:t), 10);
	M_fs_hat(t) = FS_mean(t)/g;
    
    %algorytm fts ods
    if t>optim_steps_ods
		last_q = q(t-optim_steps_ods:t);
		last_ddq = ddq(t-optim_steps_ods:t);
		last_Fy = FS_y(t-optim_steps_ods:t);
        last_T = FS_t(t-optim_steps_ods:t);
 		Fyfun = @(E) E(1)*(g+(last_ddq.^2)*E(2).*sin(last_q)) - last_Fy;
		e0 = [1, 1];
		e = lsqnonlin(Fyfun,e0);
		
		R_ods(t) = e(2);
		M_ods(t) = e(1);
        
        Ifun = @(I) I*last_ddq - last_T;
        I0 = 1;
        ie = lsqnonlin(Ifun, I0);
        I_ods(t) = ie;
    end
     
%     if t > 30
%         R_ods(t) = R_ods(t-1);
%     end;
    
    % algorytm macierzy stanu
    Tf = 1/f;
	if(t > optim_steps_sys)
		B_hat = [0;Tf];
		A_hat = (S(:, t-optim_steps_sys+1:t) - B_hat*U(t-optim_steps_sys+1:t))*pinv(S(:, t-optim_steps_sys:t-1));
		sys_hat = ss(A_hat, sysd.B, sysd.C, sysd.D, Tf);
		sysc_hat = d2c(sys_hat);
		A_hat = sysc_hat.A;
		I1 = -k/A_hat(2,1);
		I2 = -b/A_hat(2, 2);
		I_sys(t) = (I1+I2)/2;
	end;    
end;

t = length(T);
Time = 0:1/f:t/f;
Time = Time(1:end-1);

clf;
fig = figure(1);
plot(Time, [q; dq; ddq], '-');
legend('q', 'dq', 'ddq');
xlabel('t');

orient(fig,'landscape');
print(fig,['img/', name, '_p.pdf'],'-dpdf', '-fillpage');


fig = figure(2);
plot(Time, [FS_x; FS_y; FS_t], '-');
legend('Fx', 'Fy', 'T');
xlabel('t');

orient(fig,'landscape');
print(fig,['img/', name, '_s.pdf'],'-dpdf', '-fillpage');

fig = figure(3);
hold on;
plot(Time, [M_ods; R_ods; I_ods], '-');
plot(Time, m*ones(size(Time)), '--');
plot(Time, r*ones(size(Time)), '--');
plot(Time, I*ones(size(Time)), '--');
hold off;
legend('M\_hat', 'R\_hat', 'I\_hat', 'M', 'R', 'I');
xlabel('t');

orient(fig,'landscape');
print(fig,['img/', name, '_c.pdf'],'-dpdf', '-fillpage');

fig = figure(4);
hold on;
plot(Time, [I_sys; I_fs], '-');
plot(Time, I*ones(size(Time)), '--');
hold off;
legend('I\_hat', 'I\_fs', 'I');
xlabel('t');

orient(fig,'landscape');
print(fig,['img/', name, '_r.pdf'],'-dpdf', '-fillpage');

fig = figure(5);
hold on;
plot(Time, [Us; Akt], '-');
plot(Time, Us_exp, '--');
hold off;
legend('U', 'Aktywacja', 'U\_oczekiwane');
xlabel('t');

orient(fig,'landscape');
print(fig,['img/', name, '_u.pdf'],'-dpdf', '-fillpage');
