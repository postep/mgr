f = 10;
T = 1/f;
t_end = 10;
t_plot = 0:T:t_end;
steps = length(t_plot);
t_steps = 1:1:steps;
M = 2;
h = 1;
R = 1;
g = -9.81;
I = [(1/12)*M*h*h + 1/4*M*R*R, 0, 0; 
    0, 1/12*M*h*h + 1/4*R*R, 0;
    0, 0, 1/2*M*R*R];

Tg = [0; M*g*R; 0];
k = [1; 1; 10];
d = [10; 10; 10];

q0 = [1; 0.4; -0.6; 0; 0; 0];

AF = diag(M);
BF = eye(3);
CF = [0, 1, 0];
DF = [0, 0, 0];
AT = [zeros(3), eye(3); -inv(I)*diag(k), -inv(I)*diag(d)];
BT = [zeros(3); inv(I)*eye(3)];
CT = [0, 0, 0, 1, 1, 1];
DT = [0, 0, 0];


sysT = ss(AT, BT, CT, DT);
sysdT = c2d(sysT, 1/f);

UT = zeros(3, 1);
ddq =zeros(3, 1);
ST = zeros(6, 1);
ST(1,1) = 1;
ST(2,1) = 1;
ST(3,1) = 1;
SF = zeros(3, 1);

for t = t_steps(2:end)
    %wylicznie sterowania
    UT(:,t) = zeros(3, 1);
	%symulacja urzadzenia
    ST(:, t) = sysdT.A*(S(:, t-1)-q0) + sysdT.B*(U(:,t) + [0;M*g*R*cos(ST(2,t-1));0]);
    ddq(:, t) = (ST(4:6,t)-ST(4:6,t-1))*f;
    %symulacja FTS
    SF(:, t) = M*R*(ddq(:,t).^2) + [0;M*g;0];
end;

clf;
fig = figure(1);
plot(t_plot, ST(1:3,:), '-');
legend('qx', 'qy', 'qz');
xlabel('t');

orient(fig,'landscape');
% print(fig,['img/', name, '_q.pdf'],'-dpdf', '-fillpage');

fig = figure(2);
plot(t_plot, ST(4:6,:), '-');
legend('dqx', 'dqy', 'dqz');
xlabel('t');

orient(fig,'landscape');
% print(fig,['img/', name, '_dq.pdf'],'-dpdf', '-fillpage');

fig = figure(3);
plot(t_plot, SF(), '-');
legend('fx', 'fy', 'fz');
xlabel('t');

orient(fig,'landscape');
% print(fig,['img/', name, '_dq.pdf'],'-dpdf', '-fillpage');
