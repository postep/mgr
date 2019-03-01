clear variables;
global g l m D k I q0;
m = 1;
D = 10;
k = 10;
l = 0.4;
g = 9.80665;
% g=0;
I = 1/3*m*l*l;
Thau = 0;
q0 = 1;


tspan = [0 :1e-3: 15];
y0 = [0; 0];
[t, y] = ode45(@vdp1, tspan, y0, odeset('AbsTol', 1e-9, 'RelTol', 1e-12));

ddq = zeros(size(t));
for i = 1:length(t)
    Thau = l*m*g*cos(y(i, 1))/2;
    ddq(i) = -k*(y(i, 1)-q0)/I -D*y(i, 2)/I -l*m*g*cos(y(i, 1))/I/2 + Thau/I;
end

clear figures;
figure(1);
plot(t, y, '-');
legend('q', 'dq');
grid on;

figure(2);
plot(t, ddq, '-');
legend('ddq');
grid on;


function dx = vdp1(t, x)
    global g l m D k I q0;
    dx = zeros(2, 1);
    dx(1) = x(2);
    Thau = l*m*g*cos(x(1))/2;
    dx(2) = -k*(x(1)-q0)/I -D*x(2)/I -l*m*g*cos(x(1))/I/2 + Thau/I;
end
