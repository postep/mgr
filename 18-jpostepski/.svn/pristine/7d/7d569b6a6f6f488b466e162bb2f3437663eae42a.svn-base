clf;
t0 = 0; tf = 100;
q0 = [pi/4, 0]'; % Initial conditions 
[t, q] = ode45(@spring, [t0, tf], q0);

plot(t, q, '-');
legend('q', 'dq');

function dq = two_joints( t, q )
%DX_FUN Summary of this function goes here
%   Detailed explanation goes here

k = 2;
d = 4;
m = 3;
me = 1.5;
g = 9.81;
l = 1;
I = 4*m*l*l/3;

dq = zeros(2, 1);
dq(1) = q(2);
dq(2) = (-k*q(1) -d*q(2) -cos(q(1))*m*g*l/2 +cos(q(1))*me*g*l/2)/I;
end

function dx = spring( t, x )
%DX_FUN Summary of this function goes here
%   Detailed explanation goes here

k = 1;
d = 1;
m = 5;
g = 9.81;

dx = zeros(2, 1);
dx(1) = x(2);
dx(2) = -k*x(1)/m - d*x(2)/m - g;
end


