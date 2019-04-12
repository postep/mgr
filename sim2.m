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



Fy = @(m, r) m*(g + dq(i)^2*r*sin(q(i)));
x0 = 4;
x = lsqnonlin(fun,x0)