clear variables;
weight = 1;
a = [4, 3];
X0 = [0, 0, 0, 1]';

q_1 = [0, rand(1)*pi/2];
[ Fgn_1, M0_1, X_1, Xn_1, Xm_1 ] = model(X0, q_1, a, weight);

q_2 = [0, rand(1)*pi/2];
[ Fgn_2, M0_2, X_2, Xn_2, Xm_2 ] = model(X0, q_2, a, weight);


%% calculate theta
d_theta = q_2(end) - q_1(end);
theta_n1 = atan(cot(d_theta)-M0_2/(M0_1*sin(d_theta)));
if theta_n1 < 0
    theta_n1 = 2*pi + theta_n1;
end
