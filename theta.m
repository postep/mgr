clear variables;
weight = 1;
a = [0.5, 0.5, 0.5, 1, 1, 4, 3];
X0 = [0, 0, 0, 1]';
j = 0;
k = 0;
for i=1:1000
	qm = rand(1)*2*pi;
	q_1 = [rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, qm];
	q_2 = [rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, rand(1)*2*pi, qm];

	[ Fgn_1, M0_1, X_1, Xn_1, Xm_1, theta_n_1 ] = model(X0, q_1, a, weight);
	[ Fgn_2, M0_2, X_2, Xn_2, Xm_2, theta_n_2 ] = model(X0, q_2, a, weight);


	%% calculate theta
	d_theta = theta_n_2 - theta_n_1;
	theta_n1 = atan(cot(d_theta)-M0_2/(M0_1*sin(d_theta)));
	
	qm_solve1 = mod(theta_n1 - theta_n_1, 2*pi);
	qm_solve2 = mod(qm_solve1 + pi, 2*pi);

	Mmax1 = M0_1/cos(qm_solve1+theta_n_1);
    
	if Mmax1 < 0
		qm_solve = qm_solve1;
    else
        qm_solve = qm_solve2;
    end;
	
    if ~isalmost(qm_solve, qm)
		j = j + 1;
	end;

end
disp(j);

