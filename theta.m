clear variables;
weight = 1;
a = [4, 3];
X0 = [0, 0, 0, 1]';
j = 0;
k = 0;
for i=1:1000
	qm = rand(1)*2*pi;
	q_1 = [rand(1)*2*pi, qm];
	q_2 = [rand(1)*2*pi, qm];

	[ Fgn_1, M0_1, X_1, Xn_1, Xm_1 ] = model(X0, q_1, a, weight);
	[ Fgn_2, M0_2, X_2, Xn_2, Xm_2 ] = model(X0, q_2, a, weight);


	%% calculate theta
	d_theta = q_2(1) - q_1(1);
	theta_n1 = atan(cot(d_theta)-M0_2/(M0_1*sin(d_theta)));
	
	qm_solve1 = theta_n1 - q_1(1);
	if qm_solve1 < 0
		qm_solve1 = 2*pi + qm_solve1;
	end
	if qm_solve1 < 0
		qm_solve1 = 2*pi + qm_solve1;
	end
	
	qm_solve2 = qm_solve1 + pi;
	qm_solve3 = qm_solve1 - pi;

	Mmax1 = M0_1/cos(qm_solve1+q_1(1));
	Mmax2 = M0_1/cos(qm_solve2+q_1(1));
	Mmax3 = M0_1/cos(qm_solve3+q_1(1));
	
	qm_solve = 0;
	if Mmax1 < 0
		qm_solve = qm_solve1;
	end;
	
	if Mmax2 <= 0
		if qm_solve2 < 2*pi
			qm_solve = qm_solve2;
		else
			qm_solve = qm_solve3;
		end;
	end
	if abs(qm_solve - qm) > 1e-4
		j = j + 1;
	end;
end 
disp(j);
