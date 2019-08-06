sys = zpk([],[-1 -1 -1],1); 
[C_pi,info] = pidtune(sys,'PI')

T_pi = feedback(C_pi*sys, 1);
step(T_pi)