function [ Fgn, M0, X, Xn, Xm, theta_n ] = model( X0, q, a, weight )
    T = eye(4);
    X = X0;
    for i=1:size(q, 2)
        T = T*transformation_matrix(q(i), a(i));
        Xnew = T*X(:,1);
        X = [X Xnew];
    end
    Tinv = inv(T);
    
    %%calculate M0
    Xn = X(:,end-1);
    Xm = X(:,end);
        
    R = Xm - Xn;
    Fg = [0; weight * -10; 0; 0];
    Fgn = inv(transformation_matrix(q(1), a(1)))*Fg;
    
    M0 = det([1 1 1; R(1:3)'; Fg(1:3)']);
    
    %%calculate angle of last joint to world frame
    Xn_1 = X(:,end-2);
    dX = Xn-Xn_1;
    dX(4) = 1;
   
    theta_n = mod(atan2(dX(2), dX(1)), 2*pi);
    if theta_n < 0
        theta_n = theta_n + 2*pi;
    end;
end

