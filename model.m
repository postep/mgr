function [ Fgn, M0, X, Xn, Xm ] = model( X0, q, a, weight )
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
end

