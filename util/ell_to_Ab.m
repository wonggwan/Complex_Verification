function [A,b] = ell_to_Ab(ell)
    [q, Q] = double(ell);
    A = chol(inv(Q));
    b = -inv(A')*inv(Q)*q;
end

