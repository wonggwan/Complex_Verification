for k = 1:1
    k
    x = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]
    [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin, xmax,x, Xnmpc, P);
    if feas == 1
        for j = 1:30
            u = [tan(uOpt(1,j)); tan(uOpt(2,j)); uOpt(3,j)];
            xNext = one_step_sim(x,u,A,B,E,g)
            x = xNext;
        end  
    end
end