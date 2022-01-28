for k = 1:1
    k
    x = [5*(1-2*rand); 5*(1-2*rand); 5*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand); 1*(1-2*rand)]
    for j = 1:30
        [feas, xOpt, uOpt, JOpt] = solve_nmpc_quad(ts,Q,R,N,umin,umax,xmin, xmax,x, Xnmpc, P);
        if feas == 1
            u = [tan(uOpt(1,1)); tan(uOpt(2,1)); uOpt(3,1)];
            xNext = one_step_sim(x,u,A,B,E,g)
            x = xNext;
        end  
    end
end