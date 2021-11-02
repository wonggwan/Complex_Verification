A = [-0.5 0; 0.1 -0.2];
% B = [0.5; 0.9];
B= eye(2);
I = eye(2);
% K1 = [0.4, 0.3];
K1 = [0.3 0.3; 0.2 0.2];

X_final=[];
Y_final=[];
num = 6000;
while num>0
    x0 = [5*(1-2*rand);5*(1-2*rand)];
    resx = zeros(1,101);
    resy = zeros(1,101);
    resx(1) = x0(1);
    resy(1) = x0(2);
    cnt = 2;
    for i = 1:50
        i
        % option 1
        % xe = [2.0; 1.0];
        % u = K1 * x0
        % BK2r = -1 * (A-B*K1-I) * xe;
        % x = A*x0 - B * u + BK2r
        
        % non-applicable (possible option 2)
        % xe = [3.0; 1.0]
        % K2r = -1 * pinv(B) * (A-B*K1-I)*xe;
        % x = A*x0 + B * (-1* K1*x0 + K2r)

        
        % option 3
        % K2 = [3, 3];
        % r = [1; 1];
        % u = -1*K1*x0 + K2*r
        % xe = -1* inv(A-B*K1-I) * B * K2 * r
        % x = A * x0 + B * u
        
        %option 4
        r = [2.0; 1.0];
        K2 = -1 * eye(2) * (A-B*K1-I);
        xe =  -1* inv(A-B*K1-I) * B * K2 * r;
        u =  (-1* K1*x0 + K2 * r);
        x = A*x0 + B * u
        
        resx(cnt) = x(1);
        resy(cnt) = x(2);
        cnt = cnt + 1;
        x0 = x;
        num = num - 1;
        X_final = [X_final; x'];
        %Y_final = [Y_final; u];
        
        Y_final = [Y_final; u']; % for option 4
        if abs(x - xe) < 1e-3
            break
        end
    end
    
    %plot(resx, resy)
    reachability = rank([B A*B]);
    if reachability == 2
        disp("reachable")
    end
end
size(X_final)
X_train_ri = X_final;
y_train_ri = Y_final;
save './output/X_train_ri.mat' X_train_ri
save './output/y_train_ri.mat' y_train_ri


load X_train_ri
load y_train_ri

plot(X_train_ri(:,1),X_train_ri(:,2),'ro','LineWidth',1.5)
grid on
axis equal
