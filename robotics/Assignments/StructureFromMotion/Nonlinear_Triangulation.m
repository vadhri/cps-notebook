function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation

N = size(X0, 1);
X = zeros(N,3);

for j = 1:15
    for i =1:N
        X_refine = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:)');   
        X(i,:) = X_refine';
    end
end 

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)

    J = [Jacobian_Triangulation(C1, R1, K, X0);
         Jacobian_Triangulation(C2, R2, K, X0);
         Jacobian_Triangulation(C3, R3, K, X0)];
    
    A1 = K*R1*(X0-C1);
    A2 = K*R2*(X0-C2);
    A3 = K*R3*(X0-C3);

    u1 = A1(1); v1 = A1(2); w1 = A1(3);
    u2 = A2(1); v2 = A2(2); w2 = A2(3);
    u3 = A3(1); v3 = A3(2); w3 = A3(3);
    
    b = [x1, x2, x3]';
    f = [u1/w1, v1/w1, u2/w2, v2/w2, u3/w3, v3/w3]';
    
    delta_X = (J'*J)\J'*(b-f);
    X = X0 + delta_X;
end

function J = Jacobian_Triangulation(C, R, K, X)
    A = K*R*(X-C); 
    
    u = A(1);
    v = A(2);
    w = A(3);
    
    f = K(1);
    p_x = K(1,3);
    p_y = K(2,3);
    
    d_u = [f*R(1,1)+p_x*R(3,1), f*R(1,2)+p_x*R(3,2), f*R(1,3)+p_x*R(3,3)];
    d_v = [f*R(2,1)+p_y*R(3,1), f*R(2,2)+p_y*R(3,2), f*R(2,3)+p_y*R(3,3)];
    
    d_w = R(3,:);
    
    d_f1 = [(w*d_u-u*d_w)/w^2; (w*d_v-v*d_w)/w^2];
    
    J = [d_f1']';

end
