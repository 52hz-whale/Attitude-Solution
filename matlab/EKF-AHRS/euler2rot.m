syms q0 q1 q2 q3 a_x a_y a_z ab_x ab_y ab_z 
 
q_k0 = [q0,q1,q2,q3];
q_k0_conj = [q0,-q1,-q2,-q3];
acce_x_bias = a_x;
acce_y_bias = a_y;
acce_z_bias = a_z;
p = [1,acce_x_bias,acce_y_bias,acce_z_bias];
% quaternProd：四元数乘法
q_k1 = quaternProd(quaternProd(q_k0, p),q_k0_conj);

% 求偏导
q_k1_dif = [diff(q_k1,a_x).',diff(q_k1,a_y).',diff(q_k1,a_z).']

function ab = quaternProd(a, b)
 
    ab(1) = a(1).*b(1)-a(2).*b(2)-a(3).*b(3)-a(4).*b(4);
    ab(2) = a(1).*b(2)+a(2).*b(1)+a(3).*b(4)-a(4).*b(3);
    ab(3) = a(1).*b(3)-a(2).*b(4)+a(3).*b(1)+a(4).*b(2);
    ab(4) = a(1).*b(4)+a(2).*b(3)-a(3).*b(2)+a(4).*b(1);
    
end