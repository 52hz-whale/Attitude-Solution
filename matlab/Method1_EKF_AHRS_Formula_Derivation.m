%% 预测过程，A矩阵的雅可比矩阵
syms q0 q1 q2 q3 w_x w_y w_z wb_x wb_y wb_z T

q_k = [q0;q1;q2;q3;wb_x;wb_y;wb_z];
gyro_x_bias = w_x-wb_x;
gyro_y_bias = w_y-wb_y;
gyro_z_bias = w_z-wb_z;
 
A_11=[1,-(T/2)*gyro_x_bias,-(T/2)*gyro_y_bias,-(T/2)*gyro_z_bias;
        (T/2)*gyro_x_bias,1,(T/2)*gyro_z_bias,-(T/2)*gyro_y_bias;
        (T/2)*gyro_y_bias,-(T/2)*gyro_z_bias,1,(T/2)*gyro_x_bias;
        (T/2)*gyro_z_bias,(T/2)*gyro_y_bias,-(T/2)*gyro_x_bias,1];
    
A_12=[0,0,0;
      0,0,0;
      0,0,0;
      0,0,0];   %4*3
A_21=[0,0,0,0;
      0,0,0,0;
      0,0,0,0]; %3*4
A_22=[1,0,0;
      0,1,0;
      0,0,1];   %3*3
A=[A_11,A_12;A_21,A_22];
q=A*q_k;
q_dif=[diff(q,q0),diff(q,q1),diff(q,q2),diff(q,q3),diff(q,wb_x),diff(q,wb_y),diff(q,wb_z)];

% A =
%  
% [                 1, -(T*(w_x - wb_x))/2, -(T*(w_y - wb_y))/2, -(T*(w_z - wb_z))/2, 0, 0, 0]
% [(T*(w_x - wb_x))/2,                   1,  (T*(w_z - wb_z))/2, -(T*(w_y - wb_y))/2, 0, 0, 0]
% [(T*(w_y - wb_y))/2, -(T*(w_z - wb_z))/2,                   1,  (T*(w_x - wb_x))/2, 0, 0, 0]
% [(T*(w_z - wb_z))/2,  (T*(w_y - wb_y))/2, -(T*(w_x - wb_x))/2,                   1, 0, 0, 0]
% [                 0,                   0,                   0,                   0, 1, 0, 0]
% [                 0,                   0,                   0,                   0, 0, 1, 0]
% [                 0,                   0,                   0,                   0, 0, 0, 1]
%  
%  
% q_dif =
%  
% [                 1, -(T*(w_x - wb_x))/2, -(T*(w_y - wb_y))/2, -(T*(w_z - wb_z))/2,  (T*q1)/2,  (T*q2)/2,  (T*q3)/2]
% [(T*(w_x - wb_x))/2,                   1,  (T*(w_z - wb_z))/2, -(T*(w_y - wb_y))/2, -(T*q0)/2,  (T*q3)/2, -(T*q2)/2]
% [(T*(w_y - wb_y))/2, -(T*(w_z - wb_z))/2,                   1,  (T*(w_x - wb_x))/2, -(T*q3)/2, -(T*q0)/2,  (T*q1)/2]
% [(T*(w_z - wb_z))/2,  (T*(w_y - wb_y))/2, -(T*(w_x - wb_x))/2,                   1,  (T*q2)/2, -(T*q1)/2, -(T*q0)/2]
% [                 0,                   0,                   0,                   0,         1,         0,         0]
% [                 0,                   0,                   0,                   0,         0,         1,         0]
% [                 0,                   0,                   0,                   0,         0,         0,         1]

%% acc重力测量方程
q = [q0;q1;q2;q3];
q_ = [q0;-q1;-q2;-q3];
% 注意，q是 载体->大地 的四元数，我们要转换的大地下的重力
% 所以应该是 q_ * p * q
H1t = quaternProd(quaternProd(q_, [0;0;0;1]), q);

% H1t的jacobi矩阵
% 注意，自定义的四元数乘法是 行向量， 应该转化为列向量
H1t = H1t(2:4).';
H1t_diff =[diff(H1t,q0),diff(H1t,q1) ,diff(H1t,q2), diff(H1t,q3)];
% [-2*q2,  2*q3, -2*q0, 2*q1]
% [ 2*q1,  2*q0,  2*q3, 2*q2]
% [ 2*q0, -2*q1, -2*q2, 2*q3]

%% mag磁力测量方程
syms my_n mz_n mx_b my_b mz_b
quaternProd(quaternProd(q, [0 mx_b my_b mz_b]), q_)
%  [q1*(mx_b*q0 - my_b*q3 + mz_b*q2) - q0*(mx_b*q1 + my_b*q2 + mz_b*q3) + q2*(mx_b*q3 + my_b*q0 - mz_b*q1) + q3*(my_b*q1 - mx_b*q2 + mz_b*q0),
%   q0*(mx_b*q0 - my_b*q3 + mz_b*q2) + q2*(my_b*q1 - mx_b*q2 + mz_b*q0) + q1*(mx_b*q1 + my_b*q2 + mz_b*q3) - q3*(mx_b*q3 + my_b*q0 - mz_b*q1), 
%   q0*(mx_b*q3 + my_b*q0 - mz_b*q1) - q1*(my_b*q1 - mx_b*q2 + mz_b*q0) + q2*(mx_b*q1 + my_b*q2 + mz_b*q3) + q3*(mx_b*q0 - my_b*q3 + mz_b*q2),
%   q0*(my_b*q1 - mx_b*q2 + mz_b*q0) + q1*(mx_b*q3 + my_b*q0 - mz_b*q1) - q2*(mx_b*q0 - my_b*q3 + mz_b*q2) + q3*(mx_b*q1 + my_b*q2 + mz_b*q3)]

% 注意 这个m_n是机体测量值 转化为 大地磁场值， 再经过强行矫正为 北天方向
H2t = quaternProd(quaternProd(q_, [0 0 my_n mz_n]), q);
% 不用看第一项
%     q0*(my_n*q2 + mz_n*q3) - q2*(my_n*q0 + mz_n*q1) + q3*(my_n*q1 - mz_n*q0) - q1*(my_n*q3 - mz_n*q2)
%     q2*(my_n*q1 - mz_n*q0) + q3*(my_n*q0 + mz_n*q1) + q0*(my_n*q3 - mz_n*q2) + q1*(my_n*q2 + mz_n*q3)
%     q0*(my_n*q0 + mz_n*q1) - q1*(my_n*q1 - mz_n*q0) + q2*(my_n*q2 + mz_n*q3) - q3*(my_n*q3 - mz_n*q2)
%     q2*(my_n*q3 - mz_n*q2) - q1*(my_n*q0 + mz_n*q1) - q0*(my_n*q1 - mz_n*q0) + q3*(my_n*q2 + mz_n*q3)
H2t = H2t(2:4).';
H2t_diff =[diff(H2t,q0) diff(H2t,q1) diff(H2t,q2) diff(H2t,q3)];
% [2*my_n*q3 - 2*mz_n*q2,   2*my_n*q2 + 2*mz_n*q3, 2*my_n*q1 - 2*mz_n*q0, 2*my_n*q0 + 2*mz_n*q1]
% [2*my_n*q0 + 2*mz_n*q1,   2*mz_n*q0 - 2*my_n*q1, 2*my_n*q2 + 2*mz_n*q3, 2*mz_n*q2 - 2*my_n*q3]
% [2*mz_n*q0 - 2*my_n*q1, - 2*my_n*q0 - 2*mz_n*q1, 2*my_n*q3 - 2*mz_n*q2, 2*my_n*q2 + 2*mz_n*q3]



%% 下面函数是四元数乘法
% 四元数相乘
function ab = quaternProd(a, b)
 
    ab(1) = a(1).*b(1)-a(2).*b(2)-a(3).*b(3)-a(4).*b(4);
    ab(2) = a(1).*b(2)+a(2).*b(1)+a(3).*b(4)-a(4).*b(3);
    ab(3) = a(1).*b(3)-a(2).*b(4)+a(3).*b(1)+a(4).*b(2);
    ab(4) = a(1).*b(4)+a(2).*b(3)-a(3).*b(2)+a(4).*b(1);
    
end