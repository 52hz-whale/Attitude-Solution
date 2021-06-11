
%% 欧拉角转四元数
syms pitch  % x
syms roll   % y
syms yaw    % z 
q_x=[cos(pitch/2),1*sin(pitch/2),0,0]; % x轴：[1,0,0] 绕x轴旋转pitch0
q_y=[cos(roll/2),0,1*sin(roll/2),0];   % y轴：[0,1,0] 绕y轴旋转roll0
q_z=[cos(yaw/2),0,0,1*sin(yaw/2)];     % z轴：[0,0,1] 绕z轴旋转yaw0
 
q= quaternProd(quaternProd(q_z, q_x),q_y);

% cos(pitch0/2)*cos(roll0/2)*cos(yaw0/2) - sin(pitch0/2)*sin(roll0/2)*sin(yaw0/2),
% cos(roll0/2)*cos(yaw0/2)*sin(pitch0/2) - cos(pitch0/2)*sin(roll0/2)*sin(yaw0/2), 
% cos(pitch0/2)*cos(yaw0/2)*sin(roll0/2) + cos(roll0/2)*sin(pitch0/2)*sin(yaw0/2), 
% cos(pitch0/2)*cos(roll0/2)*sin(yaw0/2) + cos(yaw0/2)*sin(pitch0/2)*sin(roll0/2)]



%% 欧拉角转旋转矩阵
% 绕x轴旋转的矩阵
Rx = [1 0 0 ;
    0 cos(pitch) -sin(pitch);
    0 sin(pitch) cos(pitch)];
% 绕y轴旋转的矩阵
Ry = [cos(roll) 0 sin(roll);
    0 1 0;
    -sin(roll) 0 cos(roll)];
Rz = [cos(yaw) -sin(yaw) 0;
    sin(yaw) cos(yaw) 0;
    0 0 1];
% 再次强调顺规是 ZXY
R = Rz * Rx * Ry;

%[cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw), -cos(pitch)*sin(yaw), cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw)]
%[cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll),  cos(pitch)*cos(yaw), sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch)]
%[                             -cos(pitch)*sin(roll),           sin(pitch),                               cos(pitch)*cos(roll)]

%% 四元数转旋转矩阵
syms q0 q1 q2 q3 x y z
q = [q0 q1 q2 q3];
q_ = [q0 -q1 -q2 -q3];
p = [0 x y z];
Rp = quaternProd(quaternProd(q, p), q_);
Rp_diff = [diff(Rp, x).' diff(Rp, y).' diff(Rp, z).'];

%[                        0,                         0,                         0]
%[q0^2 + q1^2 - q2^2 - q3^2,         2*q1*q2 - 2*q0*q3,         2*q0*q2 + 2*q1*q3]
%[        2*q0*q3 + 2*q1*q2, q0^2 - q1^2 + q2^2 - q3^2,         2*q2*q3 - 2*q0*q1]
%[        2*q1*q3 - 2*q0*q2,         2*q0*q1 + 2*q2*q3, q0^2 - q1^2 - q2^2 + q3^2]

%% 四元数转欧拉角

% R(3,2)-> sin(pitch) = 2*q0*q1 + 2*q2*q3
% R(3,1)/R(3,3)->  -cos(pitch)*sin(roll) /  cos(pitch)*cos(roll) 
%                 = 2*q1*q3 - 2*q0*q2    /  q0^2 - q1^2 - q2^2 + q3^2
% R(1,2)/R(2,2)-> -cos(pitch)*sin(yaw)   /  cos(pitch)*cos(yaw)
%                 = 2*q1*q2 - 2*q0*q3    /  q0^2 - q1^2 + q2^2 - q3^2
% yaw = atan2(2*q0*q3 - 2*q1*q2, q0^2 - q1^2 + q2^2 - q3^2);
% pitch = asin(2*q0*q1 + 2*q2*q3);
% roll = atan2(2*q0*q2 - 2*q1*q3, q0^2 - q1^2 - q2^2 + q3^2);


%% 下面函数是四元数乘法
% 四元数相乘
function ab = quaternProd(a, b)
 
    ab(1) = a(1).*b(1)-a(2).*b(2)-a(3).*b(3)-a(4).*b(4);
    ab(2) = a(1).*b(2)+a(2).*b(1)+a(3).*b(4)-a(4).*b(3);
    ab(3) = a(1).*b(3)-a(2).*b(4)+a(3).*b(1)+a(4).*b(2);
    ab(4) = a(1).*b(4)+a(2).*b(3)-a(3).*b(2)+a(4).*b(1);
    
end
