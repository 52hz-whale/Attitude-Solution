%% 加速度计获取roll和pitch
syms yaw pitch roll g
% 旋转矩阵R 
R = [
[cos(roll)*cos(yaw) - sin(pitch)*sin(roll)*sin(yaw), -cos(pitch)*sin(yaw), cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw)]
[cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll),  cos(pitch)*cos(yaw), sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch)]
[                             -cos(pitch)*sin(roll),           sin(pitch),                               cos(pitch)*cos(roll)]
];

% [ax; ay; az] = R * [0; 0; g];
% 注意，转置
acc = R.' * [0; 0; g];

% -g*cos(pitch)*sin(roll) = ax
%            g*sin(pitch) = ay
%  g*cos(pitch)*cos(roll) = az

% pitch = asin(ay/g);
% roll = atan2(-ax, az);

%% 磁力计获取yaw
syms mx_b my_b mz_b
m = R * [mx_b; my_b; mz_b];
% 将机体坐标下的测量值只经过roll和pitch旋转到earth水平面
yaw = 0;
subs(m)
% mx_e = 
%                                         mx_b*cos(roll) + mz_b*sin(roll)
% my_e = 
% my_b*cos(pitch) - mz_b*cos(roll)*sin(pitch) + mx_b*sin(pitch)*sin(roll)
% mz_e = 
% my_b*sin(pitch) + mz_b*cos(pitch)*cos(roll) - mx_b*cos(pitch)*sin(roll)

% yaw0 = atan2( my_e, mx_e )
