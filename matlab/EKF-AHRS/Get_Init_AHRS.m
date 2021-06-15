% 从初始传感器值，直接计算初始姿态
function [pitch0,roll0,yaw0] = Get_Init_AHRS(acc_data,mag_data)
    pitch0 = asin(acc_data(2)/norm(acc_data,2));
    roll0 = atan2(-acc_data(1),acc_data(3));
    
    m_x = mag_data(1)*cos(roll0)+mag_data(3)*sin(roll0);
    m_y = mag_data(1)*sin(pitch0)*sin(roll0)+ mag_data(2)*cos(pitch0)-mag_data(3)*cos(roll0)*sin(pitch0); 
    yaw0 = atan2(m_x,m_y);
end
