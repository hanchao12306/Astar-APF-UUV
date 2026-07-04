function c_n = compute_weight()

function roll_angle = generate_roll_pitch_angle()
    roll_angle = -10 + 20 * rand(); % 生成 -10° 到 10° 的随机角度
end

% 在 -45° 到 45° 范围内生成随机角度
function yaw_angle = generate_yaw_angle()
    yaw_angle = -45 + 90 * rand(); 
end
% 调用示例
phi=generate_roll_pitch_angle(); % 生成横滚角或俯仰角
theta=generate_roll_pitch_angle(); % 生成俯仰角
varphi=generate_yaw_angle(); % 生成偏航角

    % 计算 m_phi, m_theta, m_varphi
    m_phi = abs(phi / 180);
    m_theta = abs(theta / 90);
    m_varphi = abs(varphi / 180);
    
    % 计算 m
    m = m_phi + m_theta + m_varphi;
    
    % 计算权重函数 c(n)
    c_n = 1+ exp(-m);

end