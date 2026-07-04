function pt_flag = parent_target_cal(parent_node,target_node,obs)
%PARENT_TARGET_CAL 此处显示有关此函数的摘要
%   此处显示详细说明
pt_flag=0;

for index=1:length(obs(:,1))
    pt_dis=sqrt((parent_node(1)-target_node(1))^2+(parent_node(2)-target_node(2))^2);
    op_dis=sqrt((parent_node(1)-obs(index,1))^2+(parent_node(2)-obs(index,2))^2);
    ot_dis=sqrt((obs(index,1)-target_node(1))^2+(obs(index,2)-target_node(2))^2);
    
    angle_opt=acos((pt_dis^2+op_dis^2-ot_dis^2)/(2*pt_dis*op_dis))*180/pi;
    angle_otp=acos((pt_dis^2+ot_dis^2-op_dis^2)/(2*pt_dis*ot_dis))*180/pi;
    angle=max(abs(angle_opt),abs(angle_otp));
    
    if angle<90
        L= (pt_dis+op_dis+ot_dis)/2;
        S=sqrt(L*(L-pt_dis)*(L-op_dis)*(L-ot_dis));
        line_dis=2*S/pt_dis;
    else
        line_dis=min(op_dis,ot_dis);
    end
    k(index)=line_dis;

    if line_dis<(sqrt(2)/2)*0.99
        pt_flag=1;
    end

end

end

