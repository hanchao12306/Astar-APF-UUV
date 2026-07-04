function [flag_mo,moving_obs] = moving_obs_cal(flag_mo,moving_obs)
%MOVING_OBS_CAL 此处显示有关此函数的摘要
%   此处显示详细说明
        if flag_mo==1
            moving_obs(2)=moving_obs(2)-0.008;
            %P(I+1,1)=moving_obs(1);
        end
        if flag_mo==2
            moving_obs(1)=moving_obs(1)+0.008;
            %P(I+1,1)=moving_obs(1);
        end
        if moving_obs(1)<15
            flag_mo=2;
        end
        if moving_obs(1)>28
            flag_mo=1;
        end
end

