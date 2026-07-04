clc
clear
close all


%% 修改-地图

% 地图小
m = 30;
n = 30;
start_node = [3, 4];  % 起点
target_node = [26, 27];  % 终点

% 自定义障碍物的位置（手动输入障碍物的坐标）
% 格式：[x1, y1; x2, y2; ...]
% 以下为自定义障碍物的示例位置
obs = [
    5, 5;  % 障碍物1
    9, 7.5;
    7, 8;
    8, 8;
    10, 10;  % 障碍物2
    15, 15;  % 障碍物3
    18, 18;
    20, 10;   % 障碍物4
    25, 25;  % 障碍物5
    round(m / 2), round(n / 2)  % 中心点的障碍物
];
%% 预处理

% 初始化closeList
closeList = start_node;
closeList_path = {start_node,start_node};
closeList_cost = 0;
child_nodes = child_nodes_cal(start_node, m, n,  obs, closeList); %修改-子节点搜索函数 

% 初始化openList
openList = child_nodes;
for i = 1:size(openList,1)
    openList_path{i,1} = openList(i,:);
    openList_path{i,2} = [start_node;openList(i,:)];%从初始点到第i个子节点
end

for i = 1:size(openList, 1)
    g = norm(start_node - openList(i,1:2));%修改-norm求范数，返回最大奇异值；abs求绝对值
    h = abs(target_node(1) - openList(i,1)) + abs(target_node(2) - openList(i,2));

    c_n = compute_weight();
    
    %修改-终点横坐标距离加纵坐标距离
    f = g + h;
    openList_cost(i,:) = [g, h, f];
end


%% 开始搜索
% 从openList开始搜索移动代价最小的节点
[~, min_idx] = min(openList_cost(:,3));%输出openlist_cost表中最小值的位置
parent_node = openList(min_idx,:);%父节点为代价最小节点

%% 进入循环
flag = 1;
while flag   
    
    % 修改-找出父节点的忽略closeList的子节点
    child_nodes = child_nodes_cal(parent_node, m, n,  obs, closeList); 
    
    % 判断这些子节点是否在openList中，若在，则比较更新；没在则追加到openList中
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag,openList_idx] = ismember(child_node, openList, 'rows');%ismember函数表示子节点在open表中则返回1，判断flag,输出此子节点在openlist表中的位置
        g = openList_cost(min_idx, 1) + norm(parent_node - child_node);%修改-按照新父节点计算此子节点的g,h值
        h = abs(child_node(1) - target_node(1)) + abs(child_node(2) - target_node(2)) ;

        c_n = compute_weight();
        f = g+h;
        
        if in_flag   % 若在，比较更新g和f        
            if g < openList_cost(openList_idx,1)
                openList_cost(openList_idx, 1) = g;%将openlist_cost表中第id个位置的第一个数更新为以新父节点计算的g值
                openList_cost(openList_idx, 3) = f;
                openList_path{openList_idx,2} = [openList_path{min_idx,2}; child_node];
            end
        else         % 若不在，追加到openList
            openList(end+1,:) = child_node;
            openList_cost(end+1, :) = [g, h, f];
            openList_path{end+1, 1} = child_node;
            openList_path{end, 2} = [openList_path{min_idx,2}; child_node];
        end
    end
   
    % 从openList移除移动代价最小的节点到 closeList
    closeList(end+1,: ) =  openList(min_idx,:);
    closeList_cost(end+1,1) =   openList_cost(min_idx,3);
    closeList_path(end+1,:) = openList_path(min_idx,:);
    openList(min_idx,:) = [];%openlist表中已跳出的最小值位置设为空
    openList_cost(min_idx,:) = [];
    openList_path(min_idx,:) = [];
 
    % 重新搜索：从openList搜索移动代价最小的节点（重复步骤）
    [~, min_idx] = min(openList_cost(:,3));
    parent_node = openList(min_idx,:);


    % 判断是否搜索到终点
    if parent_node == target_node
        closeList(end+1,: ) =  openList(min_idx,:);
        closeList_cost(end+1,1) =   openList_cost(min_idx,1);
        closeList_path(end+1,:) = openList_path(min_idx,:);
        flag = 0;
    end
end
    
 
 

%% 修改-画路径

path_opt = closeList_path{end,2};
path_opt_new = PathOptimization(path_opt,obs);

path_opt(:,1) = path_opt(:,1)-0.5;
path_opt(:,2) = path_opt(:,2)-0.5;
path_opt(:,3) = path_opt(:,3)-0.5;


%% A*部分结束  APF部分参数

Num_point=0;

longtitude=0;
moving_obs_traj = [];  % 用于存储动态障碍物轨迹


%% 传统APF部分

%% APF部分参数
pp=0;
Num_point_APF=0;
moving_obs = [9,11,0,0];     %移动障碍物初始位置
flag_mo=1;                  %移动障碍物移动方向标志位
D=2;                        %变化步长参数
longtitude_APF=0;


%% A*融合APF部分
moving_obs = [9,11,0,0];     %移动障碍物初始位置
for k=1:length(path_opt_new(:,1))-1
    
    Path=[];
    
    % 画地图

    % 栅格地图的行数、列数定义
    
    if k==1
        P0 = [path_opt_new(k,1),path_opt_new(k,2),0,1];
    else
        P0 = Pi;
    end
    
    Pg = [path_opt_new(k+1,1),path_opt_new(k+1,2),0,0];      % 子目标位置

    obs(:,3)=0;
    obs(:,4)=0;
    
    Pobs=[obs;moving_obs];
    % 记录初始位置
    moving_obs_traj = [moving_obs_traj; moving_obs(1:2)];

    P = [Pobs;Pg];         % 将目标位置和障碍物位置合放在一起

    Eta_att =2;            % 计算引力的增益系数
    Eta_rep_ob =15;        % 计算斥力的增益系数
    Eta_rep_edge = 50;     % 计算边界斥力的增益系数
    Eta_rev_ob=5;         % 计算速度势场斥力的增益系数

    d0 =1;                 % 障碍影响距离
    N = size(P,1);         % 障碍与目标总计个数
    len_step = 0.1;        % 步长
    Num_iter = 1000;        % 最大循环迭代次数

    %% ***************初始化结束，开始主体循环******************
    Pi = P0;               %将车的起始坐标赋给Xi
    i = 0;

    while sqrt((Pi(1)-P(N,1))^2+(Pi(2)-P(N,2))^2) > 0.9
        i = i + 1;
        Num_point=Num_point+1;
        step=step_cal(Pobs,len_step,D,Pi);

        for I = 1:size(Pobs,1)-1
            temp = Pobs(I,:);
        end
        if k~=1 || i~=1

            pp=1;
        end

        [flag_mo,moving_obs] = moving_obs_cal(flag_mo,moving_obs);  %计算移动障碍物位置
        P(I+1,:)=moving_obs;
    % 记录动态障碍物轨迹
    moving_obs_traj = [moving_obs_traj; moving_obs(1:2)];
       %Path_Pobs_dong(i,1:2)=Pobs(11,1:2);   
        %计算车辆当前位置与障碍物的单位方向向量、速度向量
        for j = 1:N-1    
            delta(j,:) = Pi(1,1:2) - P(j,1:2);  % 用车辆点-障碍点表达斥力
            dist(j,1) = norm(delta(j,:));       % 车辆当前位置与障碍物的距离
            unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % 斥力的单位方向向量

            delta_vector(j,:) = Pi(1,3:4) - Pobs(j,3:4); %车辆和障碍物的相对速度坐标表示法
            delta_vector_norm(j,1)=norm(delta_vector(j,:)); %相对速度的模长
            unit_Vector(j,:) = [delta_vector(j,1)/delta_vector_norm(j,1), delta_vector(j,2)/delta_vector_norm(j,1)]; % 斥力的单位方向向量

        end

        %计算车辆当前位置与目标的单位方向向量、速度向量
        delta(N,:) = P(N,1:2)-Pi(1,1:2);                            %用目标点-车辆点表达引力   
        dist(N,1) = norm(delta(N,:)); 
        unitVector(N,:)=[delta(N,1)/dist(N,1),delta(N,2)/dist(N,1)];

       %% 计算斥力 
        % 在原斥力势场函数增加目标调节因子（即车辆至目标距离），以使车辆到达目标点后斥力也为0
        for j = 1:N-1
            if dist(j,1) >= d0
                F_rep_ob(j,:) = [0,0];
            else
                % 障碍物的斥力1，方向由障碍物指向车辆
                F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(N,1) / dist(j,1)^2;    %注：其中n为任意常数，经过多次仿真实验所定。和ppt上第一个斥力公式不一样，没有n次方，只有一次。     
                F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];   

                % 障碍物的斥力2，方向由车辆指向目标点
                F_rep_ob2_abs = 0.5 * Eta_rep_ob * (1/dist(j,1) - 1/d0)^2;           %注：其中n为任意常数，经过多次仿真实验所定。和ppt上第二个斥力公式不一样，没有n-1次方，和原始人工势场法斥力公式一样。
                F_rep_ob2 = [F_rep_ob2_abs * unitVector(N,1), F_rep_ob2_abs * unitVector(N,2)];  

                % 改进后的障碍物合斥力计算
                F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;

            end
        end

        %%计算速度势场的斥力
        if dist(j,1)<= d0 && dist(j,1)>=0 
           F_rev_ob_abs1= Eta_rev_ob*delta_vector_norm(j,1);
           F_rev_ob1(j,:)=[ F_rev_ob_abs1*unit_Vector(j,1),  F_rev_ob_abs1*unit_Vector(j,2)];
        else  F_rev_ob1(j,:)=[0,0];
        end

        %% 计算合力和方向
        F_rep = [sum(F_rep_ob(:,1))  + F_rev_ob1(:,1),...
               sum(F_rep_ob(:,2))  + F_rev_ob1(:,2)];                                      % 所有障碍物的合斥力矢量
        F_att = [Eta_att*dist(N,1)*unitVector(N,1), Eta_att*dist(N,1)*unitVector(N,2)];    % 引力矢量
        F_sum = [F_rep(1,1)+F_att(1,1),F_rep(1,2)+F_att(1,2)];                             % 总合力矢量
        UnitVec_Fsum(i,:) = 1/norm(F_sum) * F_sum;                                         % 总合力的单位向量

        %计算车的下一步位置
        Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);  
        %计算动态障碍物1下一步的位置，改动态障碍物这里需要更改
        %Pobs(11,1:2)=Pobs(11,1:2)+len_step*Pobs(11,3:4);

        %plot(Pi(1)-0.5,Pi(2)-0.5, 'ro');

                % 记录路径点
        Path(i,:) = Pi;
        Path_all(Num_point,:) = Pi;

       %判断是否到达终点
        if sqrt((Pi(1)-P(N,1))^2+(Pi(2)-P(N,2))^2) <0.9
            break
        end
    end

    %Path(i,:)=P(N,:);        %把路径向量的最后一个点赋值为目标
    pp=0;

    Path_all(Num_point+1,:)=P(N,:);

    %% 计算路径长度
    segment_length=0;
    for i = 1:size(Path,1)-1
        segment_length=segment_length+norm(Path(i+1,1:2)-Path(i,1:2));
    end
    
    longtitude=longtitude+segment_length;


end
 disp(['融合APF算法路径长度: ', num2str(longtitude)]);  

 % 记录结束时间并计算规划时间
end_time = toc(start_time);

end_time1=end_time-end_time_IAPF;
disp(['融合APF算法规划时间: ', num2str(end_time1), '秒']);
%% 二维路径绘制
figure('Name', 'A-APF 二维路径规划结果', 'Position', [80, 100, 600, 600]);

% 设置二维视图
view(2);
grid on;
hold on;
axis equal;

% 设置坐标轴范围（根据实际数据调整，此处示例假设n,m为地图尺寸）
xlim([0, n]);
ylim([0, m]);
xlabel('X轴');
ylabel('Y轴');

% 绘制起点
plot(start_node(1), start_node(2), 'g*', 'MarkerSize', 10, 'DisplayName', '起点');
% 绘制终点
plot(target_node(1), target_node(2), 'r*', 'MarkerSize', 10, 'DisplayName', '终点');

% 绘制静态障碍物（圆形，半径0.5）
for i = 1:size(Pobs,1)
    center = Pobs(i,:);
    radius = 0.5;
    theta = linspace(0, 2*pi, 50);
    x_circle = center(1) + radius * cos(theta);
    y_circle = center(2) + radius * sin(theta);
    fill(x_circle, y_circle, 'k', 'FaceAlpha', 0.7, 'EdgeColor', 'none', 'DisplayName', '静态障碍物');
end

% 绘制动态障碍物（当前位置，圆形，半径0.5，青色透明）
center_x = moving_obs(1) - 0.5;  % 转换为单元格中心（若原坐标需要调整，可保留原意）
center_y = moving_obs(2) - 0.5;
radius = 0.5;
theta = linspace(0, 2*pi, 50);
x_circle = center_x + radius * cos(theta);
y_circle = center_y + radius * sin(theta);
fill(x_circle, y_circle, 'c', 'FaceAlpha', 0.8, 'EdgeColor', 'none', 'DisplayName', '动态障碍物');

% 绘制改进APF路径（红色点划线）
plot(Path_all_IAPF(:,1), Path_all_IAPF(:,2), 'r-.', 'LineWidth', 2, 'DisplayName', 'Improved APF');
% 绘制传统APF路径（绿色虚线）
plot(Path_all_APF(:,1), Path_all_APF(:,2), 'g--', 'LineWidth', 2, 'DisplayName', 'Traditional APF');

% 绘制动态障碍物的运动轨迹（黑色点划线）
if size(moving_obs_traj,1) > 1
    plot(moving_obs_traj(:,1)-0.5, moving_obs_traj(:,2)-0.5, 'k-.', 'LineWidth', 1, ...
         'DisplayName', '障碍物运动轨迹');
end

% 图例显示
legend('Location', 'best');
title('A-APF 二维路径规划结果');
hold off;



% B样条
% 基于曲率变化提取关键点 
k = 3; % 曲率阈值

controlPoints = Path_all(1,:);  % 初始化控制点

for i = 2:length(Path_all)-1
    % 计算当前路径点的前后点的向量
    v1 = Path_all(i,:) - Path_all(i-1,:);
    v2 = Path_all(i+1,:) - Path_all(i,:);
    
    % 计算前后点的夹角，即曲率突变点
    angle = acos(dot(v1,v2) / (norm(v1) * norm(v2)));
    
    if angle > deg2rad(k) % 曲率突变点
        point = Path_all(i,:);  % 当前控制点
        
        for j = 1:size(obs, 1)
            dist = norm(point - obs(j,:)); 
        end
        % 添加控制点
        controlPoints = [controlPoints; point];
    end
end

% 添加路径的终点
controlPoints = [controlPoints; Path_all(end,:)];

C = BSpline(controlPoints,3,'b-',2);  % 注意：需要修改BSpline函数使其返回C

avg_curvature = calculateAverageCurvature_vectorized(C);
disp(['融合APF算法平滑路径平均曲率: ', num2str(avg_curvature)]);
% B样条曲线的类型
function [C]=BSpline(varargin)
    narginchk(1,7);
    ctrls    = varargin{1};
    k        = varargin{2};
    line_type= varargin{3};
    lw       = varargin{4};
    n = size(ctrls,1);
    weights = ones(n,1) / n;
    NodeVector = linspace(0, 1, n+k); % 均匀B样条的节点矢量，n个控制点，k阶B样条，n+k个节点
    C = BSpline_gen(ctrls, n, k, NodeVector, 0.001, weights);
plot(C(:,1), C(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Fusion algorithm');
end
 
 
function [C]=BSpline_gen(ctrls,n,k,NodeVector,accuracy,weights)
    Bik_w = zeros(1,n);
    num = 1 / accuracy;
    C = zeros(num+1, 3);
    idx = 1;
    for u = 0 : accuracy : num * accuracy
        Dom = 0;
        for i = 1:n
            Bik_w(1,i) = BaseFunction(i, k, u, NodeVector) * weights(i,1); % 不同点的权重
            Dom = Dom + Bik_w(1,i);
        end        
        C(idx,:) = (Bik_w / Dom) * ctrls(:,1:3);
        idx = idx + 1;
    end
end
 
% 计算基函数Bik_u,NodeVector为节点向量
function Bik_u = BaseFunction(i, k , u, NodeVector)
    if k == 1       % 1阶0次B样条
        if NodeVector(i) <= u && u < NodeVector(i+1)
            Bik_u = 1.0;
        else
            Bik_u = 0.0;
        end
    else
        % 支撑区间的长度
        Length1 = NodeVector(i+k-1) - NodeVector(i);
        Length2 = NodeVector(i+k) - NodeVector(i+1);      
        
         % 规定0/0 = 0
        if Length1 == 0.0      
            Length1 = 1.0;
        end
        if Length2 == 0.0
            Length2 = 1.0;
        end
        
        %递归计算
        Bik_u =   (u - NodeVector(i)) / Length1 * BaseFunction(i,   k-1, u, NodeVector) ...
              + (NodeVector(i+k) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
    end
end

    p3=plot(C(:,1),C(:,2),'b-', 'LineWidth', 2,'DisplayName', 'Fusion algorithm'); 
    % 2. 只为 p1 和 p2 设置图例
legend([p1, p2,p3,p4], 'Location', 'best');

%% 提取第2、3、4列数据
col2 = closeList_cost(:, 2);  % 第2列角度数据
col3 = closeList_cost(:, 3);  % 第3列角度数据
col4 = closeList_cost(:, 4);  % 第4列角度数据

%% 生成行索引（x轴）
x = 1:size(closeList_cost, 1);  % 行号从1到21

%%  绘制折线图
figure('Color','w');  % 新建白色背景图形窗口
plot(x, col2, 'r-', 'LineWidth', 1.2);  % 第2列：红色实线
hold on;  % 保持绘图，叠加后续曲线
plot(x, col3, 'g--', 'LineWidth', 1.2); % 第3列：绿色虚线
plot(x, col4, 'b-.', 'LineWidth', 1.2); % 第4列：蓝色点划线
hold off;

%% 美化图形（标签、标题、图例、网格）
xlabel('Node', 'FontSize', 12);
ylabel('Angle(°)', 'FontSize', 12);
title('Attitude disturbance Angle', 'FontSize', 14, 'FontWeight','bold');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');  % 自动选最优位置放图例
grid on;  % 显示网格线

