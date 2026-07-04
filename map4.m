%% 修改-地图

% 栅格地图的行数、列数定义
m = 30;
n = 30;
p = 30;
start_node = [3, 4, 5];%起点
target_node = [23, 25,21];%终点

grid_size = 1; % 网格大小（单位：每个格子的边长，确保网格规范）

% 创建三维网格
[x, y, z] = meshgrid(0:grid_size:m, 0:grid_size:n, 0:grid_size:p);

% 自定义障碍物的位置（手动输入障碍物的坐标）
% 格式：[x1, y1, z1; x2, y2, z2; ...]
% 以下为自定义障碍物的示例位置
obs = [
    5, 5, 5; % 障碍物1
    6.5,7.5,9.5;
    7,8,9;
    8,8,8;
    10, 10, 10; % 障碍物2
    15, 15, 15; % 障碍物3
    18,18,20;
    20, 10, 5;  % 障碍物4
    25, 25, 25; % 障碍物5
    round(m / 2), round(n / 2), round(p / 2) % 中心点的障碍物
];

% 绘制障碍物
scatter3(obs(:,1), obs(:,2), obs(:,3), 100, 'r', 'filled'); % 红色点表示障碍物

% 设置标题和标签
title('3D Map with Obstacles');
xlabel('X');
ylabel('Y');
zlabel('Z');

% 设置视角
view(3); % 设置为三维视角
axis([0 m 0 n 0 p]); % 设置坐标轴范围
axis equal; % 确保轴的比例相同，网格大小规范

% 显示网格
grid on;