function child_nodes = child_nodes_cal(parent_node, m, n, p, obs, closeList)
    % parent_node：当前节点位置 [x, y, z]
    % m, n, p：地图的维度 (m: X轴, n: Y轴, p: Z轴)
    % obs：障碍物列表，每个障碍物是一个坐标 [x, y, z]
    % closeList：已经扩展过的节点列表

    child_nodes = [];
    
    % 定义沿三维空间扩展的所有邻居方向
    directions = [
        -1,  0,  0; % X方向负
         1,  0,  0; % X方向正
         0, -1,  0; % Y方向负
         0,  1,  0; % Y方向正
         0,  0, -1; % Z方向负
         0,  0,  1; % Z方向正
        -1, -1,  0; % X, Y对角线负
        -1,  0, -1; % X, Z对角线负
        -1,  1,  0; % X, Y对角线负
         1,  0, -1; % X, Z对角线正
         1, -1,  0; % X, Y对角线正
         1,  0,  1; % X, Z对角线正
         0, -1, -1; % Y, Z对角线负
         0, -1,  1; % Y, Z对角线正
         0,  1, -1; % Y, Z对角线负
         0,  1,  1; % Y, Z对角线正
        -1, -1, -1; % 三维对角线负
        -1, -1,  1; % 三维对角线负
        -1,  1, -1; % 三维对角线负
        -1,  1,  1; % 三维对角线负
         1, -1, -1; % 三维对角线正
         1, -1,  1; % 三维对角线正
         1,  1, -1; % 三维对角线正
         1,  1,  1  % 三维对角线正
    ];
    
    % 遍历所有方向扩展子节点
    for i = 1:size(directions, 1)
        child_node = parent_node + directions(i, :); % 计算新的子节点
        
        % 检查子节点是否在有效区域内
        if all(child_node(1) >= 1 & child_node(1) <= m) && ...
           all(child_node(2) >= 1 & child_node(2) <= n) && ...
           all(child_node(3) >= 1 & child_node(3) <= p)
            % 检查子节点是否是障碍物
            if ~ismember(child_node, obs, 'rows')
                % 将子节点添加到子节点列表
                child_nodes = [child_nodes; child_node];
            end
        end
    end
    
    % 排除已经在closeList中的节点
    delete_idx = [];
    for i = 1:size(child_nodes, 1)
        if ismember(child_nodes(i,:), closeList, 'rows')
            delete_idx(end+1,:) = i;
        end
    end
    child_nodes(delete_idx, :) = [];
end
