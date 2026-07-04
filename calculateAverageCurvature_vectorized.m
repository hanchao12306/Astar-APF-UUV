function avg_curvature = calculateAverageCurvature_vectorized(path)
    % 计算三维路径的平均曲率（基于向量叉积）
    % 使用公式：κ = |v1 × v2| / (|v1|·|v2|·|v1+v₂|/2)
    
    n = size(path, 1);
    if n < 3
        avg_curvature = 0;
        return;
    end
    
    curvatures = zeros(n-2, 1);
    
    for i = 2:n-1
        % 计算向量
        v1 = path(i,:) - path(i-1,:);
        v2 = path(i+1,:) - path(i,:);
        
        v1_norm = norm(v1);
        v2_norm = norm(v2);
        v_sum = v1 + v2;
        v_sum_norm = norm(v_sum);
        
        if v1_norm > eps && v2_norm > eps && v_sum_norm > eps
            % 计算叉积模长
            cross_product = cross(v1, v2);
            cross_norm = norm(cross_product);
            
            % 曲率公式
            curvatures(i-1) = 2 * cross_norm / (v1_norm * v2_norm * v_sum_norm);
        else
            curvatures(i-1) = 0;
        end
    end
    
    % 计算平均值
    valid_curvatures = curvatures(curvatures > eps);
    if ~isempty(valid_curvatures)
        avg_curvature = mean(valid_curvatures);
    else
        avg_curvature = 0;
    end
end