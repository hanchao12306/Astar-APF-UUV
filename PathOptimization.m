function path_opt_new = PathOptimization(path_opt,obs)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
path_opt_new=path_opt(1,:);

flag=1;
index=2;
start_point = path_opt(1,:);
end_point = path_opt(2,:);
collsion_check=parent_target_cal(start_point,end_point,obs);
while flag
   %collsion_check=parent_target_cal(start_point,end_point,obs);
   if collsion_check==1
       path_opt_new(end+1,:)=path_opt(index-1,:); 
       start_point=path_opt(index-1,:);
       index=index+1;
       end_point=path_opt(index,:);
       collsion_check=parent_target_cal(start_point,end_point,obs);
       %index=index+1;
       
   end
   if collsion_check==0 && index~=length(path_opt(:,1))
       index=index+1;
       end_point=path_opt(index,:);
       collsion_check=parent_target_cal(start_point,end_point,obs);
       
   end
   if end_point==path_opt(end,:) 
       if collsion_check==0
           path_opt_new(end+1,:)=end_point;
           flag=0;
       end
       if collsion_check==1
           path_opt_new(end+1,:)=path_opt(index-1,:);
           path_opt_new(end+1,:)=end_point;
           flag=0;
       end
       %path_opt_new(end+1,:)=end_point;
       %flag=0;
   end
   
end

end