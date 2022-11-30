classdef rrt < handle
    properties
        world;
        r;
        start_Node;
        start;
        goal;
        tree_Data = [];
        max_Node = 3000;
        max_Time = 1; %  1s
        segmentLength;
        flag = 0;
        is_Informed = 0;
        Path = [];
        Path_ = [];
        
    end
    
    methods
        function obj = rrt(varargin)
                 obj = parse_args(obj,varargin{:});
                 if isempty(obj.start) || isempty(obj.goal) || isempty(obj.segmentLength)
                     error('缺失必要条件');
                 end
                 obj.start_Node(1) = obj.start(1);
                 obj.start_Node(2) = obj.start(2);
                 obj.start_Node(3) = 0;
                 obj.start_Node(4) = 0;
                 obj.start_Node(5) = 1;

        end
        
        function obj = plan(obj)
            obj.tree_Data = [obj.start_Node];
            obj.flag = 0;
            w = obj.world;
            R = obj.r;
            if obj.is_Informed
                c(1) = obj.goal(1) -  obj.start(1);
                c(2) = obj.goal(2) -  obj.start(2);
                center(1) = 0.5 * (obj.goal(1) +  obj.start(1));
                center(2) = 0.5 * (obj.goal(2) +  obj.start(2));
                o = atan2(c(2),c(1));
                rot = [cos(o) -sin(o)
                       sin(o) cos(o)];
            end
            time1 = clock;
       
            while(1)
                time2 = clock;
                runtime = (time2 - time1);
                runtime = 3600 * runtime(4) + 60 * runtime(5) + runtime(6); 
                if obj.max_Time < runtime
                    break;
%                     error('rrt未搜索到路径')
                end
                if obj.is_Informed && obj.flag
                   idx = find(obj.tree_Data(:, 3) == 1);
                   minCost = min(obj.tree_Data(idx, 4) + norm(obj.tree_Data(idx, 1:2) - obj.goal) );
                    a = minCost/2;
                    b = (a^2 - 0.25*norm(c)^2)^0.5;
                end
                
                if ~obj.is_Informed || ~obj.flag
                    randomPoint = sample(w.Size, obj.goal, obj.segmentLength);
                else
                    x_rand_untiball= rand*unit_graph(randn(1,2));
                    randomPoint = (rot * (x_rand_untiball.*[a,b])')' + center;
                end
                
                if length(obj.tree_Data) > obj.max_Node 
                    break
%                     error('超过最大节点数')
                end
%                 children = zeros(obj.max_Node,1);
                tmp = sum((obj.tree_Data(:,1:2) - randomPoint) .* (obj.tree_Data(:,1:2) - randomPoint), 2);
                [d_min, idx] = min(tmp);
                newPoint = randomPoint - obj.tree_Data(idx,1:2);
                newPoint = obj.tree_Data(idx,1:2) + (newPoint / sqrt(d_min) ) * obj.segmentLength;
                
%                 minCost  = obj.tree_Data(idx, 4) + obj.segmentLength;
                if ~collision(newPoint, obj.tree_Data(idx,:), w, R)
                    minCost  = obj.tree_Data(idx, 4) + obj.segmentLength; %fixed segmentLength
                    newNode(1) = newPoint(1);
                    newNode(2) = newPoint(2);
                    newNode(3) = 0;
                    newNode(4) = minCost;
                    newNode(5) = idx;
                    
                    near_idx = near(obj.tree_Data, newNode, 2*obj.segmentLength);%  r should be    r*（log（n）/n）
                    
                    if size(near_idx,1)>1
                    size_near = size(near_idx,1);
                      for i = 1:size_near
                          if ~collision(newPoint, obj.tree_Data(near_idx(i), :), w, R)
                              
                              cost_near = obj.tree_Data(near_idx(i),4) + norm(obj.tree_Data(near_idx(i),1:2) - newNode(1:2));

                              if  cost_near < newNode(4)
                                newNode(4) = cost_near;
                                newNode(5) = near_idx(i);
                              end
                          end
                      end
                    end
                    obj.tree_Data = [obj.tree_Data; newNode];
                    new_node_idx = size(obj.tree_Data,1);
%                     children(newNode(5),1) = children(newNode(5),1) + 1;
                    min_cost = newNode(4);
                % rewire  
                %  
                  if size(near_idx,1)>1
                  queue = zeros(1,66);
                  reduced_idx = near_idx;
                    for j = 1:size(reduced_idx,1)
                      near_cost = obj.tree_Data(reduced_idx(j),4);
                      lcost = norm(obj.tree_Data(reduced_idx(j),1:2) - newNode(1:2));
                        if near_cost > min_cost + lcost ...
                           && ~collision(newNode(1:2), obj.tree_Data(reduced_idx(j), :), w, R)
    %                         children(obj.tree_Data(reduced_idx(j), 5),1) = children(obj.tree_Data(reduced_idx(j),5),1) - 1;
                            obj.tree_Data(reduced_idx(j),5) = new_node_idx;
    %                         children(new_node_idx,1) = children(new_node_idx,1)+1; 
                            d_cost = near_cost - (min_cost + lcost);
    %                         rewire = rewire + 1;
                            bottom = 0;
                            top = 0;
                            bottom = bottom + 1;
                            queue(bottom) = reduced_idx(j);
                            while top < bottom
                                top = top+1;
                                cur = queue(top);
                                obj.tree_Data(cur,4) = obj.tree_Data(cur,4) - d_cost;
                                kids = find(obj.tree_Data(:,5) == cur);
                                for k_ind = 1:numel(kids)
                                    bottom = bottom + 1;
                                    queue(bottom) = kids(k_ind);
                                end
                            end
                        end

                    end
                  end                    
                    if norm(obj.tree_Data(end, 1:2) - obj.goal) < obj.segmentLength
                        
                        obj.flag = 1;
                        obj.tree_Data(end, 3) = 1;
                        
                    end

                    
                end
                
            end
            
            
            
        end
        function p = plot_tree(obj)
            tree = obj.tree_Data;
            dim = 2;
            ind = size(tree,1);
            t = 1;
            while ind>0
            branch = [];
            node = tree(ind,:);
            branch = [ branch ; node ];
            parent_node = node(dim+3);
                while parent_node > 1
                cur_parent = parent_node;
                branch = [branch; tree(parent_node,:)];
                parent_node = tree(parent_node,dim+3);
                end
                branch = [branch;tree(1,:)];
                ind = ind - 1;

                X = branch(:,1);
                Y = branch(:,2);

                p(t) = plot(X,Y);
                t = t + 1;
                set(p,'Color','b','LineWidth',0.15,'Marker','.','MarkerEdgeColor','b');
                hold on;
            end
            pp = plot(0,0.0001);set(pp,'Color','b','LineWidth',1,'Marker','.','MarkerEdgeColor','b');
        end
        

        function p = plot_path(obj)
%             p = plot(obj.Path_(:, 1), obj.Path_(:, 2), 'linewidth',1,'color','r');
            p = plot(obj.Path_(:, 1), obj.Path_(:, 2),'*g');
        end
        
        function flagg = find_path(obj)
        connectingNodes = [];
        flagg = 0;
%         connectingNodes = Zeros(1, length(obj.tree_Data(1,:)));
        for i=1:size(obj.tree_Data, 1)
            if obj.tree_Data(i, 3) == 1
                flagg = 1;
                connectingNodes = [connectingNodes ; obj.tree_Data(i,:)];
            end
        end
        if flagg == 0
            warning('not find path');
            return;
        end
%         [~, idx] = min(connectingNodes(:, 4) + sum((connectingNodes(:, 1:2) - obj.goal) .* (connectingNodes(:, 1:2) - obj.goal), 2));
        [~, idx] = min(connectingNodes(:, 4));
        Path = [connectingNodes(idx, 1:2); obj.goal];
        parent_node = connectingNodes(idx, 5);
        while parent_node > 1
            Path = [obj.tree_Data(parent_node, 1:2); Path];
            parent_node = obj.tree_Data(parent_node, 5);

        end
        Path = [obj.start_Node(1:2);Path];
        clear obj.Path
        obj.Path = Path;
        end
        
        function simplePath(obj)
            w = obj.world;
            R = obj.r;
            if isempty(obj.Path)
                error('未发现原始路径');
            end
            path_ = SimplePath(obj.Path, w, R);
            obj.Path_ = path_;
        end
            
    end
    
end

function randomPoint = sample(world_Size, goal, segmentLength)
if rand < 0.95
    randomPoint(1) = world_Size(1) + rand * (world_Size(2) - world_Size(1));  % rand * (xmax - xmin) + xmin
    randomPoint(2) = world_Size(3) + rand * (world_Size(4) - world_Size(3));
else
    r = rand * 2*pi;
    randomPoint(1) = goal(1) + cos(r)* segmentLength;
    randomPoint(2) = goal(2) + sin(r)* segmentLength;
end
end

function collision_flag = collision(newPoint, oldPoint, w, r)
collision_flag = 0;

tt = 0.5 * r / norm(newPoint(1:2) - oldPoint(1:2));  % 均匀采样

if collision_flag == 0
    %sigma segmentLength / agent.R    
    for sigma = 0:tt:1
    posCheck = sigma * newPoint(1:2) + (1-sigma) * oldPoint(1:2);
      % check each obstacle
        if ~check_agent(w, posCheck, r*0.5) %relax
            collision_flag = 1;
            break;
        end
    end

end
end


function      near_idx = near(tree_Data, newNode, r)
      tmp = sum((tree_Data(:,1:2) - newNode(1:2)) .* (tree_Data(:,1:2) - newNode(1:2)), 2);
%       num  = size(tree,1); 
%       r = gama*sqrt(log(num)./num);
      near_idx = find(tmp <= r^2);
end


function path_simple = SimplePath(path, w, r)
dim = 2;
path_simple =path(1,1:dim);
path = path(:,1:dim);
t = 1;
while true
    p = size(path,1);
     
    tem_idx = p;
    for i = 1:p
        if collision(path_simple(t,:), path(tem_idx,:), w, r) == 0 && norm(path_simple(t,:) - path(tem_idx,:)) ~= 0
            path_simple(end + 1, :) = path(tem_idx, :);
            t = t + 1;
            break
        end
        tem_idx = tem_idx - 1;
    end
    if norm(path_simple(end,:) - path(end,:)) == 0
        break
    end
    path = [path_simple(1:end-1,:); path(tem_idx:end,:)];
end
 
end