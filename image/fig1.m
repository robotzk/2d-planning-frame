%% SE(2)机器人的运动规划框架
%***************************************
%Author: zk
%Date: 2022-11-30
%***************************************
clear
clc
close all
addpath(genpath('./'));

% world
Axis_limit = [-1 1 -1 1];
Scale = 5;
w = world_2d('Color','k', 'Size',[-1 1 -1 1], 'Scale_world',5, 'Num_vex',6, 'Num_obs',4, 'Num_obs_c',8, 'Density_obs',1);
w.gen_poly 
zk = w.plot_world;
print world.png -dpng -r600;

a.R = 0.4;
not_start = true;
p_ = plot_agent(0, 0, 0, 0.001);
disp('请分别单击鼠标左键点两次设定位置和方向角');
disp('单击鼠标右键点终止仿真');
while not_start == 1
    if not_start


        [xs, ys, but]=ginput(1);
        delete(p_.Edge);
        delete(p_.Head);
        if but ~= 1
            disp('未设置起始点位置，仿真终止')
            break
        end
        temp = scatter(xs, ys, 'g', 'p', 'fill');
        
        [xt, yt, but]=ginput(1);
        if but ~= 1
            disp('未设置起始方向角，仿真终止')
            break
        end
        
    end
    yaw_s = atan2(yt - ys, xt - xs);
    delete(temp);
    p_ = plot_agent(xs, ys, yaw_s, a.R);

    if check_agent(w, [xs, ys, yaw_s], a.R)
       not_start = false;
    else
        disp('起始状态未通过碰撞检测，请重新选取。')
    end

end
temp = [];
h = [];
H = [];
H1 = [];
while 1
    [xg, yg, but]=ginput(1);
    if but ~= 1
        disp('未设置目标点位置，仿真终止');
        break;
    end
    if ~isempty(temp)
         delete(temp);
    end
    if ~isempty(h)
         delete(h);
         clear h;
    end
    if ~isempty(H)
         delete(H);
         clear H;
    end  
    if ~isempty(H1)
         delete(H1);
         clear H1;
    end     
    temp = scatter(xg, yg, 'g', 'p', 'fill'); 
    [xt, yt, but]=ginput(1);
    if but ~= 1
        disp('未设置目标点方向角，仿真终止')
        break
    end
    yaw_g = atan2(yt - yg, xt - xg);
%     delete(temp);

%     if check_agent(w, [xg, yg, yaw_g], a.R)
%        not_start = false;
%     else
%         disp('目标点未通过碰撞检测，请重新选取。')
%     end
    if check_agent(w, [xg, yg], a.R)
       not_start = false;
    else
        disp('目标区域未通过碰撞检测，请重新选取。')
        h = [];
        H = [];
        H1 = [];
        continue;
    end
    
    RRT = rrt('world',w, 'start',[xs, ys], 'goal',[xg, yg], 'r', a.R, 'segmentLength',a.R, 'is_Informed',true, 'max_Time',0.5);
    RRT.plan;
    if ~RRT.flag
        disp('RRT未能搜索到路径，请重新选取目标点或增加RRT的最大搜索时间')
        h = [];
        H = [];
        H1 = [];
        continue;
    end
    flag = RRT.find_path;
    RRT.simplePath;
    H = RRT.plot_tree;
%     H1 = RRT.plot_path;
    waypoints = RRT.Path_;
    N = 0;
    v0 = [0,0];
    a0 = [0,0];
    v1 = [0,0];
    a1 = [0,0];
    T = 5;
    n_order = 5;
    while 1
        waypts = add_waypoints(waypoints, N)';
        ts = arrangeT(waypts, T);
        polys_x = minimum_snap_single_axis_simple(waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1));
        polys_y = minimum_snap_single_axis_simple(waypts(2,:),ts,n_order,v0(2),a0(2),v1(2),a1(2));
        if ~check_poly(polys_x, polys_y, ts, w, a.R)
            N = N + 1;
            continue;
        else
            H1 = plot(waypts(1, :), waypts(2, :),'*');
            set(H1,'MarkerSize',10,'MarkerEdgeColor','g');
            for i=1:size(polys_x,2)
                tt = linspace(ts(i), ts(i+1));
                xx = polys_vals(polys_x,ts,tt,0);
                yy = polys_vals(polys_y,ts,tt,0);
                h(i) = plot(xx,yy,'Color','r', 'LineWidth',2);
            end
            break;
        end

        

            
    end
        
    
    

end
print zk.png -dpng -r600;
