classdef world_2d < handle
%convex polygon obstacles.paper:Probability that n random points are in convex position.
    properties
%Default         Dim = 2;
        is_circle = 1;
        Color = 'k';
        Size = [-1,1,-1,1];
        Num_vex = 4;
        Num_obs = 5;
        Num_obs_c = 5;
        Density_obs = 2;%  more small =  larger Density
        Scale_world = 10;
        Polygon_X = [];
        Polygon_Y = [];
        Circle_R = [];
        Circle_center = [];
    end
    methods
        
        function obj = world_2d(varargin)
%             if nargin > 0
%                 obj.Scale_world = Scale ;
%                 
%             end
            obj = parse_args(obj,varargin{:});
            obj.Size = obj.Scale_world * obj.Size;
            
        end
        
        function obj = gen_poly(obj)
            obj.Polygon_X = zeros(obj.Num_obs, obj.Num_vex);
            obj.Polygon_Y = zeros(obj.Num_obs, obj.Num_vex);
            s = (obj.Size(2) - obj.Size(1)) * (obj.Size(4) - obj.Size(3)) / (obj.Density_obs * 2);
            s = s / (obj.Num_obs + obj.Num_obs_c);
            L = sqrt(4*s * tan(pi/obj.Num_vex) / obj.Num_vex );
            r = sqrt(s / pi);
            obj.Circle_R = rand(obj.Num_obs_c, 1) * r / 3 + r / 3;
            
            x_0 = zeros(obj.Num_obs + obj.Num_obs_c, 1);
            y_0 = zeros(obj.Num_obs + obj.Num_obs_c, 1);
            i = 1;
            j = 0;
            while i < obj.Num_obs + obj.Num_obs_c + 1
                j = j + 1;
                if j > 30000
                    error('please run again or increase Density_obs')
                end
                xr = (obj.Size(1) + (obj.Size(2) - obj.Size(1)) * rand )* (1 - 2*r/(obj.Size(2) - obj.Size(1)) );
                yr = (obj.Size(3) + (obj.Size(4) - obj.Size(3)) * rand )* (1 - 2*r/(obj.Size(4) - obj.Size(3)) );
%                 if min(abs(xr - x_0)) > 1*L || min(abs(yr - y_0)) > 1*L
                if min((xr - x_0).^2 + (yr - y_0).^2) > 4*r*r * 0.8    %k = 0.8
                    x_0(i) = xr;
                    y_0(i) = yr;
                    i = i + 1;
                end
            end
            obj.Circle_center = zeros(obj.Num_obs_c, 2);
            obj.Circle_center(:, 1) = x_0(end - obj.Num_obs_c + 1: end);
            obj.Circle_center(:, 2) = y_0(end - obj.Num_obs_c + 1: end);
            

            for i = 1:obj.Num_obs
%                 x = (obj.Num_vex/4) * 2 * gen_Vector(obj.Num_vex);% Heuristic   [-4/N 4/N]
%                 y = (obj.Num_vex/4) * 2 * gen_Vector(obj.Num_vex);
                while (1)
                    x = gen_Vector(obj.Num_vex);
                    y = gen_Vector(obj.Num_vex);
                    if max(x.^2 + y.^2) < 0.3 && min(x.^2 + y.^2) > 0.1
                        break;
                    end
                end
%                 x = 0.5*obj.Num_vex * x;
%                 y = 0.5*obj.Num_vex * y;
                [~, id] = sort(atan2(y, x));
                for j = 1:obj.Num_vex
                    
                    obj.Polygon_X(i,j) = x_0(i) + L * sum(x(id(1:j)));
                    obj.Polygon_Y(i,j) = y_0(i) + L * sum(y(id(1:j)));
                   
                end
                obj.Polygon_X(i,obj.Num_vex + 1) = obj.Polygon_X(i, 1);
                obj.Polygon_Y(i,obj.Num_vex + 1) = obj.Polygon_Y(i, 1);% closely
            end
            
            
            
        end
        function out = check(obj, x)
            out = false;
            for i = 1:obj.Num_obs
                out = false;
                for j = 2:(obj.Num_vex + 1)
                    t = (x(1) - obj.Polygon_X(i, j - 1)) * (obj.Polygon_Y(i, j) - obj.Polygon_Y(i, j - 1)) - (x(2) - obj.Polygon_Y(i, j - 1)) * (obj.Polygon_X(i, j) - obj.Polygon_X(i, j - 1));
                    
                    if t > 0
                        out = true;
                        break
                    end
                end
                if out == false
                    break
                end
            end
            
            if out == true
                for i = 1:obj.Num_obs_c
                    
                    dis2 = (x(1) - obj.Circle_center(i, 1))^2 + (x(2) - obj.Circle_center(i, 2))^2;
                    if dis2 < obj.Circle_R(i)*obj.Circle_R(i)
                        out = false;
                        break
                    end
                end
            end
            

        end

            
        function  fig = plot_world(obj)
%             axis(obj.Size)
            fig = figure;
            hold on
            c = obj.Color;
            for i = 1:obj.Num_obs
                plot(obj.Polygon_X(i, :), obj.Polygon_Y(i, :), c)
                fill(obj.Polygon_X(i, :), obj.Polygon_Y(i, :), c)
                
            end
            N = 100;
            th = 0:2*pi/N:2*pi;
            for i = 1:obj.Num_obs_c
                X = cos(th) * obj.Circle_R(i) + obj.Circle_center(i, 1);
                Y = sin(th) * obj.Circle_R(i) + obj.Circle_center(i, 2);
                fill(X,Y,c);
                alpha(0.5)
            end
            axis(obj.Size)
            axis equal;
            grid on;
            xlabel('X(m)')
            ylabel('Y(m)')
        end

    end
end

function X = gen_Vector(N)
x = 1 * rand(N, 1);
xx = 0.5 - x(2:end-1);
x = sort(x);
x1 = zeros(N, 1);
x1(1) = x(1);
x2 = zeros(N, 1);
x2(end) = x(1);
X = zeros(N, 1);
j = 0;k = 0;
for i = 2:N - 1
    if xx(i - 1) > 0
        j = j + 1;
        x1(1 + j) = x(i);        
    else
        k = k + 1;
        x2(end - k) = x(i);     
    end
end
x1(j + 2) = x(end);
x2(end - k - 1) = x(end);
for i = 1 : (j+1)
    X(i) = x1(i + 1) - x1(i);
end
for i = (j+2) : N
    X(i) = x2(i) - x2(i - 1);
end

end       