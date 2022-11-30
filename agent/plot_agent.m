function p = plot_agent(x, y, yaw, L,color)

% example: plot_agent(0, 0, 0, 'g')
if nargin <= 4
    color='g';
end

% L = 10;
%agent_size
 a = [0.5 0.25
     -0.5 0.25
     -0.5 -0.25 
     0.5 -0.25
     0.5 0.25];
 a = a * L;
xc = [x, y];
A = [cos(yaw)  -sin(yaw)
    sin(yaw)    cos(yaw)];
a_ = (A * (a)')' + xc;

b = zeros(4,2);
b(1,:) = (a_(1,:) + a_(2,:)) / 2;
b(3,:) = (a_(3,:) + a_(4,:)) / 2;
b(4,:) = (a_(4,:) + a_(1,:)) / 2;
t = 0.8;
b(2,:) = t * (b(1,:) + b(3,:)) / 2 + (1 - t) * b(4,:);

p.Head = patch(b(:,1), b(:,2),color);
hold on
p.Edge = plot(a_(:,1), a_(:,2), 'Color',color, 'Linewidth',1);
axis equal
% hold on
R = 0.00001;
theta = linspace(0, 2*pi);
X = R * cos(theta);
Y = R * sin(theta);
X = X + x;
Y = Y + y;
h = fill(X,Y,'y');
% plot(X, Y,'color','r', 'linewidth',2)
set(h,'EdgeColor','y','edgealpha',1,'facealpha',0.3)
end