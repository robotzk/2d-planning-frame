function p = add_waypoints(waypoints, N)
dim = size(waypoints, 2);
n = size(waypoints, 1);
t = (waypoints(2:end, :) - waypoints(1:end - 1, :));
l = sum(t.*t, 2);
l = sqrt(l);
tt = round(N/sum(l) * l);
p = [];
for i = 1: n-1
    px = linspace(waypoints(i, 1), waypoints(i + 1, 1), 2 + tt(i));
    py = linspace(waypoints(i, 2), waypoints(i + 1, 2), 2 + tt(i));
    p = [p; [px(1:end - 1)',py(1:end - 1)']];
end
p = [p;waypoints(end,:)];
end
    