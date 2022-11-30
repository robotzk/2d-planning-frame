function  flag = check_poly(polys_x, polys_y, ts, w, r)
flag = 1;
N = 20;
for i = 1: size(polys_x, 2)
    tt = linspace(ts(i), ts(i+1), N);
    x = polys_vals(polys_x,ts,tt, 0);
    y = polys_vals(polys_y,ts,tt, 0);
    dx = polys_vals(polys_x,ts,tt, 1);
    dy = polys_vals(polys_y,ts,tt, 1);
    yaw = atan2(dy, dx);
    for j = 1:N
        
        flag = check_agent(w, [x(j),y(j),yaw(j)], r);
        if ~flag
            break;
        end
    end
    if ~flag
        break;
    end
end
            
end
