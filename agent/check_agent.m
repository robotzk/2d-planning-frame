function flag = check_agent(w, x, r)
% check agent  w is world   x is state(2*1 or 3*1)   r is agent_size
flag = true;
N = 20;
if length(x) == 2
    full = true;
else
    full = false;
end
pos = zeros(N, 2);

% full is  circle check
if full == true
    
    r = r * sqrt(5) * 0.5;
    theta = linspace(0, 2*pi, N);
    pos(:, 1) = r * cos(theta) + x(1);
    pos(:, 2) = r * sin(theta) + x(2);

    for i = 1:N
        if ~ w.check(pos(i, :))
            flag = false;
            break;
        end
    end
end


% ~full is polygon check
if full ~= true
    a = [0.5 0.25
     -0.5 0.25
     -0.5 -0.25 
     0.5 -0.25
     0.5 0.25];
    a = a * r;
    
    xc = [x(1), x(2)];
    yaw = x(3);
    A = [cos(yaw)  -sin(yaw)
        sin(yaw)    cos(yaw)];
    a_ = (A * (a)')' + xc;
    
    temp = a_(2:end, :) - a_(1:end-1, :);
    temp = sqrt(sum(temp .* temp, 2));
    NN = round(N * temp / sum(temp)); %for every polygon's edge
    
    for i = 1:length(NN)
        for j = 1:NN(i)
            t = (j/NN(i)) * a_(i, :) + (1 - j/NN(i)) * a_(i+1, :);
            if ~w.check(t)
                flag = false;
                break;
            end
        end
        if ~flag
            break;
        end
    end
end
                
        
        
end