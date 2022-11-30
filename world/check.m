function out = check(x, P)
out = false;
for i = 2:size(P, 1)
        
%     t = cross([x - P(i - 1, :) , 0] , [P(i, :) - P(i - 1, :), 0]);
    tt = (x(1) - P(i - 1, 1)) * (P(i, 2) - P(i - 1, 2)) - (x(2) - P(i - 1, 2)) * (P(i, 1) - P(i - 1,1));
%     tt(i-1) = t(3);
    if tt > 0
        out = true;
        
        break
    end
end
end