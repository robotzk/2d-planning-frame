function X = gen_d(N, l)
x = rand(N, 1);
% xx = 0.5 - x(2:end-1);
x = sort(x);
x1 = zeros(N, 1);
x1(1) = x(1);
x2 = zeros(N, 1);
x2(end) = x(1);
X = zeros(N, 1);
j = 0;k = 0;
for i = 2:N - 1
    if rand > 0.5
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