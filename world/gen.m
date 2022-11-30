%%% comparison runtime
clear
close
N = 1e3;
x = [];
l = 1;

tic
disp('test1')

for i  = 1:N
%     x = [x,1];
%     x(end + 1) = 1;
gen_(N,1);
% x = p(i);
% x = rand;
end
toc
tic

disp('test2')

for i  = 1:N
%     x = [x,1];
%     x(end + 1) = 1;

gen_d(N,1);
% x = p(i);
% x = rand;
end
toc
tic

disp('test3')

for i  = 1:N
%     x = [x,1];
%     x(end + 1) = 1;

gen_d2(N,1);
% x = p(i);
% x = rand;
end
toc
function dx = gen_(N, l)
% x = [];
% for i = 1:N
%     x = [x;randi(l) + 1];
% end
x = rand(N, 1);
x = sort(x);
x1 = [];
x2 = [];
for i = 2:N - 1
    if rand > 0.5
        x1 = [x1; x(i)];
    else
        x2 = [x2; x(i)];
    end
end
x2 = flipud(x2);
X = [x(1);x1;x(end);x2;x(1)];
dx = X(2:end) - X(1:end-1);
end
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
function X = gen_d2(N, l)
x = l * rand(N, 1);
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