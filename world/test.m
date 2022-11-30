clear
close
tic
N = 1e3;
x = [];
for i  = 1:N
%     x = [x,1];
%     x(end + 1) = 1;
gen_d(N,1);
% gen_d2(N,1);
% x = p(i);
% x = rand;
end
toc
for i = 1:100
N = 5;
x = gen_d2(N,1);
plot(1:N, x, 'o')
hold on
end
w = world;
w.gen_poly
w.plot_world
D = w.Scale_world;
hold on
for i = 1:100
    x = -D + rand *D*2;
    y = -D + rand *D*2;
    if w.check([x y])
        plot(x,y,'color','r','Marker','o')
    else
        plot(x,y,'color','k','Marker','o')
        pcshow([x,y], [1,0,0],'MarkerSize', 100);
    end
end
axis equal
% plot(x_0,y_0,'o')