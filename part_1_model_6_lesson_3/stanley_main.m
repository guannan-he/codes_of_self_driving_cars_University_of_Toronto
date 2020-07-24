clear;
clc;
%%
x_ref = [0:0.1:60];%正弦参考路径
y_ref = 2 * sin(x_ref / pi);
[~,r] = size(y_ref);
L = 2;%轴距
v = 2;%车速
t = 0.1;
ub = 25 / 180 * pi;
lb = -ub;
k = 5; %转向角比例系数
ks = 0.5;%弱化因子
x = 0;
y = 0;
p = 0;
for i = 1:r
    [j,e] = find_pos(x_ref,y_ref,x,y);%计算当前横向误差和误差测量位置
    delta = -p + atan((k * e) / (ks + v));%转向角
    if delta > ub
        delta = ub;
    end
    if delta < lb
        delta = lb;
    end
    [x,y,p] = update_s(x,y,p,delta,v,t,L);%更新位置
    if x > x_ref(end)
        break;%超出路径范围停止
    end
    res(i + 1,:) = [x,y,p];
end
% update_s(0,0,0,0,0,0);
%% pos
figure(1);
clf;
plot(res(:,1),res(:,2));
hold on;
plot(x_ref,y_ref);
title('pos');
legend('ref','act');
%% phi
figure(2);
clf;
for i = 1:r-1
    p_ref(i) = atan((y_ref(i + 1) - y_ref(i)) / (x_ref(i + 1) - x_ref(i)));
end
plot(res(:,1),res(:,3));
hold on;
plot(x_ref(2:end),p_ref);
title('phi');
legend('ref','act');
%% 更新状态变量
function [x,y,phi]=update_s(xk,yk,phik,delta,v,t,L)
x = xk + v * cos(phik + delta) * t;
y = yk + v * sin(phik + delta) * t;
phi = phik + v * tan(delta) / L * t;
end
%% 寻找参考路径上点
function [j,e] = find_pos(x,y,xk,yk)
[~,r] = size(x);
for j = 1:r
    if xk < x(j)
        break;
    end
end
e = y(j) - yk;
end