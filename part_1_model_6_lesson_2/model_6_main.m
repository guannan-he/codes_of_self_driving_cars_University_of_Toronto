clear;
clc;
%%
x_ref = [0:0.1:60];%正弦参考路径
y_ref = 2 * sin(x_ref / pi);
[~,r] = size(y_ref);
L = 2;%轴距
v = 5;%车速
k = 0.5;%预瞄系数
t = 0.1;%时间间隔
ld = k * v;%预瞄距离
x = 0;
y = 0;
p = 0;
res = zeros(2,3);%储存结果
for i = 1:r
    [k,alpha] = find_pos(x_ref,y_ref,x,y,ld,p);%找预瞄点位置和航向角误差
    delta = atan(2 * L *sin(alpha) / ld);%更新转向角
    [x,y,p] = update_s(x,y,p,delta,v,t,L);%更新状态
    if x > x_ref(end)
        break;%超出路径范围停止
    end
    res(i + 1,:) = [x,y,p];
end
%%
figure(1);
clf;
plot(x_ref,y_ref);
hold on;
plot(res(:,1),res(:,2));
title('pos');
legend('ref','act');
%%
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
