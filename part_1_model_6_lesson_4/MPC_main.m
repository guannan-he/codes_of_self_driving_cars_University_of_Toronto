clear;
clc;
clf;
%% ����׼��
dt = 0.1;%ʱ����
t_ref = [0:dt:20];
v0 = 5;
x_ref = v0 * t_ref;%���Ҳο�·��,ʱ�亯��
y_ref = 2 * sin(2 * t_ref * pi / t_ref(end));
np = 20;
[~,r] = size(t_ref);
xt = [0;0;0];
res = [xt];
theta = 0;
Q = eye(3);%״̬����Ȩ��
R = eye(2);%���Ʊ���Ȩ��
Qt = [Q];
Rt = [R];
l = 2;%���
delta = 0;%ת���
v = 5;%��ʼ�ٶ�
control = [delta;v];
for i = 1:np - 1
    Qt = [Qt,zeros(size(Qt,1),size(Q,2));zeros(size(Q,2),size(Qt,1)),Q];
    Rt = [Rt,zeros(size(Rt,1),size(R,2));zeros(size(R,2),size(Rt,1)),R];
end
%% MPC��ѭ��
while xt(1) <= x_ref(end)
%     plot(xt(1),xt(2),'r*');
%     hold on;
    [A,B] = matrix_gen(xt,dt,v,l,delta);
    At = [A];
    Bt = [B];
    temp = [B];
    for j = 1:np - 1
        At = [A;At * A];
        temp = [A * temp,B];
        Bt = [Bt,zeros(size(Bt,1),size(B,2));temp];
    end
    [state_ref,flag] = find_ref_state(x_ref,y_ref,np,xt);
    if flag == 1
        break;
    end
    H = 2 * (Bt' * Qt * Bt + Rt);
    F = (2 * xt' * At' * Qt * Bt - 2 * state_ref' * Qt * Bt);
    u = quadprog(H,F);
    delta = u(1);
    v = u(2);
    control = [control,[delta;v]];
    xt = A * xt + B * [delta;v];
    res = [res,xt];
end
%%
clf;
plot(x_ref,y_ref);
hold on;
plot(res(1,:),res(2,:));
%% ��������
function [A,B] = matrix_gen(xt,t,v,l,delta)
theta = xt(3);
A = [1,0,-v * sin(theta) * t
     0,1,v * cos(theta) * t
     0,0,1];
 B = [0 t * cos(theta)
      0 t * sin(theta)
      (v * t) / (l * cos(delta)^2) t * tan(delta) / l];
end
%% �Ҳο�
function [state_ref,flag] = find_ref_state(x_ref,y_ref,np,xt)
[~,r] = size(x_ref);
flag = 0;
for i = 1:r
    if xt(1) <= x_ref(i)
        break;
    end
end
state_ref = [x_ref(i);y_ref(i);atan((y_ref(i + 1) - y_ref(i)) / x_ref(i + 1) - x_ref(i))];
for j = i + [1:np - 1]
    if j >= r
        flag = 1;
        return;
    end
    temp = [x_ref(j);y_ref(j);atan((y_ref(j + 1) - y_ref(j)) / x_ref(j + 1) - x_ref(j))];
    state_ref = [state_ref;temp];
end
end