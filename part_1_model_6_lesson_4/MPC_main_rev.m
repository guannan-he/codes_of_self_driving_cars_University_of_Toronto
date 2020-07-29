clear;
clc;
clf;
hold on;

%% 仿真参数设置
dt = 0.1;%时间间隔
np = 20;%预测步长
nc = 10;%控制步长
nx = 3;%状态量数目
nu = 2;%控制量数目

%% 参考路径设置
t_ref = [0:dt:10];
[~,r] = size(t_ref);
x_ref = 10 * t_ref;%x路径
y_ref = 2 * sin(2 * t_ref * pi / t_ref(end));%y路径
phi_ref = zeros(1,r);%航向角
k_ref = zeros(1,r);%参考曲率
diff_1 = zeros(1,r);%一阶导数
diff_2 = zeros(1,r);%二阶导数
for i = 1:r - 1
    diff_1(i) = (y_ref(i + 1) - y_ref(i)) / (x_ref(i + 1) - x_ref(i));
end
diff_1(end) = diff_1(end - 1);
for i = 2: r - 1
    diff_2(i) = (y_ref(i + 1) - 2 * y_ref(i) + y_ref(i - 1)) / (0.5 * (x_ref(i + 1) - x_ref(i - 1)))^2;
end
diff_2(end) = diff_2(end - 1);
diff_2(1) = diff_2(2);
phi_ref = atan(diff_1);
for i = 1:r
    k_ref(i) = diff_2(i) / (1 + diff_1(i)^2)^(1.5);
end
state_ref = [x_ref;y_ref;phi_ref;k_ref];%最终参考轨迹
clear x_ref y_ref phi_ref k_ref t_ref diff_1 diff_2;
%% 物理限制设置
u_max = [10;0.5];
u_min = [2;-0.5];
du_max = [0.2;0.2];
du_min = [-0.2;-0.2];

%% 车辆参数及状态设置
l = 2;%轴距
target_v = 5;%期望速度
delta = 0;%当前转向角
v = 5;%当前车身速度
control_d = [0;0];%上一时刻控制偏差
control_act = [target_v;delta];%当前实际控制值
control = [control_act];%储存实际控制指令
x_d = [0;0;0];%当前状态偏差
x_act = [0;0;0];%当前实际状态
x_res = [x_act];%储存实际状态

%% 权重矩阵及观测矩阵生成
Q = eye(nx);%状态变量权重
R = eye(nu);%控制变量权重
rou = 100;%松弛因子
Qt = [Q];
Rt = [R];
%观察矩阵
c = [1,0,0,0,0
     0,1,0,0,0
     0,0,1,0,0];
Ct = [c];
for i = 1:np - 1
    Qt = [Qt,zeros(size(Qt,1),size(Q,2));zeros(size(Q,2),size(Qt,1)),Q];
    Ct = [Ct,zeros(size(Ct,1),size(c,2));zeros(size(c,1),size(Ct,2)),c];
end
for i = 1:nc - 1
    Rt = [Rt,zeros(size(Rt,1),size(R,2));zeros(size(R,2),size(Rt,1)),R];
end
clear c R Q;

%% MPC主循环
figure(1);
clf;
plot(state_ref(1,:),state_ref(2,:));
index = 0;
tic;
while index < r
    plot(x_act(1),x_act(2),'r*');
    hold on;
    % 矩阵生成↓↓↓
    [At,Bt] = matrix_gen(x_act,dt,v,delta,l,Ct,np,nc);
    % 求当前点偏差↓↓↓
    [x_d,lateral_err,index] = find_state_ref_err(state_ref,x_act);
    if abs(lateral_err) >=2
        fprintf('横向偏差过大\n');
        break;
    end
    % 求当前约束↓↓↓
    [A_eqst,b_eqst,A_ieqst,b_ieqst,lb,ub] = get_constrains(u_max,u_min,du_max,du_min,control_act,nc,nu);
    % 求最优控制量偏差↓↓↓
    yita = [x_d;control_d];
    H = [Bt' * Qt * Bt + Rt,zeros(size(Bt,2),1);zeros(1,size(Bt,2)),rou];
    H = H + H';%quadprog程序是求1/2H，故将其变为二倍
    F = [2 * yita' * At' * Qt * Bt,0]';%最后一个对应松弛变量
    U_out = quadprog(H,F,A_ieqst,b_ieqst,A_eqst,b_eqst,lb,ub);
    % 求最优控制量↓↓↓
    delta_des = atan(state_ref(4,index) * l);%该点处期望转向角
    control_act = [target_v;delta_des] + control_d + [U_out(1);U_out(2)];%用得到的控制偏差和上一步的控制偏差修正
    v = control_act(1);
    delta = control_act(2);
    control = [control,control_act];%储存
    control_d = [U_out(1);U_out(2)];%更新当前的控制偏差
    % 更新并储存状态↓↓↓
    x_act = update_state(x_act,control_act,dt,l);
    x_res = [x_res,x_act];%储存
end
t = toc;
clc;
fprintf('用时%f秒，走%d步\n',t,index);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%调用子函数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 线性化矩阵生成（预测模型）
function [At,Bt] = matrix_gen(xt,t,v,delta,l,Ct,np,nc)
theta = xt(3);
%海塞矩阵
Ar = [1,0,-v * sin(theta) * t
     0,1,v * cos(theta) * t
     0,0,1];
Br = [t * cos(theta),0
     t * sin(theta),0
     t * tan(delta) / l,(v * t) / (l * cos(delta)^2)];
%将控制变量包含进来，详见《无人驾驶车辆模型预测控制》第三章
A = [Ar,Br;zeros(size(Br,2),size(Ar,2)),eye(size(Br,2))];
B = [Br;eye(size(Br,2))];
At = [A];
Bt = [B];
temp = [B];
%计算控制序列矩阵
for j = 1:np - 1
    At = [A;At * A];
    temp = [A * temp,B];
    Bt = [Bt,zeros(size(Bt,1),size(B,2));temp];
end
At = Ct * At;
Bt = Ct * Bt(:,1:2 * nc);
end

%% 找参考点及误差
%对平面内离散坐标点找参考
function [state_ref_err,lateral_err,index] = find_state_ref_err(state_ref,current_state)
[~,r] = size(state_ref);
distance = zeros(1,r);
for i = 1:r%找距离最近的点
    distance(i) = (current_state(1) - state_ref(1,i))^2 + (current_state(2) - state_ref(2,i))^2;
end
[val,index] = min(distance);
if state_ref(2,index) > current_state(2)%横向误差上（左）正下（右）负
    lateral_err = -val;
else
    lateral_err = val;
end
state_ref_err = current_state - state_ref(1:3,index);
end

%% 更新车辆实际状态(控制模型)
%只是最简单的更新方式，可以换成其他的
function [x_act] = update_state(x_now,control,t,l)
x_act = zeros(3,1);
x_act(1) = x_now(1) + control(1) * cos(x_now(3)) * t;
x_act(2) = x_now(2) + control(1) * sin(x_now(3)) * t;
x_act(3) = x_now(3) + control(1) * tan(control(2)) / l * t;
end

%% 生成当前点约束
%根据当前实际控制量生成约束
function [A_eqst,b_eqst,A_ieqst,b_ieqst,lb,ub] = get_constrains(u_max,u_min,du_max,du_min,control_act,nc,nu)
A_eqst = [];%等式约束A
b_eqst = [];%等式约束b
A_base = tril(ones(nc));
A_ieqst = [kron(A_base,eye(nu)),zeros(nc * nu,1)];%最后一列是忽略松弛变量
A_ieqst = [A_ieqst;-1 * A_ieqst];%不等式约束A
b_base = ones(nc,1);
Umax = kron(b_base,u_max);
Umin = kron(b_base,u_min);
Ut = kron(b_base,control_act);
b_ieqst = [Umax - Ut;Ut - Umin];%不等式约束b，控制变量差之和的富余量
M = 10;%松弛变量限制
DU_max = kron(b_base,du_max);
DU_min = kron(b_base,du_min);
lb = [DU_min;0];%控制变量差上下界
ub = [DU_max;M];
end
