clear;
clc;
clf;
hold on;

%% �����������
dt = 0.1;%ʱ����
np = 20;%Ԥ�ⲽ��
nc = 10;%���Ʋ���
nx = 3;%״̬����Ŀ
nu = 2;%��������Ŀ

%% �ο�·������
t_ref = [0:dt:10];
[~,r] = size(t_ref);
x_ref = 10 * t_ref;%x·��
y_ref = 2 * sin(2 * t_ref * pi / t_ref(end));%y·��
phi_ref = zeros(1,r);%�����
k_ref = zeros(1,r);%�ο�����
diff_1 = zeros(1,r);%һ�׵���
diff_2 = zeros(1,r);%���׵���
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
state_ref = [x_ref;y_ref;phi_ref;k_ref];%���ղο��켣
clear x_ref y_ref phi_ref k_ref t_ref diff_1 diff_2;
%% ������������
u_max = [10;0.5];
u_min = [2;-0.5];
du_max = [0.2;0.2];
du_min = [-0.2;-0.2];

%% ����������״̬����
l = 2;%���
target_v = 5;%�����ٶ�
delta = 0;%��ǰת���
v = 5;%��ǰ�����ٶ�
control_d = [0;0];%��һʱ�̿���ƫ��
control_act = [target_v;delta];%��ǰʵ�ʿ���ֵ
control = [control_act];%����ʵ�ʿ���ָ��
x_d = [0;0;0];%��ǰ״̬ƫ��
x_act = [0;0;0];%��ǰʵ��״̬
x_res = [x_act];%����ʵ��״̬

%% Ȩ�ؾ��󼰹۲��������
Q = eye(nx);%״̬����Ȩ��
R = eye(nu);%���Ʊ���Ȩ��
rou = 100;%�ɳ�����
Qt = [Q];
Rt = [R];
%�۲����
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

%% MPC��ѭ��
figure(1);
clf;
plot(state_ref(1,:),state_ref(2,:));
index = 0;
tic;
while index < r
    plot(x_act(1),x_act(2),'r*');
    hold on;
    % �������ɡ�����
    [At,Bt] = matrix_gen(x_act,dt,v,delta,l,Ct,np,nc);
    % ��ǰ��ƫ�������
    [x_d,lateral_err,index] = find_state_ref_err(state_ref,x_act);
    if abs(lateral_err) >=2
        fprintf('����ƫ�����\n');
        break;
    end
    % ��ǰԼ��������
    [A_eqst,b_eqst,A_ieqst,b_ieqst,lb,ub] = get_constrains(u_max,u_min,du_max,du_min,control_act,nc,nu);
    % �����ſ�����ƫ�������
    yita = [x_d;control_d];
    H = [Bt' * Qt * Bt + Rt,zeros(size(Bt,2),1);zeros(1,size(Bt,2)),rou];
    H = H + H';%quadprog��������1/2H���ʽ����Ϊ����
    F = [2 * yita' * At' * Qt * Bt,0]';%���һ����Ӧ�ɳڱ���
    U_out = quadprog(H,F,A_ieqst,b_ieqst,A_eqst,b_eqst,lb,ub);
    % �����ſ�����������
    delta_des = atan(state_ref(4,index) * l);%�õ㴦����ת���
    control_act = [target_v;delta_des] + control_d + [U_out(1);U_out(2)];%�õõ��Ŀ���ƫ�����һ���Ŀ���ƫ������
    control = [control,control_act];%����
    control_d = [U_out(1);U_out(2)];%���µ�ǰ�Ŀ���ƫ��
    % ���²�����״̬������
    x_act = update_state(x_act,control_act,dt,l);
    x_res = [x_res,x_act];%����
end
t = toc;
clc;
fprintf('��ʱ%f�룬��%d��\n',t,index);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%�����Ӻ���%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ���Ի��������ɣ�Ԥ��ģ�ͣ�
function [At,Bt] = matrix_gen(xt,t,v,delta,l,Ct,np,nc)
theta = xt(3);
%��������
Ar = [1,0,-v * sin(theta) * t
     0,1,v * cos(theta) * t
     0,0,1];
Br = [t * cos(theta),0
     t * sin(theta),0
     t * tan(delta) / l,(v * t) / (l * cos(delta)^2)];
%�����Ʊ���������������������˼�ʻ����ģ��Ԥ����ơ�������
A = [Ar,Br;zeros(size(Br,2),size(Ar,2)),eye(size(Br,2))];
B = [Br;eye(size(Br,2))];
At = [A];
Bt = [B];
temp = [B];
%����������о���
for j = 1:np - 1
    At = [A;At * A];
    temp = [A * temp,B];
    Bt = [Bt,zeros(size(Bt,1),size(B,2));temp];
end
At = Ct * At;
Bt = Ct * Bt(:,1:2 * nc);
end

%% �Ҳο��㼰���
%��ƽ������ɢ������Ҳο�
function [state_ref_err,lateral_err,index] = find_state_ref_err(state_ref,current_state)
[~,r] = size(state_ref);
distance = zeros(1,r);
for i = 1:r%�Ҿ�������ĵ�
    distance(i) = (current_state(1) - state_ref(1,i))^2 + (current_state(2) - state_ref(2,i))^2;
end
[val,index] = min(distance);
if state_ref(2,index) > current_state(2)%��������ϣ������£��ң���
    lateral_err = -val;
else
    lateral_err = val;
end
state_ref_err = current_state - state_ref(1:3,index);
end

%% ���³���ʵ��״̬(����ģ��)
%ֻ����򵥵ĸ��·�ʽ�����Ի���������
function [x_act] = update_state(x_now,control,t,l)
x_act = zeros(3,1);
x_act(1) = x_now(1) + control(1) * cos(x_now(3)) * t;
x_act(2) = x_now(2) + control(1) * sin(x_now(3)) * t;
x_act(3) = x_now(3) + control(1) * tan(control(2)) / l * t;
end

%% ���ɵ�ǰ��Լ��
%���ݵ�ǰʵ�ʿ���������Լ��
function [A_eqst,b_eqst,A_ieqst,b_ieqst,lb,ub] = get_constrains(u_max,u_min,du_max,du_min,control_act,nc,nu)
A_eqst = [];%��ʽԼ��A
b_eqst = [];%��ʽԼ��b
A_base = tril(ones(nc));
A_ieqst = [kron(A_base,eye(nu)),zeros(nc * nu,1)];%���һ���Ǻ����ɳڱ���
A_ieqst = [A_ieqst;-1 * A_ieqst];%����ʽԼ��A
b_base = ones(nc,1);
Umax = kron(b_base,u_max);
Umin = kron(b_base,u_min);
Ut = kron(b_base,control_act);
b_ieqst = [Umax - Ut;Ut - Umin];%����ʽԼ��b�����Ʊ�����֮�͵ĸ�����
M = 10;%�ɳڱ�������
DU_max = kron(b_base,du_max);
DU_min = kron(b_base,du_min);
lb = [DU_min;0];%���Ʊ��������½�
ub = [DU_max;M];
end