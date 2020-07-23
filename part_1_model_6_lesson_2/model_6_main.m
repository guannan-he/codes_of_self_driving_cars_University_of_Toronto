clear;
clc;
clf;

%%
x_ref = [0:0.1:60];%���Ҳο�·��
y_ref = 2 * sin(x_ref / pi);
[~,r] = size(y_ref);
L = 2;%���
v = 5;%����
k = 0.5;%Ԥ��ϵ��
t = 0.1;%ʱ����
ld = k * v;%Ԥ�����
x = 0;
y = 0;
p = 0;
res = zeros(2,3);%������
for i = 1:r
    [k,alpha] = find_pos(x_ref,y_ref,x,y,ld,p);%��Ԥ���λ�úͺ�������
    delta = atan(2 * L *sin(alpha) / ld);%����ת���
    [x,y,p] = update_s(x,y,p,delta,v,t,L);%����״̬
    if x > x_ref(end)
        break;%����·����Χֹͣ
    end
    res(i + 1,:) = [x,y,p];
end
%%
plot(x_ref,y_ref);
hold on;
plot(res(:,1),res(:,2));
legend('ref','act');