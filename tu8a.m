%�ڶ�ά�ռ�,�����˶������һ��(������)�˶�λ�á��ٶȡ����ٶ�����,�������˲��������д���
%ʵ���ҵĲ���
 
% ��������
N = 100;   %��������
Q = 5;      %��������
R = 5;      %��������
T = 10;     %����ʱ��
theta = pi/T;       %��ת�Ƕ�
distance = 60/T;    %ÿ���ߵľ���
WorldSize = 100;    %�����С
X = zeros(2, T);    %�洢ϵͳ״̬
Z = zeros(2, T);    %�洢ϵͳ�Ĺ۲�״̬
P = zeros(2, N);    %��������Ⱥ
PCenter = zeros(2, T);  %�������ӵ�����λ��
w = zeros(N, 1);         %ÿ�����ӵ�Ȩ��
err = zeros(1,T);     %���
X(:, 1) = [50; 20];     %��ʼϵͳ״̬
Z(:, 1) = [50; 20] + wgn(2, 1, 10*log10(R));    %��ʼϵͳ�Ĺ۲�״̬
 
%��ʼ������Ⱥ
for i = 1 : N
    P(:, i) = [WorldSize*rand; WorldSize*rand];
    dist = norm(P(:, i)-Z(:, 1));     %�����λ�����ľ���
    w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R);   %��Ȩ��
end
PCenter(:, 1) = sum(P, 2) / N;      %�������ӵļ�������λ��
 
%%
err(1) = norm(X(:, 1) - PCenter(:, 1));     %���Ӽ���������ϵͳ��ʵ״̬�����
figure(1);
set(gca,'FontSize',12);
hold on
plot(X(1, 1), X(2, 1), 'r.', 'markersize',30)   %ϵͳ״̬λ��
axis([0 100 0 100]);
plot(P(1, :), P(2, :), 'k.', 'markersize',5);   %��������λ��
plot(PCenter(1, 1), PCenter(2, 1), 'b.', 'markersize',25); %�������ӵ�����λ��
legend('True State', 'Particles', 'Particles Centroid');
title('Initial State');
hold off
