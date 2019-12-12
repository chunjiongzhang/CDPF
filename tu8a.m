%在二维空间,假设运动物体的一组(非线性)运动位置、速度、加速度数据,用粒子滤波方法进行处理
%实验室的博客
 
% 参数设置
N = 100;   %粒子总数
Q = 5;      %过程噪声
R = 5;      %测量噪声
T = 10;     %测量时间
theta = pi/T;       %旋转角度
distance = 60/T;    %每次走的距离
WorldSize = 100;    %世界大小
X = zeros(2, T);    %存储系统状态
Z = zeros(2, T);    %存储系统的观测状态
P = zeros(2, N);    %建立粒子群
PCenter = zeros(2, T);  %所有粒子的中心位置
w = zeros(N, 1);         %每个粒子的权重
err = zeros(1,T);     %误差
X(:, 1) = [50; 20];     %初始系统状态
Z(:, 1) = [50; 20] + wgn(2, 1, 10*log10(R));    %初始系统的观测状态
 
%初始化粒子群
for i = 1 : N
    P(:, i) = [WorldSize*rand; WorldSize*rand];
    dist = norm(P(:, i)-Z(:, 1));     %与测量位置相差的距离
    w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R);   %求权重
end
PCenter(:, 1) = sum(P, 2) / N;      %所有粒子的几何中心位置
 
%%
err(1) = norm(X(:, 1) - PCenter(:, 1));     %粒子几何中心与系统真实状态的误差
figure(1);
set(gca,'FontSize',12);
hold on
plot(X(1, 1), X(2, 1), 'r.', 'markersize',30)   %系统状态位置
axis([0 100 0 100]);
plot(P(1, :), P(2, :), 'k.', 'markersize',5);   %各个粒子位置
plot(PCenter(1, 1), PCenter(2, 1), 'b.', 'markersize',25); %所有粒子的中心位置
legend('True State', 'Particles', 'Particles Centroid');
title('Initial State');
hold off
