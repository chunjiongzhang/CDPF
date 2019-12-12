%%
%被动目标跟踪之SIR算法
clear all;close all;clc;
%%
%-------------------------------设置参数-----------------------------------%
MC_num=3;
n=500;                                                                          %采样次数
T=2;                                                                            %采样周期
N=30;                                                                          %粒子数
delta_w=0.1;                                                                      %系统噪声均方差                                         
delta_v=[1,0.1*pi/180,0.1*pi/180];                                                         %测量噪声均方差
% delta_w=10^(-6);                                                                %不加噪声时，验证算法的正确性                                         
% delta_v=[10^(-6),10^(-3)*pi/180];  
%%
%----------建立模型得到平台的位置、目标的位置、目标距平台的距离和方位---------%
platform_v=3;                                                                   %平台速度                                             %平台速度
for i=1:100                                        %平台航向
    platform_phi(i)=40*pi/180;
end
for i=101:200
    platform_phi(i)=140*pi/180;
end
for i=201:300
    platform_phi(i)=50*pi/180;
end
for i=301:400
    platform_phi(i)=130*pi/180;
end
for i=401:n
    platform_phi(i)=60*pi/180;
end                                                         %平台机动时间间隔

target_v=5;                                                                     %目标速度
target_phi=120*pi/180;                                                           %目标航向的方位角
d(1)=1000;                                                                      %目标离平台的初始距离
phi(1)=0*pi/180;                                                              %目标相对于平台的初始方位角 

platform_x(1)=0;                                                                %平台的初始位置
platform_y(1)=0;
target_x(1)=platform_x(1)+d(1)*sin(phi(1));                                     %目标的初始位置
target_y(1)=platform_y(1)+d(1)*cos(phi(1));

for i=2:n
    platform_x(i)=platform_x(i-1)+platform_v*sin(platform_phi(i))*T;            %平台的位置
    platform_y(i)=platform_y(i-1)+platform_v*cos(platform_phi(i))*T;
    target_x(i)=target_x(i-1)+target_v*sin(target_phi)*T;                       %目标的位置
    target_y(i)=target_y(i-1)+target_v*cos(target_phi)*T;

    d(i)=sqrt((target_x(i)-platform_x(i))^2+(target_y(i)-platform_y(i))^2);     %目标离平台的距离
    phi(i)=atan2(target_x(i)-platform_x(i),target_y(i)-platform_y(i));          %目标相对于平台的方位
end


% platform_v=8;                                                                   %平台速度
% platform_phi(1)=100*pi/180;                                                     %平台初始方位角                                                 
% platform_phi_new(1)=90*pi/180;                                                  %平台新初始方位角
% platform_omega=2*pi/180;                                                        %平台的角速度
% platform_turntime=100;                                                          %平台机动时间间隔
% 
% target_v=5;                                                                     %目标速度
% target_phi=90*pi/180;                                                           %目标航向的方位角
% d(1)=1000;                                                                      %目标离平台的初始距离
% phi(1)=60*pi/180;                                                              %目标相对于平台的初始方位角 
% 
% platform_x(1)=0;                                                                %平台的初始位置
% platform_y(1)=0;
% target_x(1)=platform_x(1)+d(1)*sin(phi(1));                                     %目标的初始位置
% target_y(1)=platform_y(1)+d(1)*cos(phi(1));
% 
% k=0;                                                                             %(-1)^k用于调整平台的航向
% platform_yaw=60*pi/180;                                                          %偏航角，进行蛇形跟踪
% for i=2:n
%     if i>k*(platform_turntime/T)
%         k=k+1;
%     end
%     if i>10
%         platform_phi_new(i)=phi(i-1)+(-1)^k*platform_yaw;
%     else 
%         platform_phi_new(i)=phi(i-1);
%     end
%     if abs(platform_phi_new(i))>pi
%         platform_phi_new(i)=platform_phi_new(i)-sign(platform_phi_new(i))*2*pi;
%     end
%    
%         
%     if abs(platform_phi_new(i)-platform_phi(i-1))>pi
%         platform_phi(i-1)=platform_phi(i-1)-sign(platform_phi(i-1))*2*pi;
%     end
%     if abs(platform_phi_new(i)-platform_phi(i-1))>T*platform_omega
%         platform_phi(i)=platform_phi(i-1)+sign(platform_phi_new(i)-platform_phi(i-1))*T*platform_omega;
%     else
%         platform_phi(i)=platform_phi_new(i);
%     end
%     if abs(platform_phi(i))>pi
%         platform_phi(i)=platform_phi(i)-sign(platform_phi(i))*2*pi;
%     end
% 
%     platform_x(i)=platform_x(i-1)+platform_v*sin(platform_phi(i))*T;            %平台的位置
%     platform_y(i)=platform_y(i-1)+platform_v*cos(platform_phi(i))*T;
%     target_x(i)=target_x(i-1)+target_v*sin(target_phi)*T;                       %目标的位置
%     target_y(i)=target_y(i-1)+target_v*cos(target_phi)*T;
% 
%     d(i)=sqrt((target_x(i)-platform_x(i))^2+(target_y(i)-platform_y(i))^2);     %目标离平台的距离
%     phi(i)=atan2(target_x(i)-platform_x(i),target_y(i)-platform_y(i));          %目标相对于平台的方位
% end
%%
%绘图
figure;
plot(target_x(:),target_y(:),'b',platform_x(:),platform_y(:),'c');
grid on;
xlabel('X');ylabel('Y');
legend('目标的真实位置','平台的真实位置');
%%
%--------------------------------产生测量值--------------------------------%
d_noise=delta_v(1)*randn(2,100000);                                             %加入测量噪声
phi_noise=delta_v(2)*randn(1,100000);                                           
z=zeros(2,n);                                                                   %存储测量值（方位角）
measurement=zeros(2,n);                                                         %存储测量值（x,y）
for i=1:n
    d(i)=d(i)+d_noise(1,i);
    phi(i)=phi(i)+phi_noise(1,i);
    if i==1
        z(:,i)=[phi(i);0];
    else
        z(:,i)=[phi(i);(phi(i)-phi(i-1))/T];
    end
    measurement(:,i)=[platform_x(i)+d(i).*sin(phi(i));                                      
                      platform_y(i)+d(i).*cos(phi(i))];
end
%%
%-------------------------------PF算法------------------------------------%
F=[1,T,0,0;                                                                     %状态转移矩阵
    0,1,0,0;
    0,0,1,T;
    0,0,0,1];
G=[T^2,0;                                                                       %系统过程噪声矩阵
    T,0;
    0,T^2;
    0,T];                                          
R=diag([delta_v(1)^2;delta_v(2)^2]);                                            %测量噪声方差
                                                      
x_filter1=zeros(4,n,MC_num);
for t=1:MC_num
w_noise=delta_w*randn(2,10000); 
x_filter=zeros(4,n);                                                            %存储滤波值
P=[ 0.1,     0,     0,     0;                                                   %初始状态协方差
      0, 0.005,     0,     0;
      0,     0  , 0.1,     0;
      0,     0,     0, 0.005];
for i=1:100
    i
    if i==1
        x_filter(:,i)=[target_x(1),target_v*sin(target_phi),target_y(1),target_v*cos(target_phi)]';  %初始值 
        for j=1:N
            x_part(:,j)=x_filter(:,i)+sqrt(P)*rand(4,1);                                      %初始化（采用均匀分布）
        end
    else
        for j=1:N
            x_part1(:,j)=F*x_part(:,j)+G*w_noise(:,j);                              %采样
            z_part(:,j)=[atan2(x_part1(1,j)-platform_x(i),x_part1(3,j)-platform_y(i));
                         (x_part1(2,j)*(x_part1(3,j)-platform_y(i))-x_part1(4,j)*(x_part1(1,j)-platform_x(i)))/((x_part1(1,j)-platform_x(i))^2+(x_part1(3,j)-platform_y(i)))^2]; %预测值
            if abs(z(1,i)-z_part(1,j))>pi
                z_part(:,j)=z_part(:,j)-sign(z_part(:,j))*2*pi;
            end
%             if abs(z_part(:,j))>pi
%                 z_part(:,j)=z_part(:,j)-sign(z_part(:,j))*2*pi;
%             end
             c(:,j)=z(:,i)-z_part(:,j);                                                    %预测新息
%             if c(j)<phi_noise(i)
%                 u(j)=1;
%             else
%                 u(j)=0;
%             end
        end
        f=corrcoef(z_part(1,:),z_part(2,:));
        f=f(1,2);
%         delta(1)=std(z_part(1,:))
%         delta(2)=std(z_part(2,:))
        for j=1:N
%             w(j)=1/(2*pi*delta_v(2)*delta_v(3))*exp(-1/2*(c(1,j)^2/delta_v(2)^2+c(2,j)^2/delta_v(3)^2));                                   %重要性权值    
            w(j)=1/(2*pi*delta_v(2)*delta_v(3)*sqrt(1-f^2))*exp(-1/(2*(1-f^2))*(c(1,j)^2/delta_v(2)^2-2*f*c(1,j)/delta_v(2)*c(2,j)/delta_v(3)+c(2,j)^2/delta_v(3)^2));                                   %重要性权值    
        end
        mean(c,2)
%         mean(w)
        w_sum=sum(w)
        for j=1:N
            w(j)=w(j)/w_sum;                                                   %归一化
        end 

        for j=1:N                                                                   %重采样
            u=rand;
            s=0;
            for k=1:N
                s=s+w(k);
                if s>=u
                    x_part(:,j)=x_part1(:,k);
                    break;
                end
            end
        end

%         a=[];
%         b=0;
%         for j=1:N
%             if x_part(:,j)>0
%                 a=[a,x_part(:,j)];
%                 b=b+1;
%             end
%         end
%         c=[];
%         for j=1:(N/2/b)
%             c=[c,a];
%         end
%         x_part=c;
                        
        x_filter(:,i)=mean(x_part,2);                                              %PF值
    end
    x_filter1(:,i,t)=x_filter(:,i);
end
end
x_filter=sum(x_filter1,3)/MC_num;
%%
%绘图
figure;
plot(target_x(:),target_y(:),'b',measurement(1,:),measurement(2,:),'g.',x_filter(1,:),x_filter(3,:),'r');
grid on;
xlabel('X');ylabel('Y');
legend('目标的真实位置','观测值','滤波值');
%%
% 
% a=zeros(1,n);
% for i=1:n
%     for j=1:MC_num
%         a(i)=a(i)+(x_filter1(1,i,j)-target_x(i))^2+(x_filter1(3,i,j)-target_y(i))^2;
%     end
%     RMSE(i)=sqrt(a(i)/MC_num);
% end
% figure;
% plot(1:n,RMSE(:),'b');
% hold on;
% b=zeros(2,n);
% b(1,:)=RMSE(:);
% b(2,:)=1:n;
% [c,pos]=sort(b(1,:));                                                           %从小到大排序
% b(2,:)=b(2,pos);
% b(1,:)=c;
% plot([b(2,n),b(2,n)],[0,b(1,n)],'r');
% hold on;
% str1=num2str(b(2,n));
% str2=num2str(b(1,n));
% str=strcat('N=',str1,',RMSE=',str2,'(m)');
% text(b(2,n),0.8*b(1,n),str);
% grid on;
% xlabel('times');ylabel('RMSE/m');
% legend('目标位置的RMSE','目标位置RMSE的最大值');
% 
% %%
% %-----------------------------估计速度、航向-------------------------------%
% v_filter=zeros(1,n);
% phi_filter=zeros(1,n);
% v_filter(1)=0;
% phi_filter(1)=120*pi/180;
% for i=2:n
%     v_filter(i)=sqrt((x_filter(1,i)-x_filter(1,1))^2+(x_filter(3,i)-x_filter(3,1))^2)/((i-1)*T);               %目标的速度
%     phi_filter(i)=atan2((x_filter(1,i)-x_filter(1,1)),(x_filter(3,i)-x_filter(3,1)));                          %目标的航向
% end
% %%
% %绘图
% figure;
% subplot(2,1,1);
% % plot(1:n,target_v*ones(1,n),'b',1:n,sqrt(x_filter(2,1:n).^2+x_filter(4,1:n).^2),'r');
% plot(1:n,target_v*ones(1,n),'b',1:n,v_filter,'r');
% grid on;
% xlabel('时间');ylabel('速度v');
% legend('目标速度的真实值','滤波值');
% subplot(2,1,2);
% % plot(1:n,target_phi*ones(1,n)*180/pi,'b',1:n,atan2(x_filter(2,1:n),x_filter(4,1:n))*180/pi,'r');
% plot(1:n,target_phi*ones(1,n)*180/pi,'b',1:n,phi_filter*180/pi,'r');
% grid on;
% xlabel('时间');ylabel('方位角phi(度)');
% legend('目标方位角的真实值','滤波值');
% % figure()
% % plot(i,Hm*180/pi,'b',i,h(i)*180/pi,'r');
% % xlabel('时间');ylabel('航向H(度)');
% % legend('真实值','PF');