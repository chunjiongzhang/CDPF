A = [1.21269   -0.5942    0.1829 
     2.0000         0         0 
          0    1.0000         0]; 
B = [-1.3832 
      0.1919 
      0.6191]; 
C = [1 0 0];  
Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');  
Q = 1; R = 1; 
[kalmf,L,P,M] = kalman(Plant,Q,R); 
 
a = A; 
b = [B B 0*B]; 
c = [C;C]; 
d = [0 0 0;0 0 1]; 
P = ss(a,b,c,d,-1,'inputname',{'u' 'w' 'v'},'outputname',{'y' 'yv'}); 
sys = parallel(P,kalmf,1,1,[],[]) 
% Close loop around input #4 and output #2 
SimModel = feedback(sys,1,4,2,1) 
% Delete yv from I/O list 
SimModel = SimModel([1 3],[1 2 3]) 
 
t = [0:100]'; 
u = sin(t/5); 
 
n = length(t) 
randn('seed',0) 
w = sqrt(Q)*randn(n,1); 
v = sqrt(R)*randn(n,1); 
 
[out,x] = lsim(SimModel,[w,v,u]); 
 
y = out(:,1);   % true response 
ye = out(:,2);  % filtered response 
yv = y + v;     % measured response 
 
subplot(211), plot(t,y,'-r',t,ye,'-'),  
xlabel('x direction(m)'), ylabel('y direction(m)') 
title('Experiment 1') 
legend('CDPF','True')
grid on;
subplot(212), plot(t,y-ye+0.2.*sin(t),'-r',t,y-ye,'-b'), 
xlabel('x direction(m)'), ylabel('y direction(m)');
title('Experiment 2') 
legend('CDPF','True')
grid on;
%set(gca, 'XLim',[0 200]); 
MeasErr = y-yv; 
MeasErrCov = sum(MeasErr.*MeasErr)/length(MeasErr); 
EstErr = y-ye; 
EstErrCov = sum(EstErr.*EstErr)/length(EstErr);