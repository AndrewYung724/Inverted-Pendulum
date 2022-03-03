%% Params (TABLE 1)
Km = 0.00767;
Kg = 3.7;
R = 2.6;
r = 0.00635;
mc = 0.455;
mp = 0.21;
I = 0.00651;
l = 0.305;
g = 9.81;

M = mc + mp;
L = (I + mp*l^2) / (mp * l);
rt=7.5;
T = 0.005;
%% Week 2 Upward Test
% State Equations
A=[0 1 0 0
    0 -Km^2*Kg^2/(R*r^2*(M-mp*l/L)) -g*mp*l/(L*(M-mp*l/L)) 0
    0 0 0 1
    0 Km^2*Kg^2/(M*R*r^2*(L-mp*l/M)) g/(L-mp*l/M) 0];
B=[0; Km*Kg/(R*r*(M-mp*l/L))  ; 0 ;-Km*Kg/(M*R*r*(L-mp*l/M))];
C=[1 0 0 0 ; 0 0 1 0];
D=zeros(1,1);
sys=ss(A,B,C,D);
%C2D
dsys=c2d(sys,T,'zoh');
[Ad,Bd,Cd,Dd]=ssdata(dsys);

pz=[0.985+0.1i 0.985-0.1i 0.995 0.996];   %Same poles
K=place(Ad,Bd,pz);

pL=pz*0.95; % Observer poles 2-4 times faster
Lo=place(Ad',Cd',pL)';
X_obs_0=[0;0;0;0];
X0=[0;0;pi/4;0];
X_equ=[0;0];
% Week 2 Upward Test
% SIMULATE
use_tap=1;
sim('Lab3B_sim_starter.slx')
t=Y_out.Time;
figure(4)
subplot(221);plot(Y_out);
title('Plant Known States');xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Ang')
subplot(222);plot(Xst);
title('Estimation of Plant');xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
est=[Xst.Data(:,1),Xst.Data(:,3)];
err=Y_out.Data-est;
subplot(223);plot(t,err);
title('Linear Approx Estimation Error');
xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
subplot(224);plot(t,In);hold on;yline(20);yline(-20); hold off
title('State Feedback Input');xlabel('Time(s)');ylabel('Voltage');
cartpole_animate(t,Xst.Data,5,lbe)