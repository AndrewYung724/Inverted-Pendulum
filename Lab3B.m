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
rt=10;
T = 0.005;
%% Week 2 Downward Linear Est
% State Equations
A=[0 1 0 0
    0 -Km^2*Kg^2/(R*r^2*(M-mp*l/L)) -g*mp*l/(L*(M-mp*l/L)) 0
    0 0 0 1
    0 -Km^2*Kg^2/(M*R*r^2*(L-mp*l/M)) -g/(L-mp*l/M) 0];
B=[0; Km*Kg/(R*r*(M-mp*l/L))  ; 0 ;Km*Kg/(M*R*r*(L-mp*l/M))];
C=[1 0 0 0;0 0 1 0];
D=zeros(1,1);
sys=ss(A,B,C,D);
%C2D
dsys=c2d(sys,T,'zoh');
[Ad,Bd,Cd,Dd]=ssdata(dsys);

pz=[0.99+0.099i 0.99-0.099i 0.988 0.97]*1.0007;   %Same poles
K=place(Ad,Bd,pz);

pL=[0.99+0.099i 0.99-0.099i 0.988 0.97]*0.87; % Observer poles 2-4 times faster
Lo=place(Ad',Cd',pL)';
X_obs_0=[0;0;0;0];
X0=[0;0;pi/4;0];
X_equ=[0;0;pi;0];
% SIMULATE
out=sim('Lab3BLine.slx',rt);
t=out.tout;
XLin=out.XLin;
Xst=out.Xst;

err=XLin-Xst;
figure(1)
subplot(221);plot(t,XLin);
title('Linear Model');xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
subplot(222);plot(t,Xst);
title('Estimation of Linear Model');xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
subplot(223);plot(t,err);
title('Linear Approx Estimation Error');
xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
subplot(224);plot(t,out.In);hold on;yline(20);yline(-20); hold off
title('State Feedback Control');
xlabel('Time(s)');ylabel('Voltage(V)')


XLin(:,3)=XLin(:,3)+pi;
%cartpole_animate(t,XLin,4,['Linear Est Downward IC=',num2str(X0')]);

%% Week 2 Downward
% SIMULATE
use_tap=1;
X_equ=[0;0];
X0=[0;0;pi/4;0];
sim('Lab3B_sim_starter.slx',rt);
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
lbe=['Real Plant Tap Down IC=', num2str(X0')];
%cartpole_animate(t,Xst.Data,5,lbe)
%% Week 2 Upward Linear
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

pz=[0.98+0.099i 0.98-0.099i 0.996 0.99];   %Same poles
K=place(Ad,Bd,pz);

pL=pz*0.87; % Observer poles 2-4 times faster
Lo=place(Ad',Cd',pL)';
X_obs_0=[0;0;0;0];
X0=[0;0;pi/4;0];
X_equ=[0;0];
% SIMULATE
out=sim('Lab3BLine.slx',rt);
t=out.tout;
XLin=out.XLin;
Xst=out.Xst;

err=XLin-Xst;
figure(1)
subplot(221);plot(t,XLin);
title('Linear Model');xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
subplot(222);plot(t,Xst);
title('Estimation of Linear Model');xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
subplot(223);plot(t,err);
title('Linear Approx Estimation Error');
xlabel('Time(s)');ylabel('Distance(cm)/Angle(rad)');legend('Pos','Vel','Ang','Ang Vel')
subplot(224);plot(t,out.In);hold on;yline(20);yline(-20); hold off
title('State Feedback Control');
xlabel('Time(s)');ylabel('Voltage(V)')


XLin(:,3)=XLin(:,3);
%cartpole_animate(t,XLin,4,['Linear Est Upward IC =', num2str(X0')]);

%% Week 2 Upward Test
% SIMULATE
use_tap=0;
X_equ=[0;0];
X0=[0;0;5*pi/180;0];
pz=[0.999 0.991 0.992 0.993];   %Same poles
K=place(Ad,Bd,pz);

sim('Lab3B_sim_starter.slx',rt);
t=Y_out.Time;
figure(4)
subplot(221); plot(t,Y_out.Data)
title('Upward Position & Angle');xlabel('Time(s)');ylabel('Position(cm)/Angle(rad)');legend('Pos','Ang')

subplot(222);plot(t,err);hold on; yline(0);hold off
title('Est ERROR Upward Position & Angle');xlabel('Time(s)');ylabel('Position(cm)/Angle(rad)');legend('Pos','Ang')

subplot(223);plot(t,In);hold on; yline(-20); yline(20);hold off
title('State Feedback Control');xlabel('Time(s)');ylabel('Voltage (V)')


lbe=['Upward Position & Angle IC= ',num2str(X0')];
%cartpole_animate(t,Xst.Data,5,lbe)


