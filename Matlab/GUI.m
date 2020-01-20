clc;
clear all;
close all;
%Q1.A;---------------------------------------------------------------------
t= (0.2:(1/100):3);
wm= 3*pi;
x= 8./(wm*t.^2) .* ((sin((1/2)*wm*t)).^3).*cos(2*wm*t);

figure; %new figure window
plot(t,x,'b','LineWidth',2); 
title('Q1-graph of x(t)'); xlabel('t [sec]','FontSize',12); ylabel('x(t)[V]','FontSize',12);
legend('x(t)');
hold on; %retain current graph in figure, to allow several graphs over the same
%Q1.B;---------------------------------------------------------------------
w=-17*pi:1/100:17*pi;
xjw=(pi./1i).*((tripuls((w-2.5*wm)./wm,2)+tripuls((w+1.5*wm)./wm,2)-tripuls((w-1.5*wm)./wm,2)-tripuls((w+2.5*wm)./wm,2)));
figure;
plot(w,abs(xjw),'r','LineWidth',2); 
title('Q1-graph of xjw'); xlabel('w [rad]','FontSize',12); ylabel('|x(jw)|','FontSize',12);
legend('|x(jw)|');
%Q1.C;---------------------------------------------------------------------
t= (0.2:(1/100):3);
wm= 3*pi;
x= 8./(wm*t.^2) .* ((sin((1/2)*wm*t)).^3).*cos(2*wm*t);
figure; %new figure window
plot(t,x,'b','LineWidth',2);    
title('Q1-graph of x(nT)'); xlabel('t [sec]','FontSize',12); ylabel('x(t)[V]','FontSize',12);
hold on; %retain current graph in figure, to allow several graphs over the same
tn=(0.2:(2/21):3);%2/21 it is Ts that we calculate.
y=8./(wm*tn.^2) .* ((sin((1/2)*wm*tn)).^3).*cos(2*wm*tn);
stairs(tn,y,'Color','m');
legend('x(t)','x(nT)');
%Q1.D;---------------------------------------------------------------------
ws=2*pi*10.5;
Ts=(2*pi)/ws;
xt=zeros(1,length(w));
for n = -1:1:1
    xt=xt+(pi./1i).*((tripuls((w-2.5*wm+n*ws)./wm,2)+tripuls((w+1.5*wm+n*ws)./wm,2)-tripuls((w-1.5*wm+n*ws)./wm,2)-tripuls((w+2.5*wm+n*ws)./wm,2)));
end 
xzoh=(exp(-1i*w*Ts/2)).*sinc(w/(2*pi/Ts)).*xt;

figure;
plot(w,abs(xzoh),'r','LineWidth',2); 
title('Q1-graph of Xzoh(jw)'); xlabel('w [rad]','FontSize',12); ylabel('|x(jw)|','FontSize',12);
legend('|Xzoh(jw)|');
%Q1.E;---------------------------------------------------------------------
%win= rectangularPulse(-ws/2,ws/2,w);
%h=exp(1i*pi.*w./ws)/(sinc(w/ws));
hw= rectangularPulse(-ws/2,ws/2,w).*exp(1i*pi.*w./ws)/(sinc(w/ws));
xrec=zeros(1,length(t));
i=0;
for s= 0.2:1/100:3
i=i+1;
xrec(i)=trapz(w,1/(2*pi).*hw.*xzoh.*exp(1i*w*s));
end
figure;
%plot(t,xrec,'r','LineWidth',2); 
%hold on;
%plot(t,x,'b','LineWidth',2); 
%title('Q1-graph of Xrec(t) comparing to X(t)'); xlabel('t [sec]','FontSize',12); ylabel('[V]','FontSize',12);
%legend('Xrec(t)','X(t)');


plot(w,abs(hw),'r','LineWidth',2); 
hold on;