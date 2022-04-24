clear all
close all

%Pendefinisian Parameter Secara Global pada Quadcopter
global m I g tuning_parameter

%Massa Quadcopter
m = 0.486;

%Matriks Momen Inersia Quadcopter
I = [0.00025,   0,          0;
     0,         0.000232,   0;
     0,   0,          0.0003738];
 
%Konstanta Gaya Gravitasi Quadcopter
g=9.8;

%Lambda atau Tuning Parameter pada Quadcopter
tuning_parameter=100;  
sim('quad_control_fsmc_dua');
  
%Jarak Lengan Pusat Massa ke Quadcopter
d=1;

%Pendefinisian Nilai Koordinat merupakan 3 kolom dari setiap Matriks XYZ
x=xyz(:,1);y=xyz(:,2);z=xyz(:,3);


%Pendefinisian Nilai Koordinat Sudut merupakan 3 kolom dari setiap Matriks
%Sudut
phi=phi_tht_psi(:,1);tht=phi_tht_psi(:,2);psi=phi_tht_psi(:,3);
phiC=angc(:,1);thtC=angc(:,2);psiC=angc(:,3);


%Nilai Koordinat untuk menciptakan Lintasan Helix
%zup=[0;0;0.2];
zup=[0;0;0.2];

% Matriks Rotasi Pada Plant
for i=1:50:length(x)  
      Rotn=[cos(psi(i))*cos(tht(i))-sin(phi(i))*sin(psi(i))*sin(tht(i)) -sin(psi(i))*cos(phi(i)) cos(psi(i))*sin(tht(i))+sin(psi(i))*sin(phi(i))*cos(tht(i));
            cos(tht(i))*sin(psi(i))+cos(psi(i))*sin(phi(i))*sin(tht(i))  cos(phi(i))*cos(psi(i)) sin(psi(i))*sin(tht(i))-cos(psi(i))*cos(tht(i))*sin(phi(i));
                                      -cos(phi(i))*sin(tht(i))           sin(phi(i))                           cos(phi(i))*cos(tht(i))];
                             
%Pendefinisian Setiap Koordinat
      %A=[x(i);y(i);z(i)]+Rotn*[0;-d;0];
      %B=[x(i);y(i);z(i)]+Rotn*[d;0;0];
      %C=[x(i);y(i);z(i)]+Rotn*[0;d;0];
      %D=[x(i);y(i);z(i)]+Rotn*[-d;0;0];
      
      A=[x(i);y(i);z(i)]+Rotn*[0;-d;0];
      B=[x(i);y(i);z(i)]+Rotn*[d;0;0];
      C=[x(i);y(i);z(i)]+Rotn*[0;d;0];
      D=[x(i);y(i);z(i)]+Rotn*[-d;0;0];
      
      Zup=[x(i);y(i);z(i)]+Rotn*zup;
      ACx=linspace(A(1),C(1),10);
      ACy=linspace(A(2),C(2),10);
      ACz=linspace(A(3),C(3),10);
      BDx=linspace(B(1),D(1),10);
      BDy=linspace(B(2),D(2),10);
      BDz=linspace(B(3),D(3),10);
      Zupx=linspace(x(i),Zup(1),10);
      Zupy=linspace(y(i),Zup(2),10);
      Zupz=linspace(z(i),Zup(3),10);
    
    
    subplot(221)
    
    plot3(A(1),A(2),A(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','b');
    hold on
    plot3(B(1),B(2),B(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','b');
    hold on
    plot3(C(1),C(2),C(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on
    plot3(D(1),D(2),D(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
%   plot3(ref(:,1),ref(:,2),ref(:,3),'--gs','LineWidth',1,'MarkerSize',1,'MarkerEdgeColor','r');
    plot3(ref(:,1),ref(:,2),ref(:,3),'--gs','LineWidth',0.5,'MarkerSize',0.5,'MarkerEdgeColor','g');
    hold on
    plot3(ACx,ACy,ACz,'-b','LineWidth',1);
    hold on
    plot3(BDx,BDy,BDz,'-b','LineWidth',1);
    hold on
    plot3(Zupx,Zupy,Zupz,'linewidth',1);
    hold off
    axis([-15 +15 -15 +15 -15 +15]); 
    axis square
    grid
    xlabel('x axis');
    ylabel('y axis');
    zlabel('z axis');
    title(['Time=',num2str(i*0.01)])
    pause(0.01);
    
    subplot(222)
    title(['Time=',num2str(i*0.01)])
    plot(ACx,ACy,'-b','LineWidth',1);
    hold on;
    plot(BDx,BDy,'-b','LineWidth',1);
    hold on;
    plot(A(1),A(2),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(B(1),B(2),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(C(1),C(2),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(D(1),D(2),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    plot(ref(:,1),ref(:,2));
    hold off;
    grid;
    axis([-15 +15 -15 +15]);
    axis square
    xlabel('x axis');
    ylabel('y axis');
    title(['Time=',num2str(i*0.01)])
    
    subplot(223)
    plot(ACx,ACz,'-b','LineWidth',1);
    hold on;
    plot(BDx,BDz,'-b','LineWidth',1);
    hold on;
    plot(A(1),A(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(B(1),B(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(C(1),C(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(D(1),D(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    plot(ref(:,1),ref(:,3));
    hold off;
    grid;
    axis([-15 +15 -15 +15]);
    axis square
    xlabel('x axis');
    ylabel('z axis');
    title(['Time=',num2str(i*0.01)])
    
    subplot(224)
    plot(ACy,ACz,'-b','LineWidth',1);
    hold on;
    plot(BDy,BDz,'-b','LineWidth',1);
    hold on;
    plot(A(2),A(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(B(2),B(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;                     
    plot(C(2),C(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    hold on;
    plot(D(2),D(3),'-mo','MarkerFaceColor',[1 0 0],'MarkerSize',3,'MarkerEdgeColor','r');
    plot(ref(:,2),ref(:,3));
    hold off;
    grid;
    axis([-15 +15 -15 +15]);
    axis square
    xlabel('y axis');
    ylabel('z axis');
    title(['Time=',num2str(i*0.01)])
end
figure(2)
subplot(321);
plot(t,x,'-g','Linewidth',2);
hold on
plot(t,ref(:,1),'-r','Linewidth',1);
hold off;
grid;

% Penjudulan Vertikal (1)
xlabel('time(s)');
ylabel('x(m)');
%title('x(m) vs time(s)');

legend('Actual Measure','Reference')
subplot(322);
plot(t,(180/pi)*phiC,'-r','Linewidth',1);
hold on;
plot(t,(180/pi)*phi,'-g','Linewidth',3);
hold off;
grid;

% Penjudulan Vertikal (1)
xlabel('time(s)');
ylabel('roll (in deg)');
%title('roll (in deg) vs time(s)');
legend('Reference','Actual Measure')
subplot(323);
plot(t,y,'-g','Linewidth',2);
hold on
plot(t,ref(:,2),'-r','Linewidth',1);
hold off;
grid;
% Penjudulan Vertikal (2)
xlabel('time(s)');
ylabel('y(m)');
%title('y(m) vs time(s)')
legend('Actual Measure','Reference')
subplot(324);
plot(t,(180/pi)*thtC,'-r','Linewidth',1);
hold on;
plot(t,(180/pi)*tht,'-g','Linewidth',3);
hold off;
grid;
% Penjudulan Vertikal (3)
xlabel('time(s)');
ylabel('Pitch (in deg)');
%title('Pitch (in deg) vs time(s)');
legend('Reference','Actual Measure')
subplot(325);
plot(t,z,'-g','Linewidth',2);
hold on
plot(t,ref(:,3),'-r','Linewidth',1);
hold off;
grid;
% Penjudulan Vertikal (4)
xlabel('time(s)');
ylabel('z(m)');
%title('z(m) vs time(s)');
legend('Actual Measure','Reference')
subplot(326);
plot(t,(180/pi)*psi,'-g','Linewidth',3);
hold on;
plot(t,ref(:,4),'-r','Linewidth',1);
hold off;
grid;
% Penjudulan Vertikal (4)
xlabel('time(s)');
ylabel('yaw (in deg)');
%title('yaw (in deg) vs time(s)');
legend('Actual Measure','Reference')
figure(3)
subplot(411)
plot(t,u1);
grid
title('Thrust (N) vs Time(s)');
subplot(412)
plot(t,u2(:,1));
grid
title('Rolling I/P(N m) vs Time(s)');
subplot(413)
plot(t,u2(:,2));
grid
title('Pitching I/P(N m) vs Time(s)');
subplot(414)
plot(t,u2(:,3));
grid
title('Yawing I/P(N m) vs Time(s)');

figure(4)

subplot(221);
plot(t,ref(:,1)-x,'-r','Linewidth',2);
grid;
title('error in x(m) vs time(s)')
subplot(222);
plot(t,ref(:,2)-y,'-r','Linewidth',2);
grid;
title('error in y(m) vs time(s)')
subplot(223);
plot(t,ref(:,3)-z,'-r','Linewidth',2);
grid;
title('error in z(m) vs time(s)')
subplot(224);
plot(t,(ref(:,4)-psi)*180/pi,'-r','Linewidth',2);
grid;
title('error in psi(yaw)(deg) vs time(s)')