function op=position_control(u)
global m g
xdes=u(1);
dxdes=u(2);
ddxdes=u(3);
ydes=u(4);
dydes=u(5);
ddydes=u(6);
zdes=u(7);
dzdes=u(8);
ddzdes=u(9);
psides=u(10);
x=u(11);
dx=u(12);
y=u(13);
dy=u(14);
z=u(15);
dz=u(16);
phi=u(17);
tht=u(18);
psi=u(19);
Kpz=4;Kdz=4;
Kpx=20;Kdx=30;
Kpy=10;Kdy=10;

%Pendefinisian Double Turunan Koordinat X,Y,Z pada C
ddxc=ddxdes+Kdx*(dxdes-dx)+Kpx*(xdes-x);
ddyc=ddydes+Kdy*(dydes-dy)+Kpy*(ydes-y);
ddzc=ddzdes+Kdz*(dzdes-dz)+Kpz*(zdes-z);

%Waktu yang diperlukan berasal dari turunan Koordinat X,Y,(Z+mg) pada C
t=[ddxc;ddyc;ddzc+m*g];

%didapatkan bahwa tnorm merupakan pembagian dari matriks t/matriks norm(t)
tnorm=t/norm(t);



sinTheta=cos(psides)*tnorm(1)+sin(psides)*tnorm(2);

thtc=asin(sinTheta);

sinPhi=(sin(psides)*tnorm(1)-cos(psides)*tnorm(2))/cos(thtc);

phic=asin(sinPhi);

psic=psides;

%Matriks Posisi R
R=[cos(psi)*cos(tht)-sin(phi)*sin(psi)*sin(tht) -sin(psi)*cos(phi) cos(psi)*sin(tht)+sin(psi)*sin(phi)*cos(tht);
      cos(tht)*sin(psi)+cos(psi)*sin(phi)*sin(tht)  cos(phi)*cos(psi) sin(psi)*sin(tht)-cos(psi)*cos(tht)*sin(phi);
                                -cos(phi)*sin(tht)           sin(phi)                           cos(phi)*cos(tht)];

%Representasi sinyal input kontrol u1
u1=dot(t,R*[0;0;1]);

op=[u1;phic;thtc;psic];

end