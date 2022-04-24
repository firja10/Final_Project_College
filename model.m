function op=model(u)
global m I g

%Sinyal Input Kontrol U1 dan U2
u1=u(1);
u2=[u(2);u(3);u(4)];

%Turunan dari setiap Sudut dan Sudut
dphi=u(5);
dtht=u(6);
dpsi=u(7);
phi=u(8);
tht=u(9);
psi=u(10);


% Matriks Posisi Quadcopter
R=[cos(psi)*cos(tht)-sin(phi)*sin(psi)*sin(tht) -sin(psi)*cos(phi) cos(psi)*sin(tht)+sin(psi)*sin(phi)*cos(tht);
      cos(tht)*sin(psi)+cos(psi)*sin(phi)*sin(tht)  cos(phi)*cos(psi) sin(psi)*sin(tht)-cos(psi)*cos(tht)*sin(phi);
                                -cos(phi)*sin(tht)           sin(phi)                           cos(phi)*cos(tht)];


% R double dot untuk percepatan pada Quadcopter                            
ddr=(1/m)*([0;0;-m*g]+R*[0;0;u1]);

%Turunan dari keseluruhan sudut(Kecepatan Sudut)direpresentasikan dalam bentuk array 
dang=[dphi;dtht;dpsi];                            

%Representasi Percepatan Sudut Quadcopter
ddang=inv(I)*u2-inv(I)*cross(dang,I*dang);

%Representasi Luaran Sinyal Output dari Model
op=[ddr;ddang];
end