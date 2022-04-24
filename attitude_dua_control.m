function op=attitude_dua_control(u)
global I tuning_parameter

%Nilai Momen Inersia Ix = 0.00025
Ix=I(1,1);

%Nilai Momen Inersia Iy = 0.00032
Iy=I(2,2);


%Nilai Momen Inersia Iz = 0.0003738
Iz=I(3,3);


%Pendefinisian bahwa dphi berasal dari masukkan sinyal pertama 
dphi=u(1);
%Pendefinisian bahwa dphi berasal dari masukkan sinyal kedua 
dtht=u(2);
%Pendefinisian bahwa dphi berasal dari masukkan sinyal ketiga 
dpsi=u(3);

%Pendefinisian bahwa k1sign(s1) berasal dari masukkan sinyal keempat 
k1signs1=u(4);
%Pendefinisian bahwa k2sign(s2) berasal dari masukkan sinyal kelima 
k2signs2=u(5);
%Pendefinisian bahwa k3sign(s3) berasal dari masukkan sinyal keenam 
k3signs3=u(6);

%setiap turunan sudut dijadikan 1 array 
dang=[dphi;dtht;dpsi];

%setiap turunan sudut dijadikan acuan untuk menghitung nilai alpha yang
%berfungsi untuk mencari salah satu parameter dari turunan sliding variabel
alpha=[(Iy-Iz)*dpsi*dtht/Ix;(Iz-Ix)*dpsi*dphi/Iy;(Ix-Iy)*dphi*dtht/Iz]+tuning_parameter*dang;
beta=inv(I);

betainv = inv(beta);

%Persamaan untuk Control Input
op=betainv*(-alpha+[-k1signs1;-k2signs2;-k3signs3]);
end
