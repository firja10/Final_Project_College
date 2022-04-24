function op=attitude_control(u)
global I tuning_parameter

Ix=I(1,1);
Iy=I(2,2);
Iz=I(3,3);

dphi=u(1);
dtht=u(2);
dpsi=u(3);

s1=u(4);
s2=u(5);
s3=u(6);

dang=[dphi;dtht;dpsi];
alpha=[(Iy-Iz)*dpsi*dtht/Ix;(Iz-Ix)*dpsi*dphi/Iy;(Ix-Iy)*dphi*dtht/Iz]+tuning_parameter*dang;
beta=inv(I);
k1=55;
k2=55;
k3=55;
op=inv(beta)*(-alpha+[-k1*sign(s1);-k2*sign(s2);-k3*sign(s3)]);
end

