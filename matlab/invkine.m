syms px py % pz se hace a mano con el marcador
p=[1 0 0 px
   0 1 0 py
   0 0 1 pz
   0 0 0 1];

l2=107;
l3=107;
l4=107;

r = sqrt(px^2+py^2);

q1=atan2(py,px);

cp2 = ((r-l4)^2 + l2^2 - l3^2) / (2*(r-l4)*l3);
sp2 = sqrt(1-cp2^2);
p2 = atan2(sp2,cp2);
q2 = pi/2 - p2;

cp3 = (l2^2 + l3^2 - (r-l4)^2) / (2*l2*l3);
sp3 = sqrt(1-cp3^2);
p3 = atan2(sp3,cp3);
q3 = pi - p3;

c4 = ((r-l4)^2 + l3^2 - l2^2) / (2*(r-l4)*l3);
s4 = sqrt(1-c4^2);
q4 = atan2(s4,c4);

py=l2+l3+l4;
px=0;
eval(q1)
eval(q2)
eval(q3)
eval(q4)
