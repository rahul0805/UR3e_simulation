%Inverse kinematics 
function [theta] = inv_DnC()
%Give the transformation for the end effector wrt ground frame as input
T =    [    
         1.0000         0         0   -0.4580
         0         0   -1.0000   -0.2232
         0    1.0000         0    0.0540
         0         0         0    1.0000
        ];
     
R = T(1:3,1:3);
t = T(1:3,4);

d1 = 0.1519;
a2 = -0.245;  
a3 = -0.21325; 
d5 = 0.08535;
d6 = 0.0921; 

phi = atan2(R(3,1),R(1,1));

t5 = atan2(-R(2,1),-R(2,3));

w1 = d5*sin(phi)-d6*sin(t5)*cos(phi);
w2 = d1-d5*cos(phi)-d6*sin(t5)*sin(phi);

p1 = t(1) - w1;
p2 = t(3) - w2;

c3 = (p1^2 + p2^2 - a2^2 - a3^2)/(2*a2*a3);
t3 = acos(c3);

t2 = asin((p1^2 + p2^2 + a2^2 - a3^2)/(2*a2*sqrt(p1^2 + p2^2))) - asin(p1/sqrt(p1^2 + p2^2));

t4 = phi - t2 - t3;

theta = [0 t2 t3 t4 t5 0];
end

