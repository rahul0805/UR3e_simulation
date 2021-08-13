t = 0:0.01:2;

  
x_i = 0.4570;
y_i = 0.2610;
z_i = 0.0670;
x_f = -0.3290;   
y_f = 0.1391; 
z_f = 0.1210;

t2_i = -0.0102; 
t2_f = -1.88;
t3_i = 0.0370;
t3_f = -2.14;
t4_i = -0.0268;
t4_f = -2.38;
t5_i = 0;
t5_f = -1.51;

A = trajectory(0,2,t2_i,t2_f);
B = trajectory(0,2,t3_i,t3_f);
C = trajectory(0,2,t4_i,t4_f);
D = trajectory(0,2,t5_i,t5_f);
E = trajectory(0,2,x_i,x_f);
F = trajectory(0,2,y_i,y_f);
G = trajectory(0,2,z_i,z_f);

t2 = zeros(size(t));
t3 = zeros(size(t));
t4 = zeros(size(t));
t5 = zeros(size(t));
x = zeros(size(t));
y = zeros(size(t));
z = zeros(size(t));

for i = 1:length(t)
    T = [1, t(i), t(i)^2, t(i)^3, t(i)^4, t(i)^5, t(i)^6, t(i)^7, t(i)^8, t(i)^9];
    t2(i) = T*A;
    t3(i) = T*B;
    t4(i) = T*C;
    t5(i) = T*D;
    x(i) = T*E;
    y(i) = T*F;
    z(i) = T*G;
end
figure(1);
subplot(2,2,1)
plot(t,t2);
xlabel("time");
ylabel("theta 2");
title("theta 2 vs time");
subplot(2,2,2)
plot(t,t3);
xlabel("time");
ylabel("theta 3");
title("theta 3 vs time");
subplot(2,2,3)
plot(t,t4);
xlabel("time");
ylabel("theta 4");
title("theta 4 vs time");
subplot(2,2,4)
plot(t,t5);
xlabel("time");
ylabel("theta 5");
title("theta 5 vs time");

figure(2);
subplot(3,1,1)
plot(t,x);
xlabel("time");
ylabel("x");
title("x vs time");
subplot(3,1,2)
plot(t,y);
xlabel("time");
ylabel("y");
title("y vs time");
subplot(3,1,3)
plot(t,z);
xlabel("time");
ylabel("z");
title("z vs time");

