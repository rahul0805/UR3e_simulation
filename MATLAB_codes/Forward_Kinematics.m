syms t2 t3 t4 t5 a2 a3 d1 d4 d5 d6
%a = 0;
%b = -1.88;
%c = -2.14;
%d = -1.08; 
%e = -2.38;
%f = 0;
%d1 = 0.1519;
%a2 = -0.245;  
%a3 = -0.21325; 
%d4 = 0.13105;
%d5 = 0.08535;
%d6 = 0.0921; 
T1 = dhmatrix(0,d1,0,pi/2);
T2 = dhmatrix(t2,0,a2,0);
T3 = dhmatrix(t3,0,a3,0);
T4 = dhmatrix(t4,d4,0,pi/2);
T5 = dhmatrix(t5,d5,0,-pi/2);
T6 = dhmatrix(0,d6,0,0);
T_for = T1*T2*T3*T4*T5*T6;
%T_inv = (T1*T2)\[x;y;z;1];roborr

%eqn = sin(x) + cos(2*x) == 1;
%S = solve(eqn,x,'IgnoreAnalyticConstraints',true);