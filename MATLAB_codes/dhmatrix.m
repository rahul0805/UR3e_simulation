function [T] = dhmatrix(theta,d,a,alpha)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
n = 1;
k = 1;
p = 1;
q = 1;
if(abs(alpha) == pi/2)
    n = 0;
end

if(abs(theta) == pi/2)
    k = 0;
end

if(abs(alpha) == pi)
    p = 0;
end

if(abs(theta) == pi)
    q = 0;
end

T = [k*cos(theta),-n*q*sin(theta)*cos(alpha),p*q*sin(theta)*sin(alpha),a*k*cos(theta);
    q*sin(theta),n*k*cos(theta)*cos(alpha),-k*cos(theta)*sin(alpha)*p,a*q*sin(theta);
    0,p*sin(alpha),n*cos(alpha),d;
    0,0,0,1;
    ];
end

