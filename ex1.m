%% Use method of separation of variables to solve the Temperature Equation without control
clear;
clc;
L=1;
syms x t

c_sub = sin(pi*x/L);
c0 = int(c_sub,x,0,L);
T = c0;
for i=1:39 % use the sum of first ten terms(X(x)T(t)) to approximate T(t,x)
    c_sub = sin(pi*x/L)*cos(i*pi*x/L);
    c=2/L * int(c_sub,x,0,L);
    T = T + c*exp(-(i*pi/L)^2*t)*cos(i*pi*x/L);
end

t = [0:0.005:0.15];
x = [0:0.01:1];
[t,x] = meshgrid(t,x);
T_v = subs(T);
mesh(t,x,double(T_v));
xlabel('t');
ylabel('x');
zlabel('T');