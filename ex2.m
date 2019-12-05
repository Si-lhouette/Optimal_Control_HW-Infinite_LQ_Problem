%% Use pdepe function to solve the Temperature Equation without control
clear;
clc;
global L 
L = 1;
x = linspace(0,L,50);
t = linspace(0,0.15,50);
m = 0;

sol = pdepe(m, @pdex1pde, @pdex1ic, @pdex1bc, x, t);
T = sol(:,:,1);
surf(x,t,T)
xlabel('x');
ylabel('t');
figure
plot(x,T(1,:))

function [c,f,s] = pdex1pde(x,t,T,DTDx)
    c=1;
    f=DTDx;
    s=0;
end

function T0 = pdex1ic(x)
    global L;
    T0 = sin(pi*x/L);
end

function [pl,ql,pr,qr] = pdex1bc(xl,Tl,xr,Tr,t)
    pl = 0;
    ql = 1;
    pr = 0;
    qr = 1;
end

