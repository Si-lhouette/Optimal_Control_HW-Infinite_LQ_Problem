clear;
clc;
syms x t
L = 1;
%% Generate primary function 
X = [];
for i = 0:4
    X = [X, cos(pi*i*x/L)];
end

%% Calculate P/Q/R
P = zeros(5,5);
Q = zeros(5,5);
R = zeros(5,1);
for m = 1:5
    xm = X(m);
    R(m) = int(dirac(x-0.8*L)*xm,x,0,L);   % Point temperature control:g(x)u(t); g(x) =  impulse function(x-0.8L)
    for n=1:5
        xn = cos((n-1)*pi*x/L);       
        P(m,n) = int(xn*xm,x,0,L);
        
        dxn = diff(xn,x);
        dxm = diff(xm,x);
        Q(m,n) = int(dxn*dxm,x,0,L);
    end
end

%% Calculate  state-space model
A = -inv(P)*Q;
B = inv(P)*R;
C = subs(X,x,L/2); % output matrix to observe temperature at L/2

%% Calculate optimal control & optimal trajectory
yr = sin(t); % expect output
Qw = 10; % to limit e(t)
Rw = 1; % to limit change of u(t)

Pw = are(double(A),double(B*inv(Rw)*B'),double(C'*Qw*C)); % continuous-time Riccati equation
g = inv(Pw*B*inv(Rw)*B'-A')*C'*Qw*yr;

% Calculate initial T(t)
c_sub = sin(pi*x/L);
c0 = int(c_sub,x,0,L);
T0 = [c0];
for i = 1:4
    c_sub = sin(pi*x/L)*cos(i*pi*x/L);
    c=2/L * int(c_sub,x,0,L);
    T0 = [T0;c];
end

% Calculate optimal trajectory
[t,T_v] = ode45(@(t,T) odefun(t,T,A,B,Rw,Pw,g),[0,10],double(T0)); % timespan = 0-10s

y = C*T_v';
y = double(y);
figure
plot(t,y,'Linewidth',1.3)
hold on;
plot(t,subs(yr,t),'Linewidth',1.3)
legend('y-real','y-expect')
grid on
xlabel('t/s');
ylabel('y');

% Calculate optimal control
[k,~] = size(T_v);
U = [];
for i=1:k
    u = -inv(Rw)*B'*(Pw*T_v(i,:)' - double(subs(g,t(i))));
    U = [U;u];
end
figure
plot(t,U,'Linewidth',1.3)
xlabel('t/s');
ylabel('u');
grid on

% state-space model
function dT = odefun(t,T,A,B,Rw,Pw,g)
    dT = (A-B*inv(Rw)*B'*Pw)*T + B*inv(Rw)*B'*double(subs(g,t));
end


        