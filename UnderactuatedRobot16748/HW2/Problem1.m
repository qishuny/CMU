clear all
close all
clc

%Problem 1
%Qishun Yu

A = [0,1;0,0];
B = [0;1];

% LQR control
Q = 1*eye(2);
R = 1;

% part a
coeff = 0;
Qf = coeff*eye(2);
tSpan = [0,4];
[tout, xout,tKout,Kout] = fhQR(tSpan,A,B,Q,R,Qf);
plot_func(tout, xout,tKout,Kout);
title('state trajectory with T = 4, Qf = 0')

% part b
coeff = 1;
Qf = coeff*eye(2);
tSpan = [0,8];
[tout, xout,tKout,Kout] = fhQR(tSpan,A,B,Q,R,Qf);
plot_func(tout, xout,tKout,Kout);
title('state trajectory with T = 8, Qf = 1')

% part c
coeff = 10;
Qf = coeff*eye(2);
tSpan = [0,3];
[tout, xout,tKout,Kout] = fhQR(tSpan,A,B,Q,R,Qf);
plot_func(tout, xout,tKout,Kout);
title('state trajectory with T = 3, Qf = 10')

% part d
coeff = 100;
Qf = coeff*eye(2);
tSpan = [0,2];
[tout, xout,tKout,Kout] = fhQR(tSpan,A,B,Q,R,Qf);
plot_func(tout, xout,tKout,Kout);
title('state trajectory with T = 2, Qf = 100')

function plot_func(tout, xout,tKout,Kout)
figure();
clf;
kout = reshape(Kout,2,1000);

for i=1:2
    subplot(2,1,i); hold on;
    plot(tKout,kout(i,:));
    ylabel(['K(' num2str(i) ')']);
end
xlabel('Time')
subplot(2,1,1);
title('Finite-Horizon Quadratic Regulator Gains')

figure()
clf;
plot(xout(:,1),xout(:,2))
xlabel('x')
ylabel('xdot')

end



function [tout, xout,tKout,Kout] = fhQR(tSpan,A,B,Q,R,F)

z0 = reshape(F,4,1);

tB = fliplr(tSpan);
x0 =[0;0];
options = odeset();
options.RelTol = 1e-6;
options.AbsTol = 1e-6;
sol = ode45(@(t,z)lqr_func(t,z,A,B,Q,R),tB,z0,options);

tK = linspace(tSpan(2),tSpan(1),1000);
z = deval(sol,tK);
tKout = zeros(1000,1);
Kout = zeros(2,1,1000);
for i=1:1000
    zNow = z(:,i);
    tNow = tK(i);
    S = reshape(zNow(1:4),2,2);
    K = -R\(B'*S);
    tKout(i,:) = tNow;
    Kout(:,:,i) = K;
end

[t,x]=ode45(@(t,x)dynamics(t,x,A,B,R,deval(sol,t)),tSpan, x0,options);
tout = t;
xout = x;


end
function [xdot] = dynamics(t,x,A,B,R,sol)
    
    S = reshape(sol(1:4),2,2);
    K = -R\(B'*S);
    u = K*(x-[10;0]);
    xdot=A*(x-[10;0])+B*u;
end
function ds = lqr_func(~,S,A,B,Q,R)
S = reshape(S(1:4),2,2);
% solve for riccati equation
K = R\B'*S;
dS = -(A'*S + S*A - S*B*K + Q);
ds = reshape(dS,4,1);
end



