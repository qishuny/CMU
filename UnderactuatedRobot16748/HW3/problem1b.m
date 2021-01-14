% Homework 3
% Problem 1 b
% Qishun Yu
% Worked with Huaiqian Shou

clear all
close all
clc

% x=[0 0 0 0]';
xF=[0 pi 0 0]';
EF=1;
kE = 10;
kP = 1;
kd = 1;
dt = 0.01;
% move the cart a little 
x = [0 1e-3 0 1e-3]';
output=[x'];

while abs((x(2)-xF(2)))>1 
    E=0.5*x(4)^2-cos(x(2));
    Ediff=E-EF;
    xddotF = kE*x(4)*cos(x(2))*Ediff - kP*x(1) - kd*x(3);
    u=(2-cos(x(2))^2)*xddotF-sin(x(2))*cos(x(2))-(x(4)^2)*sin(x(2));
    xdot = dynamics(x,u);
    x = x + dt*xdot;
    output(end+1,:)= x';
end

output1 = lqr_func(output(end,:)');
output = [output;output1(:,2:5)];
figure()
plot(output(:,1),output(:,3))
xlabel('x');
ylabel('xdot');
title('phase space defined by (x,xdot)')
figure()
plot(output(:,2),output(:,4))
xlabel('theta');
ylabel('thetadot');
title('phase space defined by (theta,thetadot)')

function output = lqr_func(x)
% time step and time limit
T = 10;
dt = 0.01;

% linearized at [0,pi,0,0]'
A=[0 0 1 0;
    0 0 0 1;
    0 1 0 0;
    0 2 0 0];
B=[0 0 1 1]';

%lqr
Q = 1*eye(4,4);
R = 1;
K = lqr(A,B,Q,R);

xF = [0, pi, 0, 0]';
output=[0,x'];

% simulation
for t=dt:dt:T
    xdiff=(x-xF);
    u = -K*xdiff;
    xdot = dynamics(x,u);
    x = x + dt*xdot;
    output(end+1,:)=[t,x'];
end 
end

% system dynamics
function xdot = dynamics(x,u)
        xdot = [x(3);
            x(4);
            (u + sin(x(2))*(x(4)^2 + cos(x(2))))/(1+sin(x(2))^2);
            (-u*cos(x(2)) - x(4)^2*cos(x(2))*sin(x(2)) - 2*sin(x(2)))/(1+sin(x(2))^2)];
end