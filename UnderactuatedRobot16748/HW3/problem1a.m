% Homework 3
% Problem 1 a
% Qishun Yu
% Worked with Huaiqian Shou


close all
clear all

% Controllability Check
syms x theta xdot thetadot
H = [2,cos(theta);cos(theta),1];
C = [0,-thetadot*sin(theta);0,0];
G = [0;sin(theta)];
B = [1;0];
Gq = [0,0;0,-1];
Am = [0,0,1,0;0,0,0,1;-inv(H)*Gq,-inv(H)*C];
Bm = [0;0;-inv(H)*B];

% rank of P matrix is 4, therefore controllable
P = [Bm,Am*Bm,Am^2*Bm,Am^3*Bm];
rank(P)

[theta,thetadot] = ndgrid(0:0.1:7,-4:0.1:4);
basin = zeros(size(theta));
for i = 1:size(theta,1)
    for j = 1:size(theta,2)  
        x = [0,theta(i,j),0,thetadot(i,j)]';
        u =0;
        output = lqr_func(x);
        
        xF = [0; pi;0;  0];
        if abs(output(end,3)-xF(2))<=5e-2 && abs(output(end,5)-xF(4))<=5e-2
            basin(i,j) = 1;
        end
    end
end

figure()
imagesc([theta(1,1) theta(end,1)],[thetadot(1,1) thetadot(1,end)],basin');
axis xy; 
colormap();
colorbar;
xlabel('theta initial condition');
ylabel('theta dot initial condition');
title('basin of attraction')

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

% system dynamics
    function xdot = dynamics(x,u)
        xdot = [x(3);
            x(4);
            (u + sin(x(2))*(x(4)^2 + cos(x(2))))/(1+sin(x(2))^2);
            (-u*cos(x(2)) - x(4)^2*cos(x(2))*sin(x(2)) - 2*sin(x(2)))/(1+sin(x(2))^2)];
    end
end
