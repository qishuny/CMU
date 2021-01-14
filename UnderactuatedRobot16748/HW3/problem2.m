% Homework 3
% Problem 2
% Qishun Yu
% Worked with Huaiqian Shou

clear all;
close all;
clc;

% find the minimum upper and lower bound
i = 0.85;
[states,u] = singleshooting(i);

while abs(states(end,1)-pi)>1e-2
    i = i+0.01;
    [states,u] = singleshooting(i);
end
save('states.mat','states');
save('u.mat','u');
figure()
plot(states(:,1),states(:,2));
xlabel('theta');
ylabel('thetadot');
title('Trajectory Plot with upper bound and lower bound = 0.85/-0.85');

dt=0.025;
T=200*dt;
time=0:dt:T;
figure()
plot(time,u);
xlabel('t');
ylabel('control input');
title('Control Input plot');

function [states,u] = singleshooting(k)
dt=0.025;
T=200*dt;
time=0:dt:T;


u0=zeros(length(time),1);
x0=[0,0]';

% constrains

A = [];
b = [];
Aeq = [];
beq = [];
upper = repmat(k,201,1);
lower = repmat(-k,201,1);


% Find minimum of constrained nonlinear multivariable function
options =optimoptions(@fmincon,'TolFun', 0.00000001,'MaxIter', 10000, ...
    'MaxFunEvals', 100000,'Display','iter', ...
    'DiffMinChange', 0.001,'Algorithm', 'sqp');

u = fmincon(@(u) u'*u, u0, A,b,Aeq,beq, lower,upper,@func,options);
states = traj_func(x0,u,dt);

% inequality c constraints
    function [C,Ceq]=func(u)
        dt = 0.025;
        x0 = [0,0]';
        C = [];
        states = traj_func(x0,u,dt);
        Ceq = [pi-states(end,1);
            0-states(end,2)];   
    end

% trajectory function
    function states = traj_func(x0,u,dt)
        states(1,:) = x0;
        for iter = 2:201
            xTemp = states(iter-1,:);      
            states(iter,1) = xTemp(1)+dt*xTemp(2);
            states(iter,2) = xTemp(2)+dt*(-sin(xTemp(1))+u(iter-1));
        end
    end
end