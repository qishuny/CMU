clear all
close all
clc

%Problem 3
%Qishun Yu

%initial conditions
x0=[1,2];
u = 0;


%Simulation
[tout1,xout1]=ode45(@minimum_time,[0:.01:10],x0);
[tout2,xout2]=ode45(@lqrfunc,[0:.01:20],x0);

t1 = tout1(findt(xout1))
t2 = tout2(findt(xout2))

%Plot
hold on
plot(xout1(:,1),xout1(:,2),'b')
plot(xout2(:,1),xout2(:,2),'r')
xlabel('x');
ylabel('x dot');
legend('minimum-time policy','LQR policy')
hold off


function tstable = findt(xout)
for i = 1:size(xout,1)
    if norm([xout(i,1),xout(i,2)]-[0,0])<0.05
        tstable = i;
        break
    end
end
end

%Minimum Time Policy
function xdot = minimum_time(t,x)
%State-space matrix
A = [0,1;0,0];
B = [0;1];

if x(1)>=0
    if x(2)>-abs(sqrt(2*x(1)))
        u = -1;
    else
        u = 1;
    end
else
    if x(2)>abs(-sqrt(2*x(1)))
        u = -1;
    else
        u = 1;
    end
end

xdot=A*x+B*u;
end

%LQR Policy
function xdot = lqrfunc(t,x)
%State-space matrix
A = [0,1;0,0];
B = [0;1];

% LQR control
Q = 10*eye(2);
R = 100;

% Q = 100*eye(2);
% R = 10;
[K, S, E] = lqr(A, B, Q, R);
u = -K*x;

xdot=A*x+B*u;
end