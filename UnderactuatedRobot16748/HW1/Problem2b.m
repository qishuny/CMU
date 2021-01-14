clear all
close all
clc
%Problem 2 b
%Qishun Yu
%Parameters
m = 1;
g = 9.8;
l = 1;

b = 0.25;


% Problem 2b f function

% u = 2*m*g*l*sin(Y(1));
fb = @(t,Y) [Y(2);(2*m*g*l*sin(Y(1))-m*g*l*sin(Y(1))-b*Y(2))./(m*l*l)];


y1 = linspace(-2*pi,2*pi,30);
y2 = linspace(-6,6,30);
[x,y] = ndgrid(y1,y2);
u = zeros(size(x));
v = zeros(size(x));
t=0; 

for i = 1:numel(x)
    Yprime = fb(t,[x(i); y(i)]);
    u(i) = Yprime(1);
    v(i) = Yprime(2);
    
end
quiver(x,y,u,v,'r'); figure(gcf)
xlabel('theta')
ylabel('thetadot')
hold on

xIni = pi/4;
yIni = 0;



[ts,ys] = ode45(fb,[0,400],[xIni;yIni]);


workb = sum(2*m*g*l*sin(ys(1))*ys(2))
plot(ys(:,1),ys(:,2),'b')
plot(ys(1,1),ys(1,2),'bo') % starting point
plot(ys(end,1),ys(end,2),'ks') % ending point

