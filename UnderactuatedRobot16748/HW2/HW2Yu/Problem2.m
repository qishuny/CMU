clear all
close all
clc

%Problem 2
%Qishun Yu

Grid = [
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 ];
f1 = figure(1); clf
set(f1,'units','normalized','outerposition',[0 0 1 1])
J = -50*ones(size(Grid)); 
J(Grid==0) = -1; 
J(8,9) = 100; 

v = value_iteration(J,Grid);
Policy(v,Grid);

figure(f1)

function v = value_iteration(R,World)

vprev = zeros(size(World));
v = -100*ones(size(World));
v(World==0) = R(World==0);
colormap(jet)
gamma = 0.98;
xIND = find(World == 0);
iteration = 0;

hImageV = imagesc(v);
axis equal
axis tight
set(gca,'Xtick',[], 'Ytick',[])

iteration_limit = 200;
while ~isequal(v,vprev) && iteration < iteration_limit
    vprev = v;
    for i = 1:numel(xIND)
        [~,bestPayoff] = policy(xIND(i),v);
        v(xIND(i)) = gamma*( R(xIND(i)) + bestPayoff );
    end
    iteration = iteration+1;
    set(hImageV,'cdata',v);
    drawnow
end
end

function [bestMove,bestPayoff] = policy(index,V_hat)

[Iy,Ix] = ind2sub(size(V_hat),index);
moves = [1,0; 0,1; -1,0; 0,-1; 0,0];
bestPayoff = -100; 
for k = [1,2,3,4]
    move = [moves(k,1),moves(k,2)];
    
    payoff = V_hat(Iy+move(1),Ix+move(2))+1;     
    if payoff > bestPayoff
        bestPayoff = payoff;
        bestMove = move;
    end
end
end

function [dx,dy] = Policy(v,World)

indX = find(World == 0);
imagesc(v);
axis equal
set(gca,'Xtick',[], 'Ytick',[])

[x,y] = meshgrid(1:size(World,2),1:size(World,1));
dx = zeros(size(x));
dy = zeros(size(y));

for i = 1:numel(indX)
    [Iy,Ix] = ind2sub(size(v),indX(i));
    [bestMove,~] = policy(indX(i),v);
    dx(Iy,Ix) = bestMove(1);
    dy(Iy,Ix) = bestMove(2);
end
hold on; 
quiver(x,y,dy,dx,0.5,'k');
end

