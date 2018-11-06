%% parameters
dt = 0.01;              %control peroid
L = 1.378;              %distance between front and rear wheel
dev = [0.003 0.001]';   %deviation of control input u
%%
%% variable initiation
x0 = [0 0 pi/4 0 0]';  % initiating state
x_h = zeros(5 , n+1);         % use to store state history
x_h(: , 1) = x0;

n = 1000;       % total number of steps
veh = forklift(L , dev , dt , x0);
%%
for i = 1:n
    if i < n/2
        u = [1 0]';
    else
        u = [1 pi/4]';
    end
    x_h(: , i+1) = veh(u);
    %u_h(: , i) = u;
end
plot(x_h(1 , :) , x_h(2 , :));
