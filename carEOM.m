function x_dot =carEOM(state,u,params)
W = params.CAR_WIDTH;
Rl = params.CAR_WHEEL_L;
Rr = params.CAR_WHEEL_R;

theta = state(3);

J = [cos(theta) cos(theta); 
     sin(theta) sin(theta);
     2/W          -2/W]*...
    [Rr/2 0;
     0   Rl/2];
x_dot = J*u;
end