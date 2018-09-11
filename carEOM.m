function x_dot =carEOM(state,u,params)
W = params.CAR_WIDTH;
Rl = params.CAR_WHEEL_L;
Rr = params.CAR_WHEEL_R;

theta = state(3);

V = [Rr/2,Rl/2;Rr/W,-Rl/W]*u;

J = [cos(theta) 0; 
     sin(theta) 0;
     0          1];
x_dot = J*V;
end