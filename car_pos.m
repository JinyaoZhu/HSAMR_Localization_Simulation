function pos = car_pos(state,params)



xt = state(1);
yt = state(2);
theta = state(3);
%[conner_1,...,conner_4,center]
car_shape = [params.ROBOT_LENGTH_X/2,-params.ROBOT_LENGTH_X/2,-params.ROBOT_LENGTH_X/2,params.ROBOT_LENGTH_X/2,params.ROBOT_LENGTH_X/2,0;
               params.ROBOT_LENGTH_Y/2,params.ROBOT_LENGTH_Y/2,-params.ROBOT_LENGTH_Y/2,-params.ROBOT_LENGTH_Y/2,params.ROBOT_LENGTH_Y/2,0;
               1,1,1,1,1,1];
           
T = [cos(theta),-sin(theta),xt;
     sin(theta),cos(theta),yt;
     0 ,0 ,1];
 
pos = T*car_shape;

pos = pos(1:2,:);

end