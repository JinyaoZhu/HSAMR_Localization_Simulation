function [x_hist,x_est_hist,t_hist,measurement_hist,temp_data_hist]=simulation(map,line_map,params,controlhandle)
real_time = true;
video = false;

%% *****************************Figure*****************************
disp('Initialing Figure...')

if video
    video_writer = VideoWriter('simulation','MPEG-4');
    open(video_writer);
end

h_graphic = gca;
axis equal;
grid;
xlabel('X [cm]');
ylabel('Y [cm]');
set(gcf,'Renderer','opengl');
%% **************************Initial Condition*********************
disp('Setting initial conditions...')
tstep = 0.03;
cstep = 0.03;
max_time = 50;
max_iter = max_time/cstep;
time = 0;
x_hist = [];
x_est_hist = [];
t_hist = [];
measurement_hist = [];

temp_data_hist = [];

x0 = [60; 45; 0]; %inital state
target_x = [120,80,0;
            200,45,0;
            200,100,pi/2];

x = x0;
E_x = x0;
parking_slot_out = zeros(4,4);

NUM_OF_PARTICLE = 200;

particles = zeros(4,NUM_OF_PARTICLE);
particles(1,:) = ones(1,NUM_OF_PARTICLE).*x0(1) + 5*(rand(1,NUM_OF_PARTICLE)-0.5)*10;
particles(2,:) = ones(1,NUM_OF_PARTICLE).*x0(2) + 5*(rand(1,NUM_OF_PARTICLE)-0.5)*10;
particles(3,:) = ones(1,NUM_OF_PARTICLE).*x0(3) + 1*(rand(1,NUM_OF_PARTICLE)-0.5)*0.1;
% particles(1,:) = rand(1,NUM_OF_PARTICLE)*params.MAP_X_MAX;
% particles(2,:) = rand(1,NUM_OF_PARTICLE)*params.MAP_Y_MAX;
% particles(3,:) = 2*pi*(rand(1,NUM_OF_PARTICLE)-0.5);
particles(4,:) = ones(1,NUM_OF_PARTICLE)./NUM_OF_PARTICLE;

E_x = x0;

light_sensor_meas = [0,0];
%% *****************************Simulation*************************
disp('Simulation running...');
for iter = 1:max_iter
    tic;
    timeint = time:tstep:time+cstep;
    if iter == 1
        PP = simPlot(x,particles,E_x,params,map,line_map,h_graphic);
        PP.UpdatePlot(x,particles,E_x,parking_slot_out,time);
        h_title = title(sprintf('iteration:%d, time:%4.2f',iter,time));
         pause;
    end
    
    %run simulation
    time = time+cstep;
       
   
    u = controlhandle(x,target_x,time,light_sensor_meas,params);
    light_sensor_meas = get_light_sensor(x,params,line_map);
     
    %differential equaltion
    [~,xsave] =ode45(@(t,s) carEOM(s,u,params),timeint,x);
    x = xsave(end,:)';
    x(3)=warp_to_pi(x(3));
    
    x_hist = [x_hist,x];
    
    % encoder measurement
    u_measure = u+params.ENCODER_NOISE*randn(2,1);

    [~,real_dist] = get_sensor(sensor_pos(x,params),map,params); %simulated measurements
    real_dist = real_dist+params.IR_NOISE*randn(4,1);
    measurement_hist = [measurement_hist,real_dist];
    z = measure_voltage(real_dist,params);
    
    % Particle Filter
%     [E_x,particles] = particlefilter(particles,u_measure,z,cstep,map,params,NUM_OF_PARTICLE);
%     [E_x,running_state] = complementary_filter(E_x,u_measure,light_sensor_meas,cstep,params,x);
    E_x = ekf(E_x,u_measure,real_dist,cstep,map,params);
    
    x_est_hist = [x_est_hist,E_x];
    t_hist = [t_hist,time];
    
    %parking slot detection
%     [parking_slot_out,temp_data] = parking_slot_detection(E_x,running_state,real_dist(2),cstep);
%     temp_data_hist = [temp_data_hist,temp_data];
    %plot robot
    PP.UpdatePlot(x,particles,E_x,parking_slot_out,time);
    
%     set(h_title, 'String', sprintf('iteration: %d, time: %4.2f s', iter, time));
    
    if  min(real_dist)<2
        disp('Hit obstacle!');
        break;
    end
    
    if video
        writeVideo(video_writer, getframe(h_graphic));
    end
    
    t = toc;
    
    sim_time_factor = t/cstep;
    
    set(h_title, 'String', sprintf('x:%3.1fcm, y:%3.1fcm, theta=%3.1fdegrees, time:%3.1f s, time factor:%2.1f',...
            E_x(1),E_x(2),E_x(3)*180/pi,time,sim_time_factor));
    
    if real_time && (t < cstep)
        pause(cstep-t);
    end
        
end

if video
    close(video_writer);
end

end
