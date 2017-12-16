function [E_x,new_particles] = particlefilter(particles,u,z,dt,map,params,num_of_particle)

for i = 1 : num_of_particle
    particles(1:3,i) = carEOM(particles(1:3,i),u,params)*dt + particles(1:3,i);
    
    if particles(1,i) > params.MAP_X_MAX
        particles(1,i) = params.MAP_X_MAX;
    end
    if particles(2,i) > params.MAP_Y_MAX
        particles(2,i) = params.MAP_Y_MAX;
    end
    if particles(1,i) < params.MAP_XY_MIN
        particles(1,i) = params.MAP_XY_MIN;
    end
    if particles(2,i) < params.MAP_XY_MIN
        particles(2,i) = params.MAP_XY_MIN;
    end
     particles(3,i) = warp_to_pi( particles(3,i));
     
    [~,predicted_z] = get_sensor(sensor_pos(particles(1:3,i),params),map,params);
    predicted_z = predicted_z+ params.IR_NOISE*randn(4,1);
    predicted_z = measure_voltage(predicted_z,params);
    e = max(0.01,norm(z - predicted_z));
%     e = norm(z - predicted_z);
    particles(4,i) = 1/e;% update weights
end
particles(4,:) = particles(4,:)./sum(particles(4,:)); %normalize the weigths

%get E(x)
E_x = zeros(3,1);
cos_theta = 0;
sin_theta = 0;
for i = 1:num_of_particle
    E_x(1:2,1) = E_x(1:2,1) + particles(4,i)*particles(1:2,i);
    %Compute using quadrature components to prevent problems at +/- PI
    cos_theta =  cos_theta + particles(4,i)*cos(particles(3,i));
    sin_theta =  sin_theta + particles(4,i)*sin(particles(3,i));
end
E_x(3,1) = atan2(sin_theta,cos_theta);
%resample
new_particles = particles;

temp = cumsum(particles(4,:));
for i = 1 : num_of_particle
    if i < round(num_of_particle*0.95)
        new_particles(:,i) = particles(:,find(rand <= temp,1));
    else
        new_particles(:,i) = [5*2*(rand(2,1)-0.5)+E_x(1:2);0.1*(rand(1,1)-0.5)+E_x(3);0.001];
        new_particles(3,i) = warp_to_pi( new_particles(3,i));
    end
end

new_particles(4,:) = new_particles(4,:)./sum(new_particles(4,:)); %normalize the weigths

end