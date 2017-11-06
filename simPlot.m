classdef simPlot < handle
    properties(SetAccess = public)
        k = 0;
        time = 0;
        num_state = 3;
        state;%system state
        params;%parameters
        pos_car; %position of the car
        pos_sensor;
        state_hist = [];
        est_state_hist = [];
        map;
    end
    
    properties(SetAccess = private)
        h_graphic;
        h_map;
        h_car;%handle of the car;
        h_car_center;
        h_state_hist;
        h_est_state_hist;
        h_sensor_1;
        h_sensor_2;
        h_sensor_3;
        h_sensor_4;
        h_particles;
        h_est_center;
        h_tracking_line;
        h_parking_slot_1;
        h_parking_slot_2;
        h_parking_slot_3;
        h_parking_slot_4;
    end
    
    methods
        %Constructor
        function P = simPlot(state,particles,est_state,params,map,line_map,h_graphic)
            P.state = state;
            P.map = map;
            P.params = params;
            P.pos_car = car_pos(state,params);
            P.pos_sensor = sensor_pos(state,params);
            P.h_graphic = h_graphic;
            P.state_hist = [P.state_hist,state];
            P.est_state_hist = [P.est_state_hist,est_state];
            
            hold(P.h_graphic,'on');
            
            P.h_tracking_line =  plot([30,210,210,180,180,60,60,30,30,210],...
                [45,45,105,105,75,75,105,105,45,45],'Color',[0.2 0.2 0.2]+0.6,'LineWidth',6);
            
            P.h_state_hist = plot(P.state_hist(1,:),P.state_hist(2,:),'b');
            P.h_est_state_hist = plot(P.est_state_hist(1,:),P.est_state_hist(2,:),'r');
            %             P.h_map = plot([0,P.params.MAP_X_MAX,P.params.MAP_X_MAX,0,0],...
            %                 [0,0,P.params.MAP_Y_MAX,P.params.MAP_Y_MAX,0],'black','LineWidth',2);
            x = 1:1:P.params.MAP_X_MAX;
            y = 1:1:P.params.MAP_Y_MAX;
            [X,Y] = meshgrid(x,y);
            contour(X,Y,P.map','Color',[0 0 0]);
            
            %             contour(X,Y,line_map','g');
           
            % P.h_map = image([0 P.params.MAP_X_MAX], [0 P.params.MAP_Y_MAX], P.map);
            
           
%             P.h_particles = scatter(particles(1,:),particles(2,:),particles(4,:).*length(particles(4,:))*50,'Linewidth',0.2,...
%                 'MarkerEdgeColor',[0 0.8 .2]);
            
            P.h_est_center = scatter(est_state(1),est_state(2),'MarkerEdgeColor',[0.8 0.1 0]);
            
            P.h_car = plot(P.pos_car(1,1:5),P.pos_car(2,1:5),'blue','Linewidth',1);
            P.h_car_center = scatter(P.pos_car(1,6),P.pos_car(2,6),'blue','Linewidth',1);
            
            [endpoints,~] = get_sensor(P.pos_sensor,P.map);
            P.h_sensor_1 =  plot([P.pos_sensor(1,1),endpoints(1,1)],[P.pos_sensor(2,1),endpoints(2,1)],'--','Color',[1,0,0],'Linewidth',0.5);
            P.h_sensor_2 =  plot([P.pos_sensor(1,2),endpoints(1,2)],[P.pos_sensor(2,2),endpoints(2,2)],'--','Color',[1,0,0],'Linewidth',0.5);
            P.h_sensor_3 =  plot([P.pos_sensor(1,3),endpoints(1,3)],[P.pos_sensor(2,3),endpoints(2,3)],'--','Color',[1,0,0],'Linewidth',0.5);
            P.h_sensor_4 =  plot([P.pos_sensor(1,4),endpoints(1,4)],[P.pos_sensor(2,4),endpoints(2,4)],'--','Color',[1,0,0],'Linewidth',0.5);
            
            P.h_parking_slot_1 = rectangle('Position',[1,1,1,1],'FaceColor',[0 .5 .5]);
            P.h_parking_slot_2 = rectangle('Position',[1,1,1,1],'FaceColor',[0 .5 .5]);
            P.h_parking_slot_3 = rectangle('Position',[1,1,1,1],'FaceColor',[0 .5 .5]);
            P.h_parking_slot_4 = rectangle('Position',[1,1,1,1],'FaceColor',[0 .5 .5]);
            
            legend([P.h_state_hist,P.h_est_state_hist,P.h_sensor_1],'real trajectory','estimated trajectory','IR rays');

            axis([-20,P.params.MAP_X_MAX+40,-20,P.params.MAP_Y_MAX+40]);
            
            hold(P.h_graphic,'off');
        end
        
        %Update robot's state
        function UpdateState(P,state,est_state,time)
            P.state = state;
            P.time = time;
            P.state_hist = [P.state_hist,state];
            P.est_state_hist = [P.est_state_hist,est_state];
        end
        
        %Update positon the car 
        function UpdatePosition(P)
            P.pos_car = car_pos(P.state,P.params);
            P.pos_sensor = sensor_pos(P.state,P.params);
        end
        
        %Update  plot
        function UpdatePlot(P,state,particles,est_state,parking_slot,time)
            P.UpdateState(state,est_state,time);
            P.UpdatePosition()
            
            set(P.h_car,'XData',P.pos_car(1,1:5),...
                'YData',P.pos_car(2,1:5));
            
            set(P.h_car_center,'XData',P.pos_car(1,6),...
                'YData',P.pos_car(2,6));
            
            [endpoints,~] = get_sensor(P.pos_sensor,P.map);
            
            set(P.h_sensor_1,'XData',[P.pos_sensor(1,1),endpoints(1,1)],...
                'YData',[P.pos_sensor(2,1),endpoints(2,1)]);
            set(P.h_sensor_2,'XData',[P.pos_sensor(1,2),endpoints(1,2)],...
                'YData',[P.pos_sensor(2,2),endpoints(2,2)]);
            set(P.h_sensor_3,'XData',[P.pos_sensor(1,3),endpoints(1,3)],...
                'YData',[P.pos_sensor(2,3),endpoints(2,3)]);
            set(P.h_sensor_4,'XData',[P.pos_sensor(1,4),endpoints(1,4)],...
                'YData',[P.pos_sensor(2,4),endpoints(2,4)]);
            
%              set(P.h_particles,'XData',particles(1,:),...
%                                 'YData',particles(2,:),...
%                             'SizeData',particles(4,:).*length(particles(4,:))*30);
            
            set(P.h_est_center,'XData',est_state(1),'YData',est_state(2));
            
            set(P.h_state_hist,'XData',P.state_hist(1,:),'YData',P.state_hist(2,:));
            set(P.h_est_state_hist,'XData',P.est_state_hist(1,:),'YData',P.est_state_hist(2,:));
            
            
            if(parking_slot(4,1) ~= 0)
                set(P.h_parking_slot_1,'Position',...
                [parking_slot(1,1),parking_slot(2,1)-15,parking_slot(3,1)-parking_slot(1,1),15]);
            end
            if(parking_slot(4,2) ~= 0)
                set(P.h_parking_slot_2,'Position',...
                [parking_slot(1,2),parking_slot(2,2)-15,parking_slot(3,2)-parking_slot(1,2),15]);
            end
            if(parking_slot(4,3) ~= 0)
                set(P.h_parking_slot_3,'Position',...
                [parking_slot(1,3),parking_slot(2,3),15,parking_slot(4,3)-parking_slot(2,3)]);
            end
            if(parking_slot(4,4) ~= 0)
                set(P.h_parking_slot_4,'Position',...
                [parking_slot(3,4),parking_slot(4,4),parking_slot(1,4)-parking_slot(3,4),15]);
            end
            
            drawnow;
        end
    end
    
end