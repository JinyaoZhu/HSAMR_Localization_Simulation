function [parking_slot_out,temp_data] = parking_slot_detection(pose,running_state,measure_raw,dt)

persistent update_time;
if isempty(update_time)
    update_time = 0;
end

persistent parking_slot;
if isempty(parking_slot)
    parking_slot = zeros(4,4);
end

persistent parking_slot_index;
if isempty(parking_slot_index)
    parking_slot_index = 1;
end

persistent last_measurement;
if isempty(last_measurement)
    last_measurement = med_filter_31(measure_raw);
end

measure = med_filter_31(measure_raw);

filter_delay_cm = 2;

if update_time > 1.2
    switch running_state
        case 1
            if (measure - last_measurement)/dt > 200
                if parking_slot(4,parking_slot_index)~=0
                    parking_slot(1:2,parking_slot_index) = parking_slot(1:2,parking_slot_index) + 0.5*([pose(1)-filter_delay_cm+2;30]- parking_slot(1:2,parking_slot_index));
                else
                    parking_slot(1:2,parking_slot_index) = [pose(1)-filter_delay_cm+2;30];
                end
                update_time = 0;
            elseif (measure - last_measurement)/dt < -200
                if parking_slot(4,parking_slot_index)~=0
                    parking_slot(3:4,parking_slot_index) = parking_slot(3:4,parking_slot_index) + 0.5*([pose(1)-filter_delay_cm-2;30]-parking_slot(3:4,parking_slot_index));
                else
                    parking_slot(3:4,parking_slot_index) = [pose(1)-filter_delay_cm-2;30];
                end
                parking_slot_index = parking_slot_index+1;
                update_time = 0;
            end
            
        case 2
            parking_slot_index = 3;
            if (measure - last_measurement)/dt > 200
                if parking_slot(4,parking_slot_index)~=0
                    parking_slot(1:2,parking_slot_index) =  parking_slot(1:2,parking_slot_index)  + 0.5*([230;pose(2)-filter_delay_cm+2]- parking_slot(1:2,parking_slot_index) );
                else
                    parking_slot(1:2,parking_slot_index) = [230;pose(2)-filter_delay_cm+2];
                end
                parking_slot(3:4,parking_slot_index) = [230;108];
                update_time = 0;
            end
        case 4
            update_time = 0;
        case 5
            parking_slot_index = 4;
            if (measure - last_measurement)/dt < -200 && update_time > 5
                if parking_slot(4,parking_slot_index)~=0
                    parking_slot(3:4,parking_slot_index) =parking_slot(3:4,parking_slot_index) + 0.5*( [pose(1)+filter_delay_cm+2;95] -parking_slot(3:4,parking_slot_index));
                else
                    parking_slot(3:4,parking_slot_index) = [pose(1)+filter_delay_cm+2;95];
                end
                parking_slot(1:2,parking_slot_index) = [156;95];
                update_time = 0;
            end
        case 8
            parking_slot_index = 1;
        otherwise
    end
end

temp_data = (measure - last_measurement)/dt;
last_measurement = measure;
update_time = update_time+dt;
parking_slot_out = parking_slot;

end