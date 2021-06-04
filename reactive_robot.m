function response = reactive_robot(robot_id, weights, t_delay)
    import matlab.net.*
    import matlab.net.http.*
    import matlab.net.http.field.*
    
    while true
        % Retrieve kde, env_sensibility and robots positions/heading from server API
        r = RequestMessage;
        uri = URI('http://localhost:5000/mission/kde');
        resp = send(r,uri);
        kde = resp.Body.Data.kde;

        r = RequestMessage;
        uri = URI('http://localhost:5000/simulation/env_sensibility');
        resp = send(r,uri);
        dist_grid = resp.Body.Data.env_sensibility;

        r = RequestMessage;
        uri = URI('http://localhost:5000/mission/robots_pos');
        resp = send(r,uri);
        robots_pos = resp.Body.Data.robots_pos;
        heading = resp.Body.Data.robots_heading;

        aux_mask = kde < 0;

        % Matlab's different indexing starts with 1 instead of 0
        robots_pos = robots_pos + 1;

        neighbors = robots_pos(setdiff(1:end, robot_id), :);
        [target, mapvalue] = computeTargetMulti(robots_pos(robot_id, :), heading(robot_id), kde, neighbors, dist_grid, weights);
        if norm(target - robots_pos(robot_id, :)) > 0
            % A*
            GoalRegister = int8(zeros(size(aux_mask)));
            GoalRegister(target(2), target(1)) = 1;
            for k = 1:size(neighbors, 1)
               aux_mask(neighbors(k, 2), neighbors(k, 1)) = 1;
            end
            result = ASTARPATH(robots_pos(robot_id, 1), robots_pos(robot_id, 2), aux_mask, GoalRegister, 1);
            if size(result, 1) > 1
                move = [result(end - 1, 2) - robots_pos(robot_id, 1), result(end - 1, 1) - robots_pos(robot_id, 2)];

                robots_pos(robot_id, :) = robots_pos(robot_id, :) + move;
                heading(robot_id) = atan2(move(2), move(1));
            end
        else 
            disp(['Robot ', num2str(robot_id), ' stopped']);
            disp(mapvalue);
        end


        % Publish robot computed position and heading to server API
        xgrid = robots_pos(robot_id, 1) - 1;
        ygrid = robots_pos(robot_id, 2) - 1;
        request_str = ['{"robot_id": "', num2str(robot_id), '", "xgrid": "', num2str(xgrid), '", "ygrid": "', num2str(ygrid), '", "robot_heading": "', num2str(heading(robot_id)) , '", "lon": "[]", "lat": "[]"}'];
        request = RequestMessage( 'POST', ...
            [ContentTypeField( 'application/vnd.api+json' ), AcceptField('application/vnd.api+json')], ...
             request_str);
        response = request.send('http://localhost:5000/robot_fb/');
        
        pause(t_delay);
    end
end

function [target, mapvalue] = computeTargetMulti(pos, heading, grid, neighbors, dist_grid, weights)
    max_value = 0;
    target = pos;
    max_heading = 0;
    mapvalue = zeros(size(grid));
    
    kappa = weights(1);
    omega_c = weights(2);
    omega_s = weights(3);
    omega_d = weights(4);
    omega_n = weights(5);
    
    for i = 1:size(grid, 2)
        for j = 1:size(grid, 1)
            if grid(j, i) < 0 % out of border conditions
                continue;
            else
                current = [i, j];
                if any(current ~= pos)
                    move = current - pos;
                    distance = norm(move);
                    new_heading = atan2(move(2), move(1));

                    distance_nearest_neigh = Inf;
                    for k = 1:size(neighbors, 1)
                        distance_neigh = norm(current - neighbors(k, :));
                        if (distance_neigh < distance_nearest_neigh)
                            distance_nearest_neigh = distance_neigh;
                        end
                    end
                    
                    if grid(j, i)>0
                        mapvalue(j,i) = max(kappa + omega_c*grid(j, i) + omega_s*dist_grid(j ,i) - omega_d * distance + omega_n * distance_nearest_neigh, 0);
                    else
                        mapvalue(j,i) = 0;
                    end

                    value = mapvalue(j,i);
                    if (value > max_value) || (value == max_value && abs(new_heading - heading) <= abs(max_heading - heading))
                        target = current;
                        max_value = value;
                        max_heading = new_heading;
                    end
                end
            end
        end
    end
    if (max_value == 0)
        target = pos;
    end
end