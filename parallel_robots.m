robot_id = [0, 1, 2];
robot_weights = [2.0, 0.1, 0.3, 0.2, 1;
    0.1, 2.0, 0.3, 0.2, 1;
    2.0, 0.1, 0.3, 0.2, 1];
robot_delay = [3, 3, 5];

parfor (i = 1:3)
    reactive_robot(robot_id(i), robot_weights(i, :), robot_delay(i));
end
