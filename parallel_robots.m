import matlab.net.*
import matlab.net.http.*
r = RequestMessage;
uri = URI('http://localhost:5000/mission/robots_weights');
resp = send(r,uri);
weights = resp.Body.Data.weights;
n_robots = size(weights, 1);

robot_delay = [3, 5, 2, 4, 3, 5, 6, 3, 3, 4];

parfor (i = 1:n_robots)
    reactive_robot(i, weights(i, :), robot_delay(i));
end
