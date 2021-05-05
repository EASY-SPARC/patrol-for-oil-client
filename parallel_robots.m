import matlab.net.*
import matlab.net.http.*
r = RequestMessage;
uri = URI('http://localhost:5000/mission/robots_weights');
resp = send(r,uri);
weights = resp.Body.Data.weights;

n_robots = size(weights, 1);

robot_delay = [3, 3, 5];

parfor (i = 1:n_robots)
    reactive_robot(i, weights(i, :), robot_delay(i));
end
