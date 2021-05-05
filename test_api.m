import matlab.net.*
import matlab.net.http.*
r = RequestMessage;
uri = URI('http://localhost:5000/mission/kde');
resp = send(r,uri);
kde = resp.Body.Data.kde;

figure(1)
imagesc(kde)
set(gca, 'YDir', 'normal')
caxis([-1, 5])
colormap jet
colorbar

r = RequestMessage;
uri = URI('http://localhost:5000/simulation/env_sensibility');
resp = send(r,uri);
env_sensibility = resp.Body.Data.env_sensibility;

figure(2)
title('Gaussian potential field')
imagesc(env_sensibility)
set(gca, 'YDir', 'normal')
caxis([-1, 5])
colormap jet
colorbar