import matlab.net.*
import matlab.net.http.*
import matlab.net.http.field.*

% Retrieve kde and env_sensibility from server API
r = RequestMessage;
uri = URI('http://localhost:5000/kde/');
resp = send(r,uri);
kde = resp.Body.Data.kde;

r = RequestMessage;
uri = URI('http://localhost:5000/env_sensibility/');
resp = send(r,uri);
env_sensibility = resp.Body.Data.env_sensibility;

%figure(1)
%imagesc(kde)
%set(gca, 'YDir', 'normal')
%caxis([-1, 5])
%colormap jet
%colorbar

%figure(2)
%imagesc(env_sensibility)
%set(gca, 'YDir', 'normal')
%caxis([-1, 5])
%colormap jet
%colorbar




% Publish robot computed position to server API
request = RequestMessage( 'POST', ...
    [ContentTypeField( 'application/vnd.api+json' ), AcceptField('application/vnd.api+json')], ...
    '{"xgrid": "21", "ygrid": "38", "lon": "[]", "lat": "[]"}' );
response = request.send('http://localhost:5000/robot_fb/');