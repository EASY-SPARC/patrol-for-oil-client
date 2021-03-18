import matlab.net.http.*
import matlab.net.http.field.*

request_str = '{"robot_id": "0", "xgrid": "25", "ygrid": "36", "robot_heading": "0", "lon": "[]", "lat": "[]"}';
request = RequestMessage( 'POST', ...
    [ContentTypeField( 'application/vnd.api+json' ), AcceptField('application/vnd.api+json')], ...
     request_str);
response = request.send('http://localhost:5000/robot_fb/');