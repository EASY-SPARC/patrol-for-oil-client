import matlab.net.http.*
import matlab.net.http.field.*

request_str = '{"robot_id": "2", "xgrid": "17", "ygrid": "37", "robot_heading": "0", "lon": "[]", "lat": "[]"}';
request = RequestMessage( 'POST', ...
    [ContentTypeField( 'application/vnd.api+json' ), AcceptField('application/vnd.api+json')], ...
     request_str);
response = request.send('http://localhost:5000/robot_fb/');
