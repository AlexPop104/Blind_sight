## *********************************************************
##
## File autogenerated for the pico_zense_camera package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 291, 'description': 'Depth distance confidance (0% keeps all, 100% drops every depth point with <100% confidence', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'depth_confidence_threshold', 'edit_method': '', 'default': 100, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 291, 'description': 'Optimise the camera for this depth range', 'max': 15.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'depth_range', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 291, 'description': 'RGB Resolution', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'rgb_resolution', 'edit_method': "{'enum_description': 'List of supported resolutions', 'enum': [{'srcline': 12, 'description': 'Max Resolution', 'srcfile': '/home/alex/Blind_sight/src/pico_zense_camera/cfg/pico_zense_dcam710.cfg', 'cconsttype': 'const char * const', 'value': '1920x1080', 'ctype': 'std::string', 'type': 'str', 'name': '1920x1080'}, {'srcline': 13, 'description': 'Default Resolution', 'srcfile': '/home/alex/Blind_sight/src/pico_zense_camera/cfg/pico_zense_dcam710.cfg', 'cconsttype': 'const char * const', 'value': '1280x720', 'ctype': 'std::string', 'type': 'str', 'name': '1280x720'}, {'srcline': 14, 'description': 'Depth Resolution', 'srcfile': '/home/alex/Blind_sight/src/pico_zense_camera/cfg/pico_zense_dcam710.cfg', 'cconsttype': 'const char * const', 'value': '640x480', 'ctype': 'std::string', 'type': 'str', 'name': '640x480'}, {'srcline': 15, 'description': 'Min Resolution', 'srcfile': '/home/alex/Blind_sight/src/pico_zense_camera/cfg/pico_zense_dcam710.cfg', 'cconsttype': 'const char * const', 'value': '640x360', 'ctype': 'std::string', 'type': 'str', 'name': '640x360'}]}", 'default': '1280x720', 'level': 0, 'min': '', 'type': 'str'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

pico_zense_dcam710_1920x1080 = '1920x1080'
pico_zense_dcam710_1280x720 = '1280x720'
pico_zense_dcam710_640x480 = '640x480'
pico_zense_dcam710_640x360 = '640x360'
