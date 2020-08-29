import json
from bottle import route, get, post, run, template, request, static_file

DATA_FILE_DIR = '/run/telemetry'

def clean_project_name(project):
    # since project is used a a file name, remove any symbols 
    return "".join(filter(str.isalnum, project))
         
@get('/telemetry/<project>')
def return_telemtry(project):
    cleaned_porject = clean_project_name(project)
    print('Downloading telemetry file for \'{0}\''.format(cleaned_porject))
    return static_file('{0}.data'.format(cleaned_porject), root=DATA_FILE_DIR)
        

@post('/telemetry/<project>')
def ingest_telemetry(project):
    cleaned_porject = clean_project_name(project)
    telemetry_data = request.json
    if telemetry_data is not None:
        filename = '{0}/{1}.data'.format(DATA_FILE_DIR,cleaned_porject)
        with open(filename, 'a') as f:
            f.write(json.dumps(telemetry_data)+'\n')
        print('Received telemetry for \'{0}\': {1}'.format(cleaned_porject,telemetry_data))
    return '<p>telemetry accepted for {{project}}</p>\n'


run(host='0.0.0.0', port=8000, debug=True)