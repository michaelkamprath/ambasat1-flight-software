import json
from bottle import route, get, post, run, request, static_file

DATA_FILE_DIR = '/run/telemetry'
DATE_FILE_EXTENSION = 'jsonl'

def clean_project_name(project):
    # since project is used a a file name, remove any symbols 
    return "".join(filter(str.isalnum, project))
         
@get('/telemetry/<project>')
def return_telemetry(project):
    cleaned_project = clean_project_name(project)
    print('Downloading telemetry file for \'{0}\''.format(cleaned_porject))
    return static_file('{0}.{1}'.format(cleaned_project, DATE_FILE_EXTENSION), root=DATA_FILE_DIR)
        

@post('/telemetry/<project>')
def ingest_telemetry(project):
    cleaned_project = clean_project_name(project)
    telemetry_data = request.json
    if telemetry_data is not None:
        filename = '{0}/{1}.{2}'.format(DATA_FILE_DIR, cleaned_project, DATE_FILE_EXTENSION)
        with open(filename, 'a') as f:
            f.write(json.dumps(telemetry_data)+'\n')
        print('Received telemetry for \'{0}\': {1}'.format(cleaned_project, telemetry_data))
    return 'telemetry accepted for {0}\n'.format(cleaned_project)


run(host='0.0.0.0', port=8000, debug=True)