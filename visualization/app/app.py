from flask import Flask, render_template, request, redirect, send_from_directory
from werkzeug.utils import secure_filename
import os
import roslibpy

app = Flask(__name__)
ros_client = roslibpy.Ros(host='localhost', port=9090)
set_source_mesh_service = roslibpy.Service(ros_client, '/localization/source/set/mesh', 'localization_msgs/String')
set_source_point_cloud_service = roslibpy.Service(ros_client, '/localization/source/set/point_cloud', 'localization_msgs/String')
set_destination_point_cloud_service = roslibpy.Service(ros_client, '/localization/destination/set/point_cloud', 'localization_msgs/String')
UPLOAD_FOLDER = '/home/richard/loca-app/docker/context/upload'

source = None
destination = None


@app.route("/")
def index():
    return render_template('index.html', data={"source": source, "destination": destination})


@app.route('/upload', methods=['POST'])
def upload_file():
    if request.method == 'POST':
        files = request.files.getlist("file")
        for file in files:
            if file:
                file.save(os.path.join(UPLOAD_FOLDER, secure_filename(file.filename)))
        global destination
        global source
        source = files[0].filename if files[0].filename else None
        destination = files[1].filename if files[1].filename else None
        # set_source_mesh_service.call(roslibpy.ServiceRequest({'data': source})) if source else None
        set_source_point_cloud_service.call(roslibpy.ServiceRequest({'data': source})) if source else None
        set_destination_point_cloud_service.call(roslibpy.ServiceRequest({'data': destination})) if destination else None
    return redirect('/')


@app.route('/favicon.ico')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'), 'favicon.png')


@app.route('/model/<model>')
def send_model(model):
    return send_from_directory(UPLOAD_FOLDER, model)


if __name__ == '__main__':
    ros_client.run()
    app.run(debug=True)
