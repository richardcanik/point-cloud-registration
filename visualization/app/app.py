from flask import Flask, render_template, request, redirect
from werkzeug.utils import secure_filename
import os

app = Flask(__name__)
UPLOAD_FOLDER = '/home/richard/loca-app/docker/context/upload'

destination = None


@app.route("/")
def index():
    return render_template('index.html', data={"destination": destination})


@app.route('/upload', methods=['POST'])
def upload_file():
    if request.method == 'POST':
        files = request.files.getlist("file")
        for file in files:
            if file:
                file.save(os.path.join(UPLOAD_FOLDER, secure_filename(file.filename)))
        global destination
        destination = files[1].filename if files[1].filename else None
    return redirect('/')


if __name__ == '__main__':
    app.run(debug=True)
