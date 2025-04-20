from flask import Blueprint, request, render_template, jsonify
import os
from werkzeug.utils import secure_filename
from app.video_processing import process_video

bp = Blueprint('main', __name__)

@bp.route('/')
def index():
    return render_template('index.html')

@bp.route('/upload', methods=['POST'])
def upload_video():
    if 'file' not in request.files:
        return jsonify({"error": "No file uploaded"})

    file = request.files['file']

    if file.filename == '':
        return jsonify({"error": "Empty filename"})

    if file and allowed_file(file.filename):
        filename = secure_filename(file.filename)
        filepath = os.path.join('static/uploads', filename)
        file.save(filepath)
        abs_path = os.path.abspath(filepath)

        result = process_video(abs_path)
        return jsonify(result)

    return jsonify({"error": "Unsupported file format"})

def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in {'mp4', 'mov', 'avi', 'webm'}

@bp.route('/viewer')
def viewer():
    return render_template("viewer.html")
