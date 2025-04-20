from flask import Flask

def create_app():
    app = Flask(__name__)
    app.config['UPLOAD_FOLDER'] = './static/uploads'
    app.config['ALLOWED_EXTENSIONS'] = {'mp4', 'mov', 'avi'}

    from app import routes
    app.register_blueprint(routes.bp)

    return app
