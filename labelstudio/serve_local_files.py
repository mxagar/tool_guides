from flask import Flask, send_from_directory
from flask_cors import CORS

SERVER_DIRECTORY = 'C:/Users/Msagardi/git_repositories/tool_guides/labelstudio/data/'

app = Flask(__name__)
CORS(app)  # This will enable CORS for all routes

@app.route('/<path:path>')
def serve_file(path):
    return send_from_directory(SERVER_DIRECTORY, path)

if __name__ == '__main__':
    app.run(port=8000)
