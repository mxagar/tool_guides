import os
from flask import Flask, request, render_template, jsonify
from flask_sqlalchemy import SQLAlchemy

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///site.db'
db = SQLAlchemy(app)

from models import TextPiece

with app.app_context():
    db.create_all()

@app.route('/', methods=['GET', 'POST'])
def home():
    if request.method == 'POST':
        content = request.form['content']
        text_piece = TextPiece(content=content)
        db.session.add(text_piece)
        db.session.commit()

    last_texts = TextPiece.query.order_by(TextPiece.id.desc()).limit(5).all()
    return render_template('index.html', texts=last_texts)

@app.route('/text/<int:text_id>', methods=['GET'])
def get_text(text_id):
    text_piece = TextPiece.query.get_or_404(text_id)
    return jsonify({'id': text_piece.id, 'content': text_piece.content})

if __name__ == '__main__':
    #db.create_all()
    app.run(debug=True)
