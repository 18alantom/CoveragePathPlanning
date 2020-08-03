from flask import Markup
from flask import Flask
from flask_pymongo import pymongo
from app import app
import json
#from .temp import temp
#from app.run_coverage import coverage_algorithms
from cpp_algorithms import run_coverage

from flask import render_template, session, url_for, request, redirect, make_response, jsonify


CONNECTION_STRING = "mongodb+srv://test:test@cluster0.ae3zt.mongodb.net/test?retryWrites=true&w=majority"
client = pymongo.MongoClient(CONNECTION_STRING)
db = client.get_database('cluster0')
user_collection = pymongo.collection.Collection(db, 'user_collection')
global req1
global r
global coverage_path


@app.route("/", methods=['POST', 'GET'])
def index():
    '''if 'username' in session:
            return 'You are logged in as '+session['username']'''

    if request.method == 'POST':
        users = db.collection
        name = request.form.get('username')
        password = request.form.get('password')
        ex_user = users.find_one({'username': name})

        if ex_user is None:
            users.insert({'username': name, 'password': password})
            session['username'] = request.form.get('username')
            return redirect(url_for('shapefile'))

        '''else:
			return('The username already exists!')'''

    return render_template('index.html')


@app.route("/input", methods=['POST', 'GET'])
def input():

    return render_template('shapefile.html',)


@app.route("/input/hello", methods=['POST'])
def hello():
    global r
    req1 = request.get_json()
    res1 = make_response(jsonify(req1), 200)
    r = req1

    return res1

@app.route("/shapefile")
def shapefile():
    return render_template('shapefile.html')


@app.route("/shapefile/create-entry", methods=['POST'])
def create_entry():
    global r
    req = request.get_json()

    print(r)
    print(req)
    side = r['side']
    print(side)
    coverage_path = run_coverage(
        req, side=side)
    print(coverage_path)
    res = make_response(jsonify(coverage_path), 200)
    return res
