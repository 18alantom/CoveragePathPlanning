#import geopandas
from flask import Flask
from flask_pymongo import pymongo
from app import app
from flask import render_template,session,url_for,request,redirect,make_response,jsonify
CONNECTION_STRING = "mongodb+srv://test:test@cluster0.ae3zt.mongodb.net/test?retryWrites=true&w=majority"
client = pymongo.MongoClient(CONNECTION_STRING)
db = client.get_database('cluster0')
user_collection = pymongo.collection.Collection(db, 'user_collection')
#mongo=pymongo(app)
global req1

@app.route("/",methods=['POST','GET'])
def index():
	
	if 'username' in session:
		return 'You are logged in as '+session['username']
	
	if 	request.method=='POST':
		users=db.collection
		name=request.form.get('username')
		password=request.form.get('password')
		ex_user=users.find_one({'username': name})

		if ex_user is None:
			users.insert({'username':name, 'password':password})
			session['username']=request.form.get('username')
			return redirect(url_for('input'))

		else:
			return('The username already exists!')

		
	return render_template('index.html')

@app.route("/input",methods=['POST','GET'])
def input():
    if request.method=='POST':
    	
    	return redirect(url_for('shconv_2'))
    	
    return render_template('input.html')



@app.route("/input/hello",methods=['POST'])
def hello():
	global roll
	req1=request.get_json()
	print(req1)
	roll=req1
	res1=make_response(jsonify(req1),200)
	return res1

@app.route("/shconv_2")
def shconv_2():

	return render_template('shconv_2.html')

@app.route("/shconv_2/create-entry",methods=['POST'])
def create_entry():
	req = request.get_json()
	print(req)
	res = make_response(jsonify(req), 200)
	return res


'''@app.route("/about")
def about():
	return 
		<form method ="POST" action="/create">
			 <input type="file" name="shapefile_image">
			 <input type="submit">
		</form>
	'''         

	        	




@app.route('/create',methods=['POST'])
def create():
	if 'shapefile_image' in request.files:
		x=db.files
		shapefile_image=request.files['shapefile_image']
		#x.save(shapefile_image.filename, shapefile_image)
		x.insert({'shapefile_image_name':shapefile_image})
	return 'Done!'    


@app.route("/test")
def test():
	return roll
