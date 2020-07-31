#import geopandas
from flask import Flask
from flask_pymongo import pymongo
from app import app
from flask import render_template,session,url_for,request,redirect
CONNECTION_STRING = "mongodb+srv://test:test@cluster0.ae3zt.mongodb.net/test?retryWrites=true&w=majority"
client = pymongo.MongoClient(CONNECTION_STRING)
db = client.get_database('cluster0')
user_collection = pymongo.collection.Collection(db, 'user_collection')
#mongo=pymongo(app)


@app.route("/",methods=['POST','GET'])
def index():
	from flask import jsonify
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
	   result=request.form
	   return ({'result':result})	
	return render_template('input.html')


@app.route("/about")
def about():
	return '''
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
	from flask import jsonify
	myshpfile = geopandas.read_file('IND_cov.gri')
	myshpfile.to_file('myJson.geojson', driver='GeoJSON')
	#db.collection.insert_one({"name":"kk"})
	return(myshpfile)
