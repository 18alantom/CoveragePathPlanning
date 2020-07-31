import os
from flask import Flask
from flask_pymongo import PyMongo



app=Flask(__name__)
app.secret_key = os.urandom(24)


from app import views
from app import admin_views
#from app import db