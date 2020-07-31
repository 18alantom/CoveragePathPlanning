from flask import Flask
from flask_pymongo import pymongo
from app import app
CONNECTION_STRING = "mongodb+srv://test:test@cluster0.ae3zt.mongodb.net/test?retryWrites=true&w=majority"
client = pymongo.MongoClient(CONNECTION_STRING)
db = client.get_database('cluster0')
user_collection = pymongo.collection.Collection(db, 'user_collection')