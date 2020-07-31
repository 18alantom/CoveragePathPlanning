from app import app
from app import views
from flask import request

@app.route("/admin/dashboard")
def admin_dashboard():
	result=request.form
	return ("Hi")