from flask import Flask, request

app = Flask(__name__)

@app.route("/", methods=["GET", "POST"])
def index():
    return "This is the default page"

@app.route("/upload_run_result", methods=["GET", "POST"])
def upload_run_results():
    if request.method == "POST":
        uploaded_text = request.data.decode("utf-8")    
        print("Run results: \n" + uploaded_text)
        return uploaded_text
    else:
        return "This endpoint doesn't support GET requests"
    
@app.route("/upload_run_data", methods=["GET", "POST"])
def upload_run_data():
    if request.method == "POST":
        uploaded_text = request.data.decode("utf-8")    
        print("Run data: \n" + uploaded_text)
        return uploaded_text
    else:
        return "This endpoint doesn't support GET requests"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=80, debug=True)
