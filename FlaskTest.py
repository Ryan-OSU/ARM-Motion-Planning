from flask import Flask, render_template, request

app = Flask(__name__)

@app.route("/", methods=['GET', 'POST'])
def index():
    status = "Disabled"
    print(request.method)
    if request.method == 'POST':
        if request.form.get('Enable') == 'Enable':
                # pass
            status = "Enabled"
            print(status)
        elif  request.form.get('Disable') == 'Disable':
                # pass # do something else
            status = "Disabled"
            print(status)
        else:
                # pass # unknown
            return render_template('index.html', status)
    elif request.method == 'GET':
            # return render_template("index.html")
        print("No Post Back Call")
    return render_template('index.html', status_display = status)
       
    
if __name__ == '__main__':
    app.run()