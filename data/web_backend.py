# Our Backend for the App!
# Built with Flask

# Import Flask
from flask import Flask
import requests

# Create the application
app = Flask(__name__)
API_URL = "https://maps.googleapis.com/maps/api/place/findplacefromtext/json?inputtype=textquery&locationbias=ipbias&fields=formatted_address,name,rating&"

@app.route('/')
def hello():
    return "Hello World!"

# path parameters
@app.route('/<name>')
def personal_hello(name):
    return "Hello, " + name

# serving hello.html
@app.route('/fancy/<name>')
def some_page(name):
    return flask.render_template('hello.html', name=name)

# serving find.html
@app.route('/find', methods=['GET'])
def find():
    return flask.render_template('find.html')

# process query
@app.route('/process_query', methods=['POST'])
def process_query():
    data = flask.request.form
    location = data['some_location']
    print(location)

    requestString = formRequest(location)
    responses = makeGET(requestString)['candidates']
    return flask.render_template('find.html', responses=responses)

def formRequest(input):
    # TODO
    return API_URL + "key=" + readKey() + "&input=" + input # return some string

def makeGET(input):
    # TODO
    print('input' + input)

    response = requests.get(input)
    print(response)
    if response:
        return response.json()
    else:
        return "uh oh" # return some response

def readKey():
    f = open("secrets.txt", "r")
    contents = f.read()
    return contents.strip()

if __name__ == '__main__':
    app.run(debug=True)
