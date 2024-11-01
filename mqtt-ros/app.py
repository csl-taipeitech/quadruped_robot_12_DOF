from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import paho.mqtt.client as mqtt

# Initialize Flask app and enable CORS
app = Flask(__name__, static_url_path="", static_folder="static")
CORS(app)

# MQTT Broker Configuration
MQTT_BROKER = "172.18.0.2"
MQTT_PORT = 1883
MQTT_TOPIC = "pingpong/primitive"

# Define MQTT client and callbacks
mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}")

def on_publish(client, userdata, mid):
    """Callback for when a message is published."""
    print("Message Published")

# Set MQTT callbacks
mqtt_client.on_connect = on_connect
mqtt_client.on_publish = on_publish
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

# Button actions mapped to corresponding messages
button_actions = {
    "forward": "Forward",
    "backward": "Backward",
    "left": "Left",
    "right": "Right",
    "pause": "Pause",
    "accelerate_forward_backward": "Accelerate Forward/Backward",
    "decelerate_forward_backward": "Decelerate Forward/Backward",
    "accelerate_left_right": "Accelerate Left/Right",
    "decelerate_left_right": "Decelerate Left/Right",
}

# Routes
@app.route("/")
def index():
    """Serve the main HTML page."""
    return send_from_directory("static", "index.html")

@app.route("/send_message", methods=["POST"])
def send_message():
    """Endpoint to send a message based on the button ID."""
    data = request.json
    #print("Received data:", data)  # Log the received data
    button_id = data.get("button_id")

    # Publish message if button ID is valid
    if button_id in button_actions:
        message = button_actions[button_id]
        mqtt_client.publish(MQTT_TOPIC, message)
        return '', 200
    return '', 400

# Run the app
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
