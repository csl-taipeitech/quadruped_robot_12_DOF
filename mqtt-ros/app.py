from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import paho.mqtt.client as mqtt
import logging
from datetime import datetime

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize Flask app and enable CORS
app = Flask(__name__, static_url_path="", static_folder="static")
CORS(app)

# MQTT Broker Configuration
MQTT_BROKER = "172.18.0.2"
MQTT_PORT = 1883
MQTT_TOPIC = "pingpong/primitive"

# Define MQTT client and callbacks
mqtt_client = mqtt.Client()
mqtt_connected = False  # 追踪MQTT連接狀態

def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    global mqtt_connected
    if rc == 0:
        mqtt_connected = True
        logger.info("Connected to MQTT Broker!")
    else:
        mqtt_connected = False
        logger.error(f"Failed to connect to MQTT broker, return code {rc}")

def on_disconnect(client, userdata, rc):
    """Callback for when the client disconnects from the broker."""
    global mqtt_connected
    mqtt_connected = False
    logger.warning(f"Disconnected from MQTT broker with code: {rc}")

def on_publish(client, userdata, mid):
    """Callback for when a message is published."""
    logger.info(f"Message {mid} Published")

# Set MQTT callbacks
mqtt_client.on_connect = on_connect
mqtt_client.on_publish = on_publish
mqtt_client.on_disconnect = on_disconnect

# Try to connect to MQTT broker
try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
except Exception as e:
    logger.error(f"Failed to connect to MQTT broker: {e}")

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

@app.route("/health")
def health_check():
    """Endpoint to check server status."""
    return jsonify({
        'status': 'online',
        'mqtt_connected': mqtt_connected,
        'timestamp': datetime.now().isoformat()
    })

@app.route("/send_message", methods=["POST"])
def send_message():
    """Endpoint to send a message based on the button ID."""
    try:
        data = request.json
        logger.info(f"Received data: {data}")
        button_id = data.get("button_id")

        if not mqtt_connected:
            return jsonify({
                'status': 'error',
                'message': 'MQTT broker not connected'
            }), 503

        # Publish message if button ID is valid
        if button_id in button_actions:
            message = button_actions[button_id]
            result = mqtt_client.publish(MQTT_TOPIC, message)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                return jsonify({
                    'status': 'success',
                    'message': f'Command {message} sent successfully'
                })
            else:
                return jsonify({
                    'status': 'error',
                    'message': f'Failed to publish message: {result.rc}'
                }), 500
        else:
            return jsonify({
                'status': 'error',
                'message': f'Invalid button ID: {button_id}'
            }), 400

    except Exception as e:
        logger.error(f"Error processing request: {e}")
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

# Error handlers
@app.errorhandler(404)
def not_found(e):
    return jsonify({'error': 'Not found'}), 404

@app.errorhandler(500)
def server_error(e):
    return jsonify({'error': 'Internal server error'}), 500

# Run the app
if __name__ == "__main__":
    logger.info("Starting Flask server...")
    app.run(host="0.0.0.0", port=5000, debug=True)