import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import json

HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Quadruped Command Center</title>
    <style>
        :root {
            --bg-color: #0b0f19;
            --card-bg: rgba(20, 25, 40, 0.6);
            --card-border: rgba(255, 255, 255, 0.1);
            --text-main: #e2e8f0;
            --text-muted: #94a3b8;
            --accent-blue: #38bdf8;
            --accent-purple: #c084fc;
            --accent-red: #fb7185;
            --accent-green: #34d399;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
        }

        body {
            background-color: var(--bg-color);
            background-image: 
                radial-gradient(at 0% 0%, rgba(56, 189, 248, 0.15) 0px, transparent 50%),
                radial-gradient(at 100% 100%, rgba(192, 132, 252, 0.15) 0px, transparent 50%);
            color: var(--text-main);
            min-height: 100vh;
            padding: 2rem;
            display: flex;
            flex-direction: column;
            align-items: center;
        }

        header {
            text-align: center;
            margin-bottom: 3rem;
        }

        h1 {
            font-size: 2.5rem;
            font-weight: 800;
            background: linear-gradient(to right, var(--accent-blue), var(--accent-purple));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 0.5rem;
            letter-spacing: -0.05em;
        }

        p.subtitle {
            color: var(--text-muted);
            font-size: 1.1rem;
        }

        .step-control {
            margin-top: 1.5rem;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 0.75rem;
            background: var(--card-bg);
            padding: 0.75rem 1.5rem;
            border-radius: 12px;
            border: 1px solid var(--card-border);
            backdrop-filter: blur(12px);
            display: inline-flex;
        }

        .step-control label {
            color: var(--text-main);
            font-size: 0.95rem;
            font-weight: 500;
        }

        .step-control input {
            background: rgba(0, 0, 0, 0.2);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: var(--accent-blue);
            padding: 0.5rem 0.75rem;
            border-radius: 8px;
            font-size: 1rem;
            font-weight: bold;
            width: 90px;
            text-align: center;
            outline: none;
            transition: all 0.2s;
        }

        .step-control input:focus {
            border-color: var(--accent-blue);
            box-shadow: 0 0 10px rgba(56, 189, 248, 0.2);
        }

        .dashboard {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 1.5rem;
            width: 100%;
            max-width: 1200px;
        }

        .panel {
            background: var(--card-bg);
            backdrop-filter: blur(12px);
            -webkit-backdrop-filter: blur(12px);
            border: 1px solid var(--card-border);
            border-radius: 16px;
            padding: 1.5rem;
            box-shadow: 0 4px 30px rgba(0, 0, 0, 0.1);
        }

        .panel h2 {
            font-size: 1.25rem;
            font-weight: 600;
            margin-bottom: 1.25rem;
            color: var(--text-main);
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }

        .panel h2::before {
            content: '';
            display: block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
        }

        .panel.sys h2::before { background-color: var(--accent-red); }
        .panel.linear h2::before { background-color: var(--accent-blue); }
        .panel.lateral h2::before { background-color: var(--accent-green); }
        .panel.posture h2::before { background-color: var(--accent-purple); }

        .btn-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 0.75rem;
        }

        .btn-full {
            grid-column: 1 / -1;
        }

        button {
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: var(--text-main);
            padding: 0.875rem 1rem;
            border-radius: 10px;
            font-size: 0.95rem;
            font-weight: 500;
            cursor: pointer;
            transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1);
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            gap: 0.25rem;
        }

        button span {
            font-size: 0.75rem;
            color: var(--text-muted);
            font-weight: 400;
        }

        button:hover {
            background: rgba(255, 255, 255, 0.08);
            border-color: rgba(255, 255, 255, 0.2);
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        }

        button:active {
            transform: translateY(0);
        }

        /* Specialized Buttons */
        .btn-danger {
            background: rgba(251, 113, 133, 0.1);
            border-color: rgba(251, 113, 133, 0.3);
            color: var(--accent-red);
        }
        .btn-danger:hover {
            background: rgba(251, 113, 133, 0.2);
            border-color: var(--accent-red);
            box-shadow: 0 0 15px rgba(251, 113, 133, 0.3);
        }

        .btn-primary {
            background: rgba(56, 189, 248, 0.1);
            border-color: rgba(56, 189, 248, 0.3);
            color: var(--accent-blue);
        }
        .btn-primary:hover {
            background: rgba(56, 189, 248, 0.2);
            border-color: var(--accent-blue);
            box-shadow: 0 0 15px rgba(56, 189, 248, 0.3);
        }

        /* Toast Notification */
        #toast {
            position: fixed;
            bottom: 2rem;
            right: 2rem;
            background: var(--card-bg);
            backdrop-filter: blur(12px);
            border: 1px solid var(--accent-blue);
            color: white;
            padding: 1rem 1.5rem;
            border-radius: 12px;
            transform: translateY(150%);
            transition: transform 0.3s cubic-bezier(0.4, 0, 0.2, 1);
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            z-index: 1000;
            display: flex;
            align-items: center;
            gap: 0.75rem;
        }

        #toast.show {
            transform: translateY(0);
        }

        .status-dot {
            width: 8px;
            height: 8px;
            background-color: var(--accent-blue);
            border-radius: 50%;
            box-shadow: 0 0 8px var(--accent-blue);
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0% { box-shadow: 0 0 0 0 rgba(56, 189, 248, 0.7); }
            70% { box-shadow: 0 0 0 6px rgba(56, 189, 248, 0); }
            100% { box-shadow: 0 0 0 0 rgba(56, 189, 248, 0); }
        }
    </style>
</head>
<body>

    <header>
        <h1>Quadruped Command Center</h1>
        <p class="subtitle">Real-time ROS2 Web Interface</p>
        
        <div class="step-control">
            <label for="step-input">Steps (0 = Infinity):</label>
            <input type="number" id="step-input" value="0" min="0" max="1000">
        </div>
    </header>

    <div class="dashboard">
        <!-- System Controls -->
        <div class="panel sys">
            <h2>System Control</h2>
            <div class="btn-grid">
                <button class="btn-primary btn-full" onclick="sendCommand('ZERO')">
                    ZERO
                    <span>Reset / Home Position</span>
                </button>
                <button class="btn-danger btn-full" onclick="sendCommand('ROBOTOFF')">
                    ROBOT OFF
                    <span>Safe Shutdown Sequence</span>
                </button>
            </div>
        </div>

        <!-- Linear Movement -->
        <div class="panel linear">
            <h2>Linear Gaits</h2>
            <div class="btn-grid">
                <button onclick="sendCommand('TROT_FORWARD')">Trot Fwd</button>
                <button onclick="sendCommand('TROT_BACKWARD')">Trot Bwd</button>
                
                <button onclick="sendCommand('WALK_FORWARD')">Walk Fwd</button>
                <button onclick="sendCommand('WALK_BACKWARD')">Walk Bwd</button>
                
                <button onclick="sendCommand('WAVE_FORWARD')">Wave Fwd</button>
                <button onclick="sendCommand('WAVE_BACKWARD')">Wave Bwd</button>
            </div>
        </div>

        <!-- Lateral Movement -->
        <div class="panel lateral">
            <h2>Lateral & Turning</h2>
            <div class="btn-grid">
                <button onclick="sendCommand('TURN_LEFT')">Turn Left</button>
                <button onclick="sendCommand('TURN_RIGHT')">Turn Right</button>
                
                <button onclick="sendCommand('STRAFE_LEFT')">Strafe Left</button>
                <button onclick="sendCommand('STRAFE_RIGHT')">Strafe Right</button>
            </div>
        </div>

        <!-- Body Postures -->
        <div class="panel posture">
            <h2>Body Postures</h2>
            <div class="btn-grid">
                <button onclick="sendCommand('BODY_HEAVE')">
                    Heave
                    <span>Vertical Up/Down</span>
                </button>
                <button onclick="sendCommand('BODY_ROLL')">
                    Roll
                    <span>Lateral Sway</span>
                </button>
                <button onclick="sendCommand('BODY_PITCH')">
                    Pitch
                    <span>Longitudinal Sway</span>
                </button>
                <button onclick="sendCommand('BODY_CIRCLE')">
                    Circle
                    <span>Rotational Sway</span>
                </button>
            </div>
        </div>
    </div>

    <div id="toast">
        <div class="status-dot"></div>
        <span id="toast-msg">Command sent</span>
    </div>

    <script>
        let toastTimeout;

        function showToast(cmd, step) {
            const toast = document.getElementById('toast');
            const msg = document.getElementById('toast-msg');
            msg.textContent = `Published: ${cmd} ${step}`;
            
            toast.classList.add('show');
            clearTimeout(toastTimeout);
            toastTimeout = setTimeout(() => {
                toast.classList.remove('show');
            }, 2500);
        }

        async function sendCommand(cmd) {
            try {
                const stepInput = document.getElementById('step-input');
                const stepVal = parseInt(stepInput.value) || 0;

                const response = await fetch('/api/command', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ cmd: cmd, step: stepVal })
                });
                
                if (response.ok) {
                    showToast(cmd, stepVal);
                } else {
                    console.error('Failed to send command');
                }
            } catch (error) {
                console.error('Error:', error);
                alert('Connection to ROS2 Node failed. Is the server running?');
            }
        }
    </script>
</body>
</html>
"""

# Global reference to ROS2 node for the HTTP handler
gui_node = None

class WebGUIRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/api/command':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                cmd = data.get('cmd', '')
                step = data.get('step', 0)
                
                if cmd and gui_node:
                    # Publish string message e.g. "BODY_HEAVE 0"
                    msg = String()
                    msg.data = f"{cmd} {step}"
                    gui_node.publisher_.publish(msg)
                    gui_node.get_logger().info(f'Web UI published: "{msg.data}"')

                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok'}).encode('utf-8'))
            except Exception as e:
                self.send_response(400)
                self.end_headers()
                if gui_node:
                    gui_node.get_logger().error(f'Error parsing POST data: {str(e)}')
        else:
            self.send_response(404)
            self.end_headers()

    # Suppress default HTTP logging to stdout to keep ROS logs clean
    def log_message(self, format, *args):
        pass

class WebGuiNode(Node):
    def __init__(self):
        super().__init__('node_web_gui')
        self.publisher_ = self.create_publisher(String, '/gait_control', 10)
        self.server_port = 8080
        self.server = None
        self.server_thread = None

    def start_server(self):
        global gui_node
        gui_node = self
        self.server = ThreadingHTTPServer(('0.0.0.0', self.server_port), WebGUIRequestHandler)
        self.get_logger().info(f'Web GUI Server started at http://localhost:{self.server_port}')
        
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

    def stop_server(self):
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        if self.server_thread:
            self.server_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = WebGuiNode()
    
    node.start_server()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_server()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
