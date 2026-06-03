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
    <title>Quadruped Control</title>
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg: #0a0e17;
            --surface: rgba(255,255,255,0.04);
            --border: rgba(255,255,255,0.08);
            --text: #e2e8f0;
            --muted: #64748b;
            --blue: #38bdf8;
            --red: #f87171;
            --green: #34d399;
            --purple: #a78bfa;
            --radius: 12px;
        }

        * { box-sizing: border-box; margin: 0; padding: 0; }

        body {
            font-family: 'Inter', system-ui, sans-serif;
            background: var(--bg);
            color: var(--text);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            padding: 1.5rem;
        }

        .app {
            width: 100%;
            max-width: 420px;
            display: flex;
            flex-direction: column;
            gap: 1rem;
        }

        /* ── Header ─────────────────────────────── */
        .header {
            text-align: center;
            padding: 1.25rem 0 0.5rem;
        }
        .header h1 {
            font-size: 1.4rem;
            font-weight: 700;
            background: linear-gradient(135deg, var(--blue), var(--purple));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            letter-spacing: -0.03em;
        }
        .header p {
            color: var(--muted);
            font-size: 0.8rem;
            margin-top: 0.25rem;
        }

        /* ── Step input ─────────────────────────── */
        .step-row {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 0.5rem;
            padding: 0.6rem 1rem;
            background: var(--surface);
            border: 1px solid var(--border);
            border-radius: var(--radius);
        }
        .step-row label {
            font-size: 0.8rem;
            color: var(--muted);
            font-weight: 500;
        }
        .step-row input {
            width: 64px;
            background: rgba(0,0,0,0.3);
            border: 1px solid var(--border);
            color: var(--blue);
            font-size: 0.95rem;
            font-weight: 600;
            text-align: center;
            padding: 0.35rem 0.5rem;
            border-radius: 8px;
            outline: none;
            transition: border-color 0.2s;
        }
        .step-row input:focus {
            border-color: var(--blue);
        }

        /* ── Section ────────────────────────────── */
        .section {
            background: var(--surface);
            border: 1px solid var(--border);
            border-radius: var(--radius);
            padding: 1rem;
        }
        .section-label {
            font-size: 0.65rem;
            text-transform: uppercase;
            letter-spacing: 0.1em;
            color: var(--muted);
            margin-bottom: 0.6rem;
            font-weight: 600;
        }

        /* ── Button grids ───────────────────────── */
        .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 0.5rem; }
        .grid-3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 0.5rem; }

        /* ── Buttons ────────────────────────────── */
        button {
            font-family: 'Inter', system-ui, sans-serif;
            background: rgba(255,255,255,0.03);
            border: 1px solid var(--border);
            color: var(--text);
            padding: 0.7rem 0.5rem;
            border-radius: 10px;
            font-size: 0.82rem;
            font-weight: 500;
            cursor: pointer;
            transition: all 0.15s ease;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 0.15rem;
            user-select: none;
            -webkit-tap-highlight-color: transparent;
        }
        button .icon { font-size: 1.2rem; }
        button .sub {
            font-size: 0.65rem;
            color: var(--muted);
            font-weight: 400;
        }
        button:hover {
            background: rgba(255,255,255,0.07);
            border-color: rgba(255,255,255,0.15);
            transform: translateY(-1px);
        }
        button:active {
            transform: scale(0.97);
        }

        .btn-home {
            background: rgba(56,189,248,0.08);
            border-color: rgba(56,189,248,0.25);
            color: var(--blue);
        }
        .btn-home:hover {
            background: rgba(56,189,248,0.15);
            border-color: var(--blue);
            box-shadow: 0 0 12px rgba(56,189,248,0.2);
        }

        .btn-off {
            background: rgba(248,113,113,0.08);
            border-color: rgba(248,113,113,0.25);
            color: var(--red);
        }
        .btn-off:hover {
            background: rgba(248,113,113,0.15);
            border-color: var(--red);
            box-shadow: 0 0 12px rgba(248,113,113,0.2);
        }

        /* ── Toast ──────────────────────────────── */
        #toast {
            position: fixed;
            bottom: 1.25rem;
            left: 50%;
            transform: translateX(-50%) translateY(120%);
            background: rgba(15,20,35,0.9);
            backdrop-filter: blur(12px);
            border: 1px solid var(--border);
            color: var(--text);
            padding: 0.6rem 1.2rem;
            border-radius: 10px;
            font-size: 0.8rem;
            font-weight: 500;
            transition: transform 0.25s ease;
            z-index: 100;
            white-space: nowrap;
        }
        #toast.show { transform: translateX(-50%) translateY(0); }
    </style>
</head>
<body>
<div class="app">

    <div class="header">
        <h1>Quadruped Control</h1>
        <p>ROS 2 Web Interface</p>
    </div>

    <div class="step-row">
        <label>Steps</label>
        <input type="number" id="step-input" value="0" min="0" max="999">
        <label style="color:var(--muted);font-size:0.7rem;">(0 = ∞)</label>
    </div>

    <!-- System -->
    <div class="section">
        <div class="section-label">System</div>
        <div class="grid-2">
            <button class="btn-home" onclick="send('ZERO')">
                <span class="icon">⌂</span> HOME
                <span class="sub">Reset position</span>
            </button>
            <button class="btn-off" onclick="send('ROBOTOFF')">
                <span class="icon">⏻</span> OFF
                <span class="sub">Shutdown</span>
            </button>
        </div>
    </div>

    <!-- Movement -->
    <div class="section">
        <div class="section-label">Movement</div>
        <div class="grid-2">
            <button onclick="send('TROT_FORWARD')">
                <span class="icon">↑</span> Trot Fwd
            </button>
            <button onclick="send('TROT_BACKWARD')">
                <span class="icon">↓</span> Trot Bwd
            </button>
            <button onclick="send('TURN_LEFT')">
                <span class="icon">↺</span> Turn Left
            </button>
            <button onclick="send('TURN_RIGHT')">
                <span class="icon">↻</span> Turn Right
            </button>
        </div>
    </div>

    <!-- Body -->
    <div class="section">
        <div class="section-label">Body Motion</div>
        <div class="grid-2">
            <button onclick="send('BODY_HEAVE')">
                <span class="icon">↕</span> Heave
            </button>
            <button onclick="send('BODY_ROLL')">
                <span class="icon">⇔</span> Roll
            </button>
            <button onclick="send('BODY_PITCH')">
                <span class="icon">⇕</span> Pitch
            </button>
            <button onclick="send('BODY_CIRCLE')">
                <span class="icon">◎</span> Circle
            </button>
        </div>
    </div>

</div>

<div id="toast"></div>

<script>
    let tt;
    function toast(t) {
        const el = document.getElementById('toast');
        el.textContent = t;
        el.classList.add('show');
        clearTimeout(tt);
        tt = setTimeout(() => el.classList.remove('show'), 2000);
    }

    async function send(cmd) {
        const step = parseInt(document.getElementById('step-input').value) || 0;
        try {
            const r = await fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ cmd, step })
            });
            if (r.ok) toast(`✓ ${cmd} ${step}`);
            else toast('✗ Failed');
        } catch (e) {
            toast('✗ Connection lost');
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
