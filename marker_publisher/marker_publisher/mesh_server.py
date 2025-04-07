import os
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from rclpy.node import Node


class MeshServer:
    def __init__(self, node: Node, mesh_resource_dir: str, port: int):
        # Ensure mesh directory exists
        if not os.path.exists(mesh_resource_dir):
            raise FileNotFoundError(f"Mesh directory does not exist: {mesh_resource_dir}")
        self.mesh_resource_dir = mesh_resource_dir
        
        self.node = node
        self.port = port
        self.http_thread = None
        self.httpd = None

    def start(self):
        mesh_resource_dir = self.mesh_resource_dir
        node = self.node  # 存储节点引用以便在Handler中使用

        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=mesh_resource_dir, **kwargs)
            
            # 重写log_message方法，将HTTP请求日志发送到ROS节点
            def log_message(self, format, *args):
                node.get_logger().info(f"HTTP: {self.address_string()} - {format % args}")
                
            # 记录额外的文件访问细节
            def do_GET(self):
                # 记录请求路径
                node.get_logger().info(f"GET request for {self.path} from {self.client_address}")
                try:
                    # 检查请求的文件是否存在
                    file_path = os.path.join(mesh_resource_dir, self.path.lstrip('/'))
                    if os.path.exists(file_path):
                        node.get_logger().info(f"File exists: {file_path}")
                    else:
                        node.get_logger().warn(f"File not found: {file_path}")
                except Exception as e:
                    node.get_logger().error(f"Error checking file: {str(e)}")
                # 调用原始方法处理请求
                return super().do_GET()

        def run_server():
            try:
                node.get_logger().info(f"Starting HTTP server, serving from: {mesh_resource_dir}")
                self.httpd = HTTPServer(("", self.port), Handler)
                node.get_logger().info(
                    f"HTTP server started on port {self.port}"
                )
                self.httpd.serve_forever()
            except Exception as e:
                node.get_logger().error(f"HTTP server error: {str(e)}")

        self.http_thread = threading.Thread(target=run_server, daemon=True)
        self.http_thread.start()
        
        self.node.get_logger().info(f"HTTP server running on port {self.port}")
        self.node.get_logger().info(f"Serving files from: {self.mesh_resource_dir}")

    def stop(self):
        if self.httpd:
            self.node.get_logger().info("Shutting down HTTP server...")
            self.httpd.shutdown()
            self.httpd.server_close()
            self.http_thread.join(timeout=1.0)
            if self.http_thread.is_alive():
                self.node.get_logger().warn("HTTP server thread did not terminate gracefully") 