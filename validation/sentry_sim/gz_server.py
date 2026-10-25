"""Headless Gazebo Harmonic server, using its native Python bindings."""
import sys
from gz.sim8 import Server, ServerConfig

config = ServerConfig()
config.set_sdf_file(sys.argv[1])
server = Server(config)
if not server.run(True, 0, False):
    raise SystemExit('Gazebo server failed')
