#!/usr/bin/env python3
"""Check managed processes and the browser desktop, without probing ROS topics.

This is a container liveness check. Sensor publication must be checked
separately after starting the simulator.
"""

import json
import os
from pathlib import Path
import socket
import sys
import urllib.request


STATE_FILE = Path('/tmp/sci-runtime/state.json')


def check_http(port):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(f'http://127.0.0.1:{port}/vnc.html', timeout=2) as response:
        if response.status != 200:
            raise RuntimeError(f'noVNC returned HTTP {response.status}')


def check_vnc(port):
    with socket.create_connection(('127.0.0.1', port), timeout=2) as connection:
        if not connection.recv(12).startswith(b'RFB '):
            raise RuntimeError('VNC did not return an RFB greeting')


def main():
    # These modes let the startup script wait for actual service availability.
    if len(sys.argv) == 3 and sys.argv[1] == '--http-port':
        check_http(int(sys.argv[2]))
        return
    if len(sys.argv) == 3 and sys.argv[1] == '--vnc-port':
        check_vnc(int(sys.argv[2]))
        return
    if len(sys.argv) != 1:
        raise ValueError('expected --http-port PORT, --vnc-port PORT, or no arguments')

    state = json.loads(STATE_FILE.read_text())
    expected = {'xvfb', 'fluxbox', 'x11vnc', 'websockify', 'simulation'}
    if set(state['processes']) != expected:
        raise RuntimeError('incomplete process state')
    for name, pid in state['processes'].items():
        os.kill(pid, 0)
        # kill(pid, 0) also succeeds for a zombie, which is not a healthy service.
        status = Path(f'/proc/{pid}/stat').read_text().rsplit(') ', 1)[1][0]
        if status in {'Z', 'X'}:
            raise RuntimeError(f'{name} has exited')
    check_vnc(int(state['vnc_port']))
    check_http(int(state['web_port']))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f'SCI healthcheck: {error}', file=sys.stderr)
        sys.exit(1)
