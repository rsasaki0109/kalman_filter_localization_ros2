# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Check byte-range integrity and quaternion interpolation at a yaw wrap."""

import io
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import math
import threading
import unittest
import zipfile

from fetch_odaiba import RemoteZip
import numpy as np
from render import interpolate


class RangeTest(unittest.TestCase):
    def test_zip_random_access_and_no_ignored_range(self):
        """Read compressed data by exact ranges; reject full-body fallback."""
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w', compression=zipfile.ZIP_DEFLATED) as writer:
            writer.writestr('Tokyo_Data/Odaiba/imu.csv', 'real-file-stand-in\n'*100)
        payload = stream.getvalue()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_HEAD(self):
                self.send_response(200)
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()

            def do_GET(self):
                a, b = map(int, self.headers['Range'].removeprefix('bytes=').split('-'))
                self.send_response(200 if self.path == '/ignore' else 206)
                self.send_header('Content-Range', f'bytes {a}-{b}/{len(payload)}')
                self.send_header('Content-Length', str(b-a+1))
                self.end_headers()
                self.wfile.write(payload[a:b+1])

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        url = 'http://127.0.0.1:'+str(server.server_port)
        try:
            remote = RemoteZip(url+'/archive')
            with zipfile.ZipFile(remote) as reader:
                self.assertEqual(reader.read('Tokyo_Data/Odaiba/imu.csv'),
                                 b'real-file-stand-in\n'*100)
            with self.assertRaisesRegex(RuntimeError, 'exact byte range'):
                RemoteZip(url+'/ignore').read(4)
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


class AttitudeTest(unittest.TestCase):
    def test_shortest_quaternion_path_at_yaw_wrap(self):
        """The ±180 degree wrap must not interpolate through zero heading."""
        reference = np.zeros((2, 11))
        reference[:, 0] = [10., 10.1]
        yaw = np.radians([179., -179.])
        reference[:, 6] = np.sin(yaw/2)
        reference[:, 7] = np.cos(yaw/2)
        reference[:, 10] = yaw
        middle = interpolate(reference, np.array([10.05]))[0]
        self.assertAlmostEqual(abs(middle[6]), 1., places=12)
        self.assertAlmostEqual(middle[7], 0., places=12)
        self.assertAlmostEqual(middle[10], math.pi, places=12)
        self.assertAlmostEqual(np.linalg.norm(middle[4:8]), 1., places=12)


if __name__ == '__main__':
    unittest.main()
