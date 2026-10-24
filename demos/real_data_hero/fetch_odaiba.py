#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Fetch only Odaiba navigation files from the official UrbanNav ZIP by ranges."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import shutil
import urllib.request
import zipfile

URL = ('https://dl.dropboxusercontent.com/scl/fi/b0d7jilvpjxf8h471iqvb/'
       'Tokyo_Data.zip?rlkey=z1qjhr84nb6ahxypvy6fn5ow6')
FILES = {'imu.csv', 'reference.csv', 'base.nav', 'base_trimble.obs',
         'rover_trimble.obs', 'rover_ublox.obs'}


class RemoteZip(io.RawIOBase):
    """Seek/read with verified HTTP range bounds; never download the LiDAR bags."""

    def __init__(self, url):
        self.url, self.offset, self.transferred = url, 0, 0
        with urllib.request.urlopen(urllib.request.Request(url, method='HEAD'), timeout=30) as r:
            self.size = int(r.headers['Content-Length'])

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.offset

    def seek(self, offset, whence=0):
        self.offset = offset + (0 if whence == 0 else self.offset if whence == 1 else self.size)
        if self.offset < 0:
            raise ValueError('Negative archive seek')
        return self.offset

    def read(self, size=-1):
        size = min(self.size-self.offset, size if size >= 0 else self.size-self.offset)
        if size <= 0:
            return b''
        end = self.offset+size-1
        request = urllib.request.Request(self.url, headers={
            'Range': 'bytes={}-{}'.format(self.offset, end), 'Accept-Encoding': 'identity'})
        with urllib.request.urlopen(request, timeout=60) as r:
            expected = 'bytes {}-{}/{}'.format(self.offset, end, self.size)
            if r.status != 206 or r.headers.get('Content-Range') != expected:
                raise RuntimeError('Server did not honor the exact byte range')
            data = r.read(size+1)
        if len(data) != size:
            raise RuntimeError('Incomplete or excessive range response')
        self.offset += size
        self.transferred += size
        return data


def fetch(output):
    output.mkdir(parents=True, exist_ok=False)
    remote = RemoteZip(URL)
    artifacts = []
    with zipfile.ZipFile(remote) as archive:
        selected = [info for info in archive.infolist()
                    if 'Odaiba' in info.filename.split('/') and Path(info.filename).name in FILES]
        if len(selected) != len(FILES) or {Path(i.filename).name for i in selected} != FILES:
            raise RuntimeError('Official Odaiba member names changed: ' +
                               str([i.filename for i in selected]))
        for info in selected:
            destination = output/Path(info.filename).name
            partial = destination.with_suffix(destination.suffix+'.partial')
            print('Downloading', info.filename, 'bytes', info.file_size, flush=True)
            with archive.open(info) as source, partial.open('wb') as target:
                shutil.copyfileobj(source, target, length=1024*1024)
            if partial.stat().st_size != info.file_size:
                raise RuntimeError('Incomplete ZIP member')
            partial.rename(destination)
            artifacts.append(dict(member=info.filename, bytes=info.file_size,
                                  crc32='{:08x}'.format(info.CRC),
                                  sha256=hashlib.sha256(destination.read_bytes()).hexdigest()))
    manifest = dict(source='https://github.com/IPNL-POLYU/UrbanNavDataset#tokyo-dataset',
                    archive_url=URL, archive_bytes=remote.size,
                    transferred_bytes=remote.transferred, files=artifacts,
                    selected_split='Odaiba regression/demo; not the Shinjuku holdout')
    (output/'download_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    fetch(parser.parse_args().output)
