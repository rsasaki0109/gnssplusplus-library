"""Fetch only the four public UrbanNav Odaiba inputs using ZIP byte ranges."""

import argparse
import io
import urllib.request
import zipfile
import json
import hashlib
from pathlib import Path

URL = "https://dl.dropboxusercontent.com/s/isu1bugpgr8dpas/Tokyo_Data.zip"


class Remote(io.RawIOBase):
    def __init__(self):
        with urllib.request.urlopen(
            urllib.request.Request(URL, method="HEAD"), timeout=45
        ) as r:
            self.size = int(r.headers["Content-Length"])
        self.pos = 0

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=0):
        self.pos = (
            offset
            if whence == 0
            else self.pos + offset if whence == 1 else self.size + offset
        )
        return self.pos

    def read(self, n=-1):
        if self.pos >= self.size:
            return b""
        if n < 0:
            n = self.size - self.pos
        end = min(self.size, self.pos + n) - 1
        if end < self.pos:
            return b""
        req = urllib.request.Request(URL, headers={"Range": f"bytes={self.pos}-{end}"})
        with urllib.request.urlopen(req, timeout=120) as r:
            if r.status != 206:
                raise RuntimeError(f"Range request unsupported: {r.status}")
            blob = r.read()
        if len(blob) != end - self.pos + 1:
            raise RuntimeError("truncated range")
        self.pos += len(blob)
        return blob


repo = Path(__file__).resolve().parents[2]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument(
    "--output-dir", type=Path, default=repo / "data/driving/Tokyo_Data/Odaiba"
)
p.add_argument(
    "--manifest", type=Path, default=repo / "output/readme-hero/input-manifest.json"
)
a = p.parse_args()
root = a.output_dir
root.mkdir(parents=True, exist_ok=True)
manifest = []
with zipfile.ZipFile(Remote()) as z:
    selected = [
        i
        for i in z.infolist()
        if "/Odaiba/" in "/" + i.filename
        and Path(i.filename).name
        in ["rover_trimble.obs", "base_trimble.obs", "base.nav", "reference.csv"]
    ]
    print([(i.filename, i.file_size, i.compress_size) for i in selected], flush=True)
    if len(selected) != 4:
        raise RuntimeError("Expected all four Odaiba inputs")
    for info in selected:
        target = root / Path(info.filename).name
        print("Extracting", info.filename, flush=True)
        # Reading the full member minimizes round trips. ZipFile checks its CRC.
        blob = z.read(info)
        if target.exists() and target.read_bytes() != blob:
            raise RuntimeError(f"Refusing to overwrite differing input: {target}")
        target.write_bytes(blob)
        manifest.append(
            {
                "member": info.filename,
                "size": len(blob),
                "crc32": f"{info.CRC:08x}",
                "sha256": hashlib.sha256(blob).hexdigest(),
            }
        )
a.manifest.parent.mkdir(parents=True, exist_ok=True)
a.manifest.write_text(json.dumps({"url": URL, "files": manifest}, indent=2))
print("Verified CRC and saved all four inputs.", flush=True)
