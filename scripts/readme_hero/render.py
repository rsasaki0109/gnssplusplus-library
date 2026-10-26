"""Capture a measured PLATEAU/OSM scene; incomplete map loads fail explicitly."""

import argparse
import asyncio
import hashlib
import io
import json
import mimetypes
import urllib.request
from pathlib import Path
from urllib.parse import urlparse, unquote
from PIL import Image
from playwright.async_api import async_playwright

root = Path(__file__).resolve().parent
repo = root.parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument(
    "--trace", dest="scene", type=Path, default=repo / "docs/odaiba_hero_trace.json"
)
p.add_argument("--output", type=Path, default=repo / "docs/odaiba_hero.gif")
p.add_argument("--basemap", type=Path, default=repo / "docs/odaiba_hero_basemap.png")
p.add_argument("--cesium-dir", type=Path, required=True)
p.add_argument("--sky-metadata", type=Path, default=repo / "docs/odaiba_hero_sky.json")
p.add_argument("--sky-sprite", type=Path, default=repo / "docs/odaiba_hero_sky.png")
p.add_argument(
    "--cache-dir", type=Path, default=repo / "output/readme-hero/resource-cache"
)
p.add_argument("--fps", type=int, default=8)
p.add_argument("--frames", type=int, default=80)
a = p.parse_args()
if not 1 <= a.fps <= 50 or a.frames < 2:
    p.error("fps must be between 1 and 50; at least two frames required")
data = json.loads(a.scene.read_text())
sky_metadata = json.loads(a.sky_metadata.read_text())
if sky_metadata.get("schema") != "odaiba_gt_zenith.v2":
    p.error(
        "Sky snapshot uses an obsolete observation-filtered schema; rebuild the exporter and prepare_sky.py"
    )
if (
    sky_metadata["inputs"]["trace_sha256"]
    != hashlib.sha256(a.scene.read_bytes()).hexdigest()
):
    p.error("Sky data does not match this trace; regenerate with prepare_sky.py")
if (
    sky_metadata["sprite_sha256"]
    != hashlib.sha256(a.sky_sprite.read_bytes()).hexdigest()
):
    p.error("Sky sprite does not match its metadata")
cache = a.cache_dir
cache.mkdir(parents=True, exist_ok=True)
a.output.parent.mkdir(parents=True, exist_ok=True)
if not (a.cesium_dir / "Cesium.js").is_file():
    p.error("--cesium-dir must contain the npm Cesium.js build")


async def main():
    failures = []
    resources = set()
    general = asyncio.Semaphore(6)
    osm = asyncio.Semaphore(2)
    locks = {}
    async with async_playwright() as pw:
        browser = await pw.chromium.launch(
            executable_path="/usr/bin/chromium",
            args=[
                "--no-sandbox",
                "--use-angle=swiftshader",
                "--enable-unsafe-swiftshader",
            ],
        )
        page = await browser.new_page(
            viewport={"width": 960, "height": 480}, device_scale_factor=1
        )
        page.on("pageerror", lambda e: print("JS ERROR", str(e), flush=True))

        async def route(req):
            url = req.request.url
            parts = urlparse(url)
            if parts.netloc == "hero.local":
                path = unquote(parts.path)
                local = (
                    a.cesium_dir / path.removeprefix("/cesium/")
                    if path.startswith("/cesium/")
                    else (
                        a.basemap
                        if path == "/basemap.png"
                        else (
                            a.sky_metadata
                            if path == "/sky.json"
                            else (
                                a.sky_sprite
                                if path == "/sky.png"
                                else root / "viewer.html"
                            )
                        )
                    )
                )
                await req.fulfill(
                    path=str(local),
                    content_type=mimetypes.guess_type(local.name)[0]
                    or "application/octet-stream",
                )
                return
            key = hashlib.sha256(url.encode()).hexdigest()
            blob = cache / key
            meta = cache / (key + ".json")
            try:
                lock = locks.setdefault(url, asyncio.Lock())
                async with lock:
                    if not blob.exists():

                        def fetch():
                            request = urllib.request.Request(
                                url,
                                headers={
                                    "User-Agent": "gnssplusplus-library README illustration (https://github.com/rsasaki0109/gnssplusplus-library)"
                                },
                            )
                            with urllib.request.urlopen(
                                request, timeout=40
                            ) as response:
                                body = response.read()
                                kind = response.headers.get(
                                    "Content-Type", "application/octet-stream"
                                )
                            blob.write_bytes(body)
                            meta.write_text(json.dumps({"url": url, "type": kind}))
                            return body, kind

                        async with (
                            osm
                            if parts.hostname == "tile.openstreetmap.org"
                            else general
                        ):
                            body, kind = await asyncio.to_thread(fetch)
                        print(
                            "cached",
                            parts.hostname,
                            parts.path[-65:],
                            len(body),
                            flush=True,
                        )
                    else:
                        body = blob.read_bytes()
                        kind = json.loads(meta.read_text())["type"]
                resources.add(url)
                await req.fulfill(
                    body=body,
                    content_type=kind,
                    headers={"Access-Control-Allow-Origin": "*"},
                )
            except Exception as e:
                failures.append((url, str(e)))
                print("FETCH FAILED", parts.hostname, str(e), flush=True)
                await req.abort()

        await page.route("**/*", route)
        try:
            await page.goto("http://hero.local/")
            await page.evaluate("data=>prepare(data)", data)
            speed = (data["end"] - data["start"]) * a.fps / a.frames
            await page.locator("header span").evaluate(
                "(e,text)=>e.textContent=text",
                f"UrbanNav Tokyo Odaiba · {speed:.1f}× replay",
            )
            frames = []
            for i in range(a.frames):
                t = data["start"] + (data["end"] - data["start"]) * i / (a.frames - 1)
                await page.evaluate("t=>renderFrame(t)", t)
                try:
                    await page.wait_for_function(
                        # requestRenderMode only advances tiles on a render;
                        # glb (Draco) content finishes decoding asynchronously.
                        "(viewers.forEach(v=>v.scene.requestRender()), viewers.every(v=>v.scene.globe.tilesLoaded && v.heroTiles.tilesLoaded && v.bridgeTiles.every(t=>t.tilesLoaded)))",
                        timeout=60000,
                    )
                except Exception:
                    await page.screenshot(path=str(a.output.with_suffix(".debug.png")))
                    print(
                        "LOAD STATE",
                        await page.evaluate(
                            "viewers.map(v=>({globe:v.scene.globe.tilesLoaded,city:v.heroTiles.tilesLoaded,pending:v.heroTiles.statistics.numberOfPendingRequests,processing:v.heroTiles.statistics.numberOfTilesProcessing,bridges:v.bridgeTiles.map(t=>({loaded:t.tilesLoaded,pending:t.statistics.numberOfPendingRequests,processing:t.statistics.numberOfTilesProcessing,selected:t.statistics.selected}))}))"
                        ),
                        flush=True,
                    )
                    print("FAILURES", failures, flush=True)
                    raise
                await page.evaluate("viewers.forEach(v=>v.render())")
                if failures or await page.evaluate("tileFailures.length"):
                    raise RuntimeError(f"Incomplete scene: {failures[:3]}")
                frames.append(
                    Image.open(io.BytesIO(await page.screenshot())).convert("RGB")
                )
                if i == 0:
                    frames[0].save(str(a.output.with_suffix(".first.png")))
                if i % 12 == 0:
                    print(f"frame {i+1}/{a.frames}", flush=True)
            if not resources:
                raise RuntimeError("No external map resources loaded")
            # Reserve the method colors: a map-dominated adaptive palette can
            # otherwise turn thin orange/mint trajectories into gray pixels.
            adaptive = frames[len(frames) // 2].quantize(colors=108).getpalette()[:324]
            reserved = [
                "f5ac63",
                "60e2bb",
                "dc9958",
                "56c9a6",
                "c48850",
                "4db393",
                "ac7848",
                "439d82",
                "ffffff",
                "eef3f8",
                "9aaabd",
                "0c111a",
                "101824",
                "17202b",
                "8093a8",
                "303945",
                "39e6a8",
                "ff6b6b",
                "aab2c0",
                "a1c6e1",
            ]
            colors = adaptive + [
                int(c[j : j + 2], 16) for c in reserved for j in (0, 2, 4)
            ]
            palette = Image.new("P", (1, 1))
            palette.putpalette(colors)
            # GIF stores centiseconds; distribute rounding instead of making
            # every 8 fps frame 120 ms (which would mislabel playback speed).
            durations = [
                (round((i + 1) * 100 / a.fps) - round(i * 100 / a.fps)) * 10
                for i in range(len(frames))
            ]
            encoded = [
                f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames
            ]
            encoded[0].save(
                a.output,
                save_all=True,
                append_images=encoded[1:],
                duration=durations,
                loop=0,
                optimize=True,
                disposal=1,
            )
            a.output.with_suffix(".provenance.json").write_text(
                json.dumps(
                    {
                        "scene": data,
                        "resources": sorted(resources),
                        "frames": len(frames),
                        "fps": a.fps,
                    },
                    indent=2,
                )
            )
            print("Saved", a.output, a.output.stat().st_size, "bytes", flush=True)
        finally:
            await browser.close()


asyncio.run(main())
