"""
Shared Kit start-up for the B' Isaac-side scripts (test_base.py, test_t6.py).

  start_app(livestream=False)  -> SimulationApp, headless, no window (default; used for PASS/FAIL runs)
  start_app(livestream=True)   -> SimulationApp, headless + WebRTC livestream of the full UI on port 8211

The livestream configuration is copied from the example shipped in the Isaac Sim 4.1 image:
  /isaac-sim/standalone_examples/api/omni.isaac.kit/livestream.py
with two deliberate changes:
  * the WebRTC extension (omni.services.streamclient.webrtc) is enabled instead of the native one,
    because the Omniverse Streaming Client is deprecated and a browser is enough;
  * window_width/height are 1280x720 instead of 1920x1080, to save VRAM on 4 GB laptop GPUs.
Only ONE livestream extension can be enabled per Isaac Sim instance (comment in that example).
"""

LIVESTREAM_URL = "http://127.0.0.1:8211/streaming/webrtc-client?server=127.0.0.1"


def start_app(livestream=False, log=print):
    try:
        from isaacsim import SimulationApp  # recommended import in 4.1 (avoids the deprecation warning)
    except ImportError:  # pragma: no cover
        from omni.isaac.kit import SimulationApp

    if not livestream:
        return SimulationApp({"headless": True})

    app = SimulationApp({
        "width": 1280,
        "height": 720,
        "window_width": 1280,
        "window_height": 720,
        "headless": True,
        "hide_ui": False,              # stream the full UI (viewport, Stage, Property)
        "renderer": "RayTracedLighting",
        "display_options": 3286,       # default grid, as in the NVIDIA example
    })
    from omni.isaac.core.utils.extensions import enable_extension

    app.set_setting("/app/window/drawMouse", True)
    app.set_setting("/app/livestream/proto", "ws")
    app.set_setting("/ngx/enabled", False)
    enable_extension("omni.services.streamclient.webrtc")
    log(f"LIVESTREAM (WebRTC) enabled. Open in Chrome/Chromium on this machine: {LIVESTREAM_URL}")
    log("LIVESTREAM note: viewing only - rendering the UI slows the simulation; do not use for PASS/FAIL runs")
    return app
