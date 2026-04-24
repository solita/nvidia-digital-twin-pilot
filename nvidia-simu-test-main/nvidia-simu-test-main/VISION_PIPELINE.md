# Machine Vision Pipeline

Add a forward-facing RGB camera to the simulated Jetbot, capture frames with Isaac Sim's Replicator annotator API, publish as ROS2 topics, and display live on the web dashboard — robot-eye view, semantic segmentation overlay, and object detection bounding boxes.

All three features share a single camera and RenderProduct. Ground-truth segmentation and bounding boxes come free from the simulator (no ML model needed). A YOLO extension is planned as an optional addition for comparing ML inference against ground truth.

---

## Current State

The project currently uses **lidar-only** perception:

| Sensor | Topic | Rate | Dashboard |
|--------|-------|------|-----------|
| PhysX 2D Lidar (360 rays) | `/scan` | ~18 Hz | Polar plot overlay |
| Odometry (articulation pose) | `/odom` | ~50 Hz | Position/velocity readout |
| Twist commands | `/cmd_vel` | ~10 Hz | Linear/angular display |
| Simulation clock | `/clock` | ~56 Hz | Sim time counter |

There is an orbit camera (`/World/FollowCam`) for the viewport, but it is **not published as a ROS2 topic** — it only drives the WebRTC streaming view. No camera images flow through ROS2 or the dashboard.

---

## Architecture

```
Isaac Sim container (GPU, A4000)
  └─ /World/Jetbot/chassis/FrontCamera        ← USD Camera prim
     └─ RenderProduct (480 × 320)
        │
        ├─ rgb annotator
        │   └─ JPEG compress (quality 75)
        │   └─ rclpy publish ─────────────────→ /camera/image/compressed
        │                                       sensor_msgs/CompressedImage
        │
        ├─ semantic_segmentation annotator
        │   └─ Class ID → colour map
        │   └─ PNG encode (240 × 160)
        │   └─ rclpy publish ─────────────────→ /camera/segmentation/compressed
        │                                       sensor_msgs/CompressedImage
        │
        └─ bounding_box_2d_tight annotator
            └─ Struct → JSON
            └─ rclpy publish ─────────────────→ /camera/detections
                                                std_msgs/String

Dashboard container (ros2-dashboard)
  └─ ROS2 subscriptions (BEST_EFFORT QoS)
     ├─ /camera/image/compressed        → base64 encode → WebSocket
     ├─ /camera/segmentation/compressed → base64 encode → WebSocket
     └─ /camera/detections              → parse JSON    → WebSocket

Browser (index.html)
  └─ Camera panel
     ├─ <canvas> base layer: JPEG robot-eye feed
     ├─ <canvas> overlay: semantic segmentation (togglable opacity)
     └─ <canvas> overlay: bounding box rectangles + labels
```

### Data Flow Summary

| Topic | Format | Size/frame | Target FPS | Bandwidth |
|-------|--------|-----------|------------|-----------|
| `/camera/image/compressed` | JPEG, 480×320 | ~20 KB | 10 | ~200 KB/s |
| `/camera/segmentation/compressed` | PNG, 240×160 | ~8 KB | 5 | ~40 KB/s |
| `/camera/detections` | JSON string | ~0.5 KB | 5 | ~2.5 KB/s |
| **Total** | | | | **~242 KB/s** |

The WebSocket to the browser throttles images to ~2 FPS to keep browser bandwidth reasonable over Internet connections.

---

## Phase 1: Robot-Eye Camera Feed

Mount an RGB camera on the Jetbot and stream its first-person view to the dashboard.

### 1.1 Add Camera Prim

Create a USD Camera on the Jetbot chassis in `ros2_jetbot.py`, positioned just above the lidar sensor:

```python
front_cam = UsdGeom.Camera.Define(stage, "/World/Jetbot/chassis/FrontCamera")
front_cam.GetFocalLengthAttr().Set(18.0)           # wide FOV for indoor navigation
front_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 20.0))

cam_xf = UsdGeom.Xformable(front_cam.GetPrim())
cam_xf.ClearXformOpOrder()
cam_xf.AddTranslateOp().Set(Gf.Vec3d(0.05, 0.0, 0.08))  # 5 cm forward, 8 cm up
```

The camera moves with the robot because it is a child of the `chassis` prim.

### 1.2 Create RenderProduct

Enable the Replicator extension and create an off-screen render target:

```python
ext_mgr.set_extension_enabled_immediate("omni.replicator.core", True)
import omni.replicator.core as rep

render_product = rep.create.render_product(
    "/World/Jetbot/chassis/FrontCamera", (480, 320)
)
```

A RenderProduct tells the RTX renderer to produce an image from this camera every frame, separate from the main viewport. The 480×320 resolution balances visual quality against GPU overhead and JPEG bandwidth.

### 1.3 Attach RGB Annotator

```python
rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annot.attach([render_product])
```

The annotator reads the GPU framebuffer each frame and makes it available as a NumPy array.

### 1.4 ROS2 Publisher

Since Isaac Sim 5.1.0 does not have an OmniGraph node for image publishing, we create a standalone rclpy publisher inside the Isaac Sim script:

```python
import rclpy
from sensor_msgs.msg import CompressedImage

# ROS2 bridge extension may already have initialized rclpy
try:
    rclpy.init()
except RuntimeError:
    pass  # Already initialized by the ROS2 bridge

cam_node = rclpy.create_node('jetbot_camera')
cam_pub = cam_node.create_publisher(CompressedImage, '/camera/image/compressed', 1)
```

### 1.5 Physics Callback

A new `_camera_step` callback reads the annotator, compresses to JPEG, and publishes. It runs every 6th physics tick (~10 FPS at 60 Hz simulation):

```python
_cam_state = {"ticks": 0}

def _camera_step(dt: float):
    _cam_state["ticks"] += 1
    if _cam_state["ticks"] % 6 != 0:
        return

    rgba = rgb_annot.get_data()          # numpy uint8 [320, 480, 4]
    rgb = rgba[:, :, :3]                 # drop alpha channel

    # JPEG compress
    from io import BytesIO
    from PIL import Image
    buf = BytesIO()
    Image.fromarray(rgb).save(buf, format='JPEG', quality=75)
    jpeg_bytes = buf.getvalue()          # ~15-25 KB

    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = jpeg_bytes
    cam_pub.publish(msg)

world.add_physics_callback("camera", _camera_step)
```

### 1.6 Dashboard Server

Add a subscription in `dashboard_server.py`:

```python
from sensor_msgs.msg import CompressedImage
import base64

self.create_subscription(
    CompressedImage,
    '/camera/image/compressed',
    self._camera_cb,
    QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1),
)

def _camera_cb(self, msg):
    with _lock:
        _state["camera_rgb"] = base64.b64encode(msg.data).decode('ascii')
```

The WebSocket broadcast includes the base64 image every 2nd fast tick (~2 FPS to browser).

### 1.7 Browser UI

A new collapsible "Camera" card in `index.html`:

```html
<div class="card" id="camera-card">
  <h3>Robot Camera</h3>
  <img id="camera-feed" width="480" height="320" />
</div>
```

Updated on each WebSocket message:

```javascript
if (data.camera_rgb) {
    document.getElementById('camera-feed').src =
        'data:image/jpeg;base64,' + data.camera_rgb;
}
```

### Phase 1 Verification

| Check | Command / Action | Expected |
|-------|-----------------|----------|
| Topic exists | `ros2 topic list` in sidecar | `/camera/image/compressed` listed |
| Data flowing | `ros2 topic hz /camera/image/compressed` | 8–12 Hz |
| Message format | `ros2 topic echo /camera/image/compressed --no-arr` | `format: jpeg`, data length ~20000 |
| Dashboard | Open `http://<IP>:8080` | Camera panel shows updating robot-eye view |
| GPU load | `nvidia-smi` | Utilization < 80% |

---

## Phase 2: Semantic Segmentation Overlay

Isaac Sim's RTX renderer knows what every pixel represents. By tagging scene objects with semantic class labels, we get a pixel-perfect segmentation mask with **zero ML inference cost** — the renderer already has this information internally.

### 2.1 Semantic Class Labels

Tag every arena prim with a semantic class using the USD Semantics API:

```python
from pxr import Semantics

def tag_semantic(prim_path, class_name):
    prim = stage.GetPrimAtPath(prim_path)
    sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
    sem.CreateSemanticTypeAttr().Set("class")
    sem.CreateSemanticDataAttr().Set(class_name)
```

| Prim(s) | Class | Colour (on mask) |
|---------|-------|------------------|
| `/World/defaultGroundPlane` | `floor` | Grey (128, 128, 128) |
| `/World/Arena/WallN,S,E,W` | `wall` | Blue (70, 130, 230) |
| `/World/Arena/P1–P5` | `obstacle` | Red (230, 70, 70) |
| `/World/Jetbot` | `robot` | Green (70, 230, 70) |

### 2.2 Segmentation Annotator

Attach to the same RenderProduct used for RGB:

```python
seg_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
seg_annot.attach([render_product])
```

Output: `uint32[H, W]` array of class IDs, plus a `info["idToLabels"]` dict mapping IDs to semantic labels.

### 2.3 Colour Mapping and Publishing

In the `_camera_step` callback (every 12th tick, ~5 FPS):

```python
seg_data = seg_annot.get_data()
class_ids = seg_data["data"]               # uint32 [320, 480]
id_to_label = seg_data["info"]["idToLabels"]

# Map to colours
CLASS_COLOURS = {
    "floor":    (128, 128, 128),
    "wall":     (70, 130, 230),
    "obstacle": (230, 70, 70),
    "robot":    (70, 230, 70),
    "UNLABELLED": (0, 0, 0),
}

colour_mask = np.zeros((*class_ids.shape, 3), dtype=np.uint8)
for cid, info in id_to_label.items():
    label = info.get("class", "UNLABELLED")
    colour_mask[class_ids == int(cid)] = CLASS_COLOURS.get(label, (0, 0, 0))

# Resize to 240×160 and PNG encode
seg_img = Image.fromarray(colour_mask).resize((240, 160), Image.NEAREST)
buf = BytesIO()
seg_img.save(buf, format='PNG')

msg = CompressedImage()
msg.format = "png"
msg.data = buf.getvalue()                  # ~5-10 KB
seg_pub.publish(msg)
```

PNG format is essential — JPEG lossy compression creates artefacts at class boundaries, corrupting the mask.

### 2.4 Dashboard Overlay

The browser composites the segmentation mask over the camera feed:

```javascript
// Toggle modes: "rgb" | "overlay" | "segmentation"
let segMode = "overlay";
let segOpacity = 0.4;

function drawCameraFrame() {
    const ctx = cameraCanvas.getContext('2d');
    ctx.drawImage(rgbImage, 0, 0, 480, 320);

    if (segMode !== "rgb" && segImage.complete) {
        ctx.globalAlpha = segMode === "segmentation" ? 1.0 : segOpacity;
        ctx.drawImage(segImage, 0, 0, 480, 320);  // upscale 240×160 → 480×320
        ctx.globalAlpha = 1.0;
    }
}
```

Controls:
- **Toggle button**: RGB only → Overlay → Segmentation only
- **Opacity slider**: 0% – 100% when in overlay mode
- **Legend**: Colour-coded class labels below the canvas

### Phase 2 Verification

| Check | Expected |
|-------|----------|
| `ros2 topic list` | `/camera/segmentation/compressed` listed |
| `ros2 topic hz /camera/segmentation/compressed` | ~5 Hz |
| Dashboard overlay toggle | Three modes cycle correctly |
| Class colours | Walls are blue, obstacles red, floor grey |
| Boundary quality | Sharp edges between classes (no JPEG blur) |

---

## Phase 3: Object Detection (Bounding Boxes)

Isaac Sim's `bounding_box_2d` annotator returns axis-aligned bounding boxes for every visible object, with semantic labels — perfect ground-truth detections at zero compute cost.

### 3.1 Bounding Box Annotator

```python
bbox_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
bbox_annot.attach([render_product])
```

Output per visible object:

```python
{
    "semanticLabel": "obstacle",
    "x_min": 120, "y_min": 85,
    "x_max": 210, "y_max": 195,
    "occlusionRatio": 0.0
}
```

### 3.2 Publish as JSON

Publish on `/camera/detections` using `std_msgs/String` (avoids adding `vision_msgs` dependency):

```python
import json
from std_msgs.msg import String

bbox_data = bbox_annot.get_data()
detections = []
for det in bbox_data["data"]:
    detections.append({
        "label": det["semanticLabel"],
        "bbox": [int(det["x_min"]), int(det["y_min"]),
                 int(det["x_max"]), int(det["y_max"])],
        "confidence": 1.0,   # ground truth — always perfect
    })

msg = String()
msg.data = json.dumps({"detections": detections, "width": 480, "height": 320})
det_pub.publish(msg)
```

### 3.3 Dashboard Rendering

Draw coloured rectangles and labels on the camera canvas:

```javascript
function drawDetections(detections, imgWidth, imgHeight) {
    const ctx = cameraCanvas.getContext('2d');
    const sx = cameraCanvas.width / imgWidth;
    const sy = cameraCanvas.height / imgHeight;

    const COLOURS = {
        wall: '#4682E6', obstacle: '#E64646',
        floor: '#808080', robot: '#46E646',
    };

    for (const det of detections) {
        const [x1, y1, x2, y2] = det.bbox;
        const colour = COLOURS[det.label] || '#FFFFFF';

        ctx.strokeStyle = colour;
        ctx.lineWidth = 2;
        ctx.strokeRect(x1 * sx, y1 * sy, (x2 - x1) * sx, (y2 - y1) * sy);

        ctx.fillStyle = colour;
        ctx.font = '12px monospace';
        ctx.fillText(det.label, x1 * sx + 2, y1 * sy - 4);
    }
}
```

### Phase 3 Verification

| Check | Expected |
|-------|----------|
| `ros2 topic echo /camera/detections` | JSON with label + bbox arrays |
| Dashboard boxes | Coloured rectangles around visible walls and obstacles |
| Labels | Class name text above each box |
| Consistency | Boxes align with objects in the RGB image |

---

## Phase 4 (Optional): YOLO ML Detection in Sidecar

As a future extension, run a real ML object detector alongside ground truth — useful for demonstrating the sim-to-real workflow where a model trained on synthetic data is evaluated against simulator labels.

```
ros2-sidecar container
  └─ Subscribe /camera/image/compressed
  └─ JPEG decode → OpenCV → YOLOv8n inference (~30ms on CPU)
  └─ Publish /camera/detections_ml (std_msgs/String, same JSON format)
```

The dashboard would show both ground-truth (from Isaac Sim, solid lines) and ML predictions (from YOLO, dashed lines) simultaneously, making it possible to compare detection accuracy visually.

This requires adding `ultralytics` and `opencv-python-headless` to the sidecar Docker image (~500 MB).

---

## Technical Decisions

### Why These Choices?

| Decision | Rationale |
|----------|-----------|
| **480×320 resolution** | ~20 KB/frame JPEG. Enough detail to see obstacles; low enough for UDP transport over Docker networking. |
| **ROS2 CompressedImage transport** | Mirrors real robot camera pipelines. Educational value over simpler file-sharing. |
| **JPEG for RGB, PNG for segmentation** | JPEG lossy compression produces artefacts at sharp colour boundaries, corrupting segmentation class edges. PNG preserves them exactly. |
| **Ground-truth detection first** | Isaac Sim's `bounding_box_2d` gives perfect labels at zero GPU cost. YOLO is an optional comparison extension. |
| **rclpy publisher inside Isaac Sim** | No OmniGraph image publisher node exists in 5.1.0. Standalone rclpy node is the supported path. |
| **Segmentation at 240×160** | Half resolution saves 75% bandwidth. Class boundaries are still sharp (nearest-neighbor upscale in browser). |
| **~10 FPS capture, ~2 FPS to browser** | Capture fast enough for smooth recording; browser throttled to save WebSocket bandwidth. |

### Why Not These Alternatives?

| Alternative | Why Not |
|-------------|---------|
| **OmniGraph camera node** | Does not exist in Isaac Sim 5.1.0. The ROS2 bridge only has publisher nodes for Odometry, LaserScan, Clock, TF, and JointState. |
| **Raw Image (uncompressed)** | A single 480×320 RGB frame is 460 KB. At 10 FPS that's 4.6 MB/s — exceeds DDS UDP bandwidth in Docker. |
| **WebRTC for camera** | Already used for the main viewport. Adding a second WebRTC stream is complex and would compete for the same GPU encoder. |
| **Viewport capture** | Would capture the orbit camera (third-person), not the robot-eye view. Also interferes with the main rendering pipeline. |
| **Higher resolution** | A4000 has headroom, but the bottleneck is DDS over UDP in Docker networking, not GPU. Higher res → bigger JPEG → more latency for no real benefit at dashboard scale. |

---

## Risks and Mitigations

### Replicator API Availability

The plan assumes `omni.replicator.core` loads in the Isaac Sim 5.1.0 Docker image. If it does not:

**Fallback 1**: Use `omni.syntheticdata` extension directly (lower-level annotator API that Replicator wraps).

**Fallback 2**: Use `omni.kit.viewport.utility` to capture the viewport framebuffer. This gives RGB only (no segmentation/bbox), but is guaranteed to exist.

### rclpy Dual Initialization

The ROS2 bridge OmniGraph extension already initializes rclpy internally. Calling `rclpy.init()` a second time raises `RuntimeError`. The script catches this and uses `rclpy.create_node()` on the existing context.

If that also fails, the fallback is to publish via the OmniGraph `ROS2Publisher` generic node with raw byte arrays — functional but less clean.

### Bandwidth Over WebSocket

| Path | Budget |
|------|--------|
| RGB (2 FPS × 20 KB) | 40 KB/s |
| Segmentation (1 FPS × 8 KB) | 8 KB/s |
| Detections (2 FPS × 0.5 KB) | 1 KB/s |
| Existing data (lidar, odom, system) | ~5 KB/s |
| **Total** | **~54 KB/s** |

Well within typical Internet connections. Over slow links, the browser can skip frames by checking timestamps.

---

## Files to Modify

| File | Changes |
|------|---------|
| `scripts/instance/ros2_jetbot.py` | Add camera prim, enable Replicator, create RenderProduct, attach annotators, add semantic tags to arena prims, create rclpy publishers, add `_camera_step` physics callback |
| `scripts/instance/dashboard/dashboard_server.py` | Subscribe to `/camera/image/compressed`, `/camera/segmentation/compressed`, `/camera/detections`; base64 encode images; add camera fields to WebSocket broadcast |
| `scripts/instance/dashboard/index.html` | Add camera panel with compositing canvas, segmentation toggle/opacity slider, bounding box renderer, class legend |
| `scripts/instance/dashboard/Dockerfile` | Possibly add `ros-humble-std-msgs` (may already be present via dependencies) |

---

## New ROS2 Topics After Implementation

| Topic | Type | Publisher | Rate | QoS |
|-------|------|-----------|------|-----|
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | Isaac Sim (rclpy) | ~10 Hz | BEST_EFFORT |
| `/camera/segmentation/compressed` | `sensor_msgs/CompressedImage` | Isaac Sim (rclpy) | ~5 Hz | BEST_EFFORT |
| `/camera/detections` | `std_msgs/String` | Isaac Sim (rclpy) | ~5 Hz | BEST_EFFORT |
| `/camera/detections_ml` | `std_msgs/String` | Sidecar (YOLO) | ~5 Hz | BEST_EFFORT |

These join the existing four topics (`/odom`, `/scan`, `/cmd_vel`, `/clock`), bringing the system to 7–8 active topics.

---

## Dashboard After Implementation

```
┌──────────────────────────────────────────────────────────────┐
│  Isaac Sim + ROS2 Dashboard                                  │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─── Robot Camera ──────────────────────────────────────┐   │
│  │                                                        │   │
│  │   ┌──────────────────────────────────────────────┐    │   │
│  │   │                                              │    │   │
│  │   │   Robot-eye JPEG feed (480×320)              │    │   │
│  │   │   + segmentation overlay (toggleable)        │    │   │
│  │   │   + bounding box rectangles + labels         │    │   │
│  │   │                                              │    │   │
│  │   └──────────────────────────────────────────────┘    │   │
│  │                                                        │   │
│  │   [RGB] [Overlay] [Segmentation]    Opacity: ─●──── │   │
│  │                                                        │   │
│  │   Legend: ■ floor  ■ wall  ■ obstacle  ■ robot       │   │
│  │                                                        │   │
│  └────────────────────────────────────────────────────────┘   │
│                                                              │
│  ┌─── Lidar ──────────┐  ┌─── Robot State ──────────────┐   │
│  │ (existing polar     │  │ Position, velocity, cmd_vel  │   │
│  │  plot overlay)      │  │ Topic rates and ages         │   │
│  └─────────────────────┘  └──────────────────────────────┘   │
│                                                              │
│  ┌─── System ─────────────────────────────────────────────┐  │
│  │ GPU temp/util, CPU load, containers                     │  │
│  └─────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```
