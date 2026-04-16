"""
dashboard.py — Real-time stats dashboard for the forklift warehouse simulation.

Displays live data from lidar_subscriber.py:
  - Forklift position and speed
  - Nearest obstacle distance
  - Stop/go state
  - LIDAR point cloud (via Foxglove or custom web UI)

Options (pick one):
  A) Foxglove Studio — connects directly to ROS 2 topics, zero UI code needed.
     Run: foxglove-studio (or open https://app.foxglove.dev)
     Point it at the ROS 2 WebSocket bridge running on the Brev instance.

  B) FastAPI + web page — lightweight custom dashboard.
     Run: uvicorn dashboard:app --host 0.0.0.0 --port 8080
     Expose port 8080 in the Brev dashboard to your IP.


NEED TO THINK MORE ON THE DASHBOARD APPROACH
"""
from __future__ import annotations

# TODO: decide between Foxglove (Option A) or FastAPI (Option B)
# TODO: implement data ingestion from lidar_subscriber.py
