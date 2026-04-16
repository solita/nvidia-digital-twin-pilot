import omni.usd
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# Replace with your warehouse prim path
prim = stage.GetPrimAtPath("/World/warehouse")
bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
bbox = bbox_cache.ComputeWorldBound(prim)
r = bbox.GetRange()
print(f"Min: {r.GetMin()}")
print(f"Max: {r.GetMax()}")
print(f"Size: {r.GetMax() - r.GetMin()}")