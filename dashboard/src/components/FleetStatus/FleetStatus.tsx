import { useWarehouseStore } from "../../stores/warehouseStore";
import { ForkliftState, FORKLIFT_STATE_LABELS, FORKLIFT_STATE_COLORS } from "../../types/forklift";
import { useApi } from "../../hooks/useApi";

export function FleetStatus() {
  const forklifts = useWarehouseStore((s) => Object.values(s.forklifts));
  const { pauseForklift, resumeForklift } = useApi();

  return (
    <div style={{ padding: 16 }}>
      <h3 style={{ margin: "0 0 12px", color: "#e2e8f0" }}>Fleet Status</h3>
      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 12 }}>
        {forklifts.map((f) => {
          const stateColor = FORKLIFT_STATE_COLORS[f.state as ForkliftState] ?? "#888";
          const stateLabel = FORKLIFT_STATE_LABELS[f.state as ForkliftState] ?? "Unknown";

          return (
            <div
              key={f.forklift_id}
              style={{
                background: "#2d3748",
                borderRadius: 8,
                padding: 12,
                borderLeft: `4px solid ${stateColor}`,
              }}
            >
              <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 8 }}>
                <strong style={{ color: "#e2e8f0" }}>
                  {f.forklift_id.replace("forklift_", "Forklift ")}
                </strong>
                <span style={{ color: stateColor, fontSize: 12, fontWeight: 600 }}>
                  {stateLabel}
                </span>
              </div>
              <div style={{ fontSize: 12, color: "#a0aec0", lineHeight: 1.6 }}>
                <div>
                  Position: ({f.pose.x.toFixed(1)}, {f.pose.y.toFixed(1)})
                </div>
                <div>Battery: {f.battery_level.toFixed(0)}%</div>
                <div>Task: {f.current_task_id?.slice(0, 8) ?? "None"}</div>
              </div>
              <div style={{ marginTop: 8 }}>
                {f.paused ? (
                  <button
                    onClick={() => resumeForklift(f.forklift_id)}
                    style={{
                      background: "#22c55e",
                      color: "#fff",
                      border: "none",
                      padding: "4px 12px",
                      borderRadius: 4,
                      cursor: "pointer",
                      fontSize: 12,
                    }}
                  >
                    Resume
                  </button>
                ) : (
                  <button
                    onClick={() => pauseForklift(f.forklift_id)}
                    style={{
                      background: "#ef4444",
                      color: "#fff",
                      border: "none",
                      padding: "4px 12px",
                      borderRadius: 4,
                      cursor: "pointer",
                      fontSize: 12,
                    }}
                  >
                    Pause
                  </button>
                )}
              </div>
            </div>
          );
        })}
        {forklifts.length === 0 && (
          <div style={{ color: "#718096", gridColumn: "1 / -1", textAlign: "center", padding: 24 }}>
            No forklift data yet. Waiting for connection...
          </div>
        )}
      </div>
    </div>
  );
}
