import { useOrderStore } from "../../stores/orderStore";

const STATUS_COLORS: Record<string, string> = {
  pending: "#eab308",
  in_progress: "#3b82f6",
  completed: "#22c55e",
  cancelled: "#6b7280",
};

export function OrderQueue() {
  const orders = useOrderStore((s) => Object.values(s.orders));

  const sorted = [...orders].sort((a, b) => {
    // Active first, then by priority (desc), then by creation time
    const statusOrder = { in_progress: 0, pending: 1, completed: 2, cancelled: 3 };
    const sa = statusOrder[a.status] ?? 4;
    const sb = statusOrder[b.status] ?? 4;
    if (sa !== sb) return sa - sb;
    if (a.priority !== b.priority) return b.priority - a.priority;
    return new Date(b.created_at).getTime() - new Date(a.created_at).getTime();
  });

  return (
    <div style={{ padding: 16 }}>
      <h3 style={{ margin: "0 0 12px", color: "#e2e8f0" }}>Orders ({orders.length})</h3>
      <table style={{ width: "100%", borderCollapse: "collapse", fontSize: 13 }}>
        <thead>
          <tr style={{ color: "#a0aec0", textAlign: "left" }}>
            <th style={{ padding: "4px 8px" }}>ID</th>
            <th style={{ padding: "4px 8px" }}>Items</th>
            <th style={{ padding: "4px 8px" }}>Priority</th>
            <th style={{ padding: "4px 8px" }}>Status</th>
            <th style={{ padding: "4px 8px" }}>Forklift</th>
          </tr>
        </thead>
        <tbody>
          {sorted.map((o) => (
            <tr key={o.id} style={{ borderTop: "1px solid #2d3748" }}>
              <td style={{ padding: "6px 8px", color: "#cbd5e0", fontFamily: "monospace" }}>
                {o.id.slice(0, 8)}
              </td>
              <td style={{ padding: "6px 8px", color: "#cbd5e0" }}>{o.items.join(", ")}</td>
              <td style={{ padding: "6px 8px", color: "#cbd5e0" }}>{o.priority}</td>
              <td style={{ padding: "6px 8px" }}>
                <span
                  style={{
                    color: STATUS_COLORS[o.status] ?? "#888",
                    fontWeight: 600,
                    textTransform: "uppercase",
                    fontSize: 11,
                  }}
                >
                  {o.status}
                </span>
              </td>
              <td style={{ padding: "6px 8px", color: "#a0aec0" }}>
                {o.assigned_forklift ?? "—"}
              </td>
            </tr>
          ))}
          {sorted.length === 0 && (
            <tr>
              <td colSpan={5} style={{ padding: 16, color: "#718096", textAlign: "center" }}>
                No orders yet
              </td>
            </tr>
          )}
        </tbody>
      </table>
    </div>
  );
}
