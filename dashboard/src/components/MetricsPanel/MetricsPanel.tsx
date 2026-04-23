import { useEffect } from "react";
import {
  BarChart,
  Bar,
  XAxis,
  YAxis,
  Tooltip,
  ResponsiveContainer,
  PieChart,
  Pie,
  Cell,
} from "recharts";
import { useMetricsStore } from "../../stores/metricsStore";
import { useApi } from "../../hooks/useApi";
import type { Metrics } from "../../types/metrics";

const COLORS = ["#22c55e", "#3b82f6", "#eab308", "#6b7280"];

export function MetricsPanel() {
  const { getMetrics } = useApi();
  const metrics = useMetricsStore((s) => s.current);
  const setMetrics = useMetricsStore((s) => s.setMetrics);

  useEffect(() => {
    const poll = async () => {
      try {
        const m = (await getMetrics()) as Metrics;
        setMetrics(m);
      } catch {
        /* API not ready */
      }
    };
    poll();
    const id = setInterval(poll, 2000);
    return () => clearInterval(id);
  }, [getMetrics, setMetrics]);

  if (!metrics) {
    return (
      <div style={{ padding: 16, color: "#718096" }}>Loading metrics...</div>
    );
  }

  const orderData = [
    { name: "Completed", value: metrics.completed_orders },
    { name: "In Progress", value: metrics.in_progress_orders },
    { name: "Pending", value: metrics.pending_orders },
  ];

  const fleetData = [
    { name: "Idle", count: metrics.idle_forklifts },
    { name: "Active", count: metrics.active_forklifts },
  ];

  return (
    <div style={{ padding: 16 }}>
      <h3 style={{ margin: "0 0 12px", color: "#e2e8f0" }}>Metrics</h3>

      {/* KPI cards */}
      <div style={{ display: "grid", gridTemplateColumns: "repeat(4, 1fr)", gap: 12, marginBottom: 16 }}>
        <KpiCard label="Total Orders" value={metrics.total_orders} />
        <KpiCard label="Completed" value={metrics.completed_orders} color="#22c55e" />
        <KpiCard label="Active Forklifts" value={metrics.active_forklifts} color="#3b82f6" />
        <KpiCard label="Strategy" value={metrics.dispatch_strategy} />
      </div>

      {/* Charts */}
      <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 16 }}>
        <div style={{ background: "#2d3748", borderRadius: 8, padding: 16 }}>
          <h4 style={{ color: "#a0aec0", margin: "0 0 8px", fontSize: 13 }}>Order Status</h4>
          <ResponsiveContainer width="100%" height={180}>
            <PieChart>
              <Pie data={orderData} dataKey="value" nameKey="name" cx="50%" cy="50%" outerRadius={70}>
                {orderData.map((_, i) => (
                  <Cell key={i} fill={COLORS[i % COLORS.length]} />
                ))}
              </Pie>
              <Tooltip />
            </PieChart>
          </ResponsiveContainer>
        </div>
        <div style={{ background: "#2d3748", borderRadius: 8, padding: 16 }}>
          <h4 style={{ color: "#a0aec0", margin: "0 0 8px", fontSize: 13 }}>Fleet Utilization</h4>
          <ResponsiveContainer width="100%" height={180}>
            <BarChart data={fleetData}>
              <XAxis dataKey="name" tick={{ fill: "#a0aec0", fontSize: 12 }} />
              <YAxis allowDecimals={false} tick={{ fill: "#a0aec0", fontSize: 12 }} />
              <Tooltip />
              <Bar dataKey="count" fill="#3b82f6" radius={[4, 4, 0, 0]} />
            </BarChart>
          </ResponsiveContainer>
        </div>
      </div>
    </div>
  );
}

function KpiCard({ label, value, color }: { label: string; value: string | number; color?: string }) {
  return (
    <div style={{ background: "#2d3748", borderRadius: 8, padding: 12, textAlign: "center" }}>
      <div style={{ fontSize: 24, fontWeight: 700, color: color ?? "#e2e8f0" }}>{value}</div>
      <div style={{ fontSize: 11, color: "#a0aec0", marginTop: 4 }}>{label}</div>
    </div>
  );
}
