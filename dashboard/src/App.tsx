import { useEffect } from "react";
import { useWebSocket } from "./hooks/useWebSocket";
import { useApi } from "./hooks/useApi";
import { useOrderStore } from "./stores/orderStore";
import { useWarehouseStore } from "./stores/warehouseStore";
import { WarehouseMap } from "./components/WarehouseMap/WarehouseMap";
import { OrderQueue } from "./components/OrderQueue/OrderQueue";
import { FleetStatus } from "./components/FleetStatus/FleetStatus";
import { MetricsPanel } from "./components/MetricsPanel/MetricsPanel";
import { ConfigPanel } from "./components/ConfigPanel/ConfigPanel";
import type { ForkliftStatus } from "./types/forklift";
import type { Order } from "./types/order";

export default function App() {
  useWebSocket();
  const { getForklifts, getOrders } = useApi();
  const setForklifts = useWarehouseStore((s) => s.setForklifts);
  const setOrders = useOrderStore((s) => s.setOrders);

  // Load initial state from REST on mount
  useEffect(() => {
    const load = async () => {
      try {
        const [fl, ord] = await Promise.all([getForklifts(), getOrders()]);
        setForklifts(fl as ForkliftStatus[]);
        setOrders(ord as Order[]);
      } catch {
        /* API not ready yet */
      }
    };
    load();
  }, [getForklifts, getOrders, setForklifts, setOrders]);

  return (
    <div
      style={{
        minHeight: "100vh",
        background: "#0f0f23",
        color: "#e2e8f0",
        fontFamily: "'Inter', -apple-system, sans-serif",
      }}
    >
      {/* Header */}
      <header
        style={{
          padding: "12px 24px",
          background: "#1a1a2e",
          borderBottom: "1px solid #2d3748",
          display: "flex",
          justifyContent: "space-between",
          alignItems: "center",
        }}
      >
        <h1 style={{ margin: 0, fontSize: 20 }}>Warehouse Digital Twin</h1>
        <span style={{ color: "#a0aec0", fontSize: 13 }}>Phase 5 Dashboard</span>
      </header>

      {/* Main Grid */}
      <main
        style={{
          display: "grid",
          gridTemplateColumns: "1fr 360px",
          gridTemplateRows: "auto auto",
          gap: 16,
          padding: 16,
          maxWidth: 1440,
          margin: "0 auto",
        }}
      >
        {/* Map */}
        <div style={{ gridColumn: "1 / 2", gridRow: "1 / 2" }}>
          <WarehouseMap />
        </div>

        {/* Fleet + Config sidebar */}
        <div style={{ gridColumn: "2 / 3", gridRow: "1 / 3", overflowY: "auto" }}>
          <FleetStatus />
          <ConfigPanel />
        </div>

        {/* Metrics */}
        <div style={{ gridColumn: "1 / 2", gridRow: "2 / 3" }}>
          <MetricsPanel />
        </div>
      </main>

      {/* Order Queue (full width) */}
      <section style={{ padding: "0 16px 24px", maxWidth: 1440, margin: "0 auto" }}>
        <OrderQueue />
      </section>
    </div>
  );
}
