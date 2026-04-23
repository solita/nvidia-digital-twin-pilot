import { useState } from "react";
import { useApi } from "../../hooks/useApi";

export function ConfigPanel() {
  const { createOrder, resetSimulation, updateConfig } = useApi();
  const [items, setItems] = useState("box_001");
  const [priority, setPriority] = useState(1);
  const [strategy, setStrategy] = useState("nearest");

  const handleCreateOrder = async () => {
    const itemList = items
      .split(",")
      .map((s) => s.trim())
      .filter(Boolean);
    if (itemList.length === 0) return;
    await createOrder(itemList, priority);
    setItems("");
  };

  const handleStrategyChange = async (s: string) => {
    setStrategy(s);
    await updateConfig(s);
  };

  return (
    <div style={{ padding: 16 }}>
      <h3 style={{ margin: "0 0 12px", color: "#e2e8f0" }}>Controls</h3>

      {/* New Order */}
      <div style={{ background: "#2d3748", borderRadius: 8, padding: 12, marginBottom: 12 }}>
        <h4 style={{ color: "#a0aec0", margin: "0 0 8px", fontSize: 13 }}>New Order</h4>
        <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
          <input
            value={items}
            onChange={(e) => setItems(e.target.value)}
            placeholder="Items (comma-separated)"
            style={{
              flex: 1,
              minWidth: 150,
              background: "#1a202c",
              border: "1px solid #4a5568",
              borderRadius: 4,
              color: "#e2e8f0",
              padding: "6px 10px",
              fontSize: 13,
            }}
          />
          <select
            value={priority}
            onChange={(e) => setPriority(Number(e.target.value))}
            style={{
              background: "#1a202c",
              border: "1px solid #4a5568",
              borderRadius: 4,
              color: "#e2e8f0",
              padding: "6px 10px",
              fontSize: 13,
            }}
          >
            <option value={1}>Priority 1</option>
            <option value={2}>Priority 2</option>
            <option value={3}>Priority 3</option>
          </select>
          <button
            onClick={handleCreateOrder}
            style={{
              background: "#3b82f6",
              color: "#fff",
              border: "none",
              padding: "6px 16px",
              borderRadius: 4,
              cursor: "pointer",
              fontSize: 13,
            }}
          >
            Create
          </button>
        </div>
      </div>

      {/* Dispatch Strategy */}
      <div style={{ background: "#2d3748", borderRadius: 8, padding: 12, marginBottom: 12 }}>
        <h4 style={{ color: "#a0aec0", margin: "0 0 8px", fontSize: 13 }}>Dispatch Strategy</h4>
        <div style={{ display: "flex", gap: 8 }}>
          {["nearest", "batched"].map((s) => (
            <button
              key={s}
              onClick={() => handleStrategyChange(s)}
              style={{
                background: strategy === s ? "#3b82f6" : "#1a202c",
                color: "#fff",
                border: strategy === s ? "none" : "1px solid #4a5568",
                padding: "6px 16px",
                borderRadius: 4,
                cursor: "pointer",
                fontSize: 13,
                textTransform: "capitalize",
              }}
            >
              {s}
            </button>
          ))}
        </div>
      </div>

      {/* Reset */}
      <button
        onClick={() => {
          if (window.confirm("Reset simulation? All in-flight orders will be cancelled.")) {
            resetSimulation();
          }
        }}
        style={{
          background: "#ef4444",
          color: "#fff",
          border: "none",
          padding: "8px 20px",
          borderRadius: 4,
          cursor: "pointer",
          fontSize: 13,
          width: "100%",
        }}
      >
        Reset Simulation
      </button>
    </div>
  );
}
