import { Stage, Layer, Rect, RegularPolygon, Text, Line } from "react-konva";
import { useWarehouseStore } from "../../stores/warehouseStore";
import { ForkliftState, FORKLIFT_STATE_COLORS } from "../../types/forklift";

const WAREHOUSE_W = 50; // meters
const WAREHOUSE_H = 30;
const CANVAS_W = 800;
const CANVAS_H = 480;
const SCALE = Math.min(CANVAS_W / WAREHOUSE_W, CANVAS_H / WAREHOUSE_H);

function toCanvas(x: number, y: number): [number, number] {
  return [x * SCALE, (WAREHOUSE_H - y) * SCALE]; // flip Y
}

export function WarehouseMap() {
  const forklifts = useWarehouseStore((s) => Object.values(s.forklifts));

  // Shelf racks (placeholder positions)
  const shelves = [
    { x: 8, y: 5, w: 4, h: 20, label: "Rack A" },
    { x: 18, y: 5, w: 4, h: 20, label: "Rack B" },
    { x: 28, y: 5, w: 4, h: 20, label: "Rack C" },
    { x: 38, y: 5, w: 4, h: 20, label: "Rack D" },
  ];

  // Loading docks
  const docks = [
    { x: 0, y: 26, w: 6, h: 3, label: "Dock 1" },
    { x: 44, y: 26, w: 6, h: 3, label: "Dock 2" },
  ];

  return (
    <div style={{ border: "1px solid #333", borderRadius: 8, overflow: "hidden" }}>
      <Stage width={CANVAS_W} height={CANVAS_H}>
        <Layer>
          {/* Background */}
          <Rect x={0} y={0} width={CANVAS_W} height={CANVAS_H} fill="#1a1a2e" />

          {/* Grid lines */}
          {Array.from({ length: WAREHOUSE_W + 1 }, (_, i) => (
            <Line
              key={`vg-${i}`}
              points={[i * SCALE, 0, i * SCALE, CANVAS_H]}
              stroke="#ffffff10"
              strokeWidth={0.5}
            />
          ))}
          {Array.from({ length: WAREHOUSE_H + 1 }, (_, i) => (
            <Line
              key={`hg-${i}`}
              points={[0, i * SCALE, CANVAS_W, i * SCALE]}
              stroke="#ffffff10"
              strokeWidth={0.5}
            />
          ))}

          {/* Shelves */}
          {shelves.map((s) => {
            const [cx, cy] = toCanvas(s.x, s.y + s.h);
            return (
              <Rect
                key={s.label}
                x={cx}
                y={cy}
                width={s.w * SCALE}
                height={s.h * SCALE}
                fill="#4a5568"
                cornerRadius={2}
              />
            );
          })}
          {shelves.map((s) => {
            const [cx, cy] = toCanvas(s.x + s.w / 2, s.y + s.h / 2);
            return (
              <Text
                key={`${s.label}-txt`}
                x={cx - 20}
                y={cy - 6}
                text={s.label}
                fontSize={11}
                fill="#a0aec0"
                align="center"
                width={40}
              />
            );
          })}

          {/* Docks */}
          {docks.map((d) => {
            const [cx, cy] = toCanvas(d.x, d.y + d.h);
            return (
              <Rect
                key={d.label}
                x={cx}
                y={cy}
                width={d.w * SCALE}
                height={d.h * SCALE}
                fill="#2b6cb0"
                cornerRadius={2}
              />
            );
          })}
          {docks.map((d) => {
            const [cx, cy] = toCanvas(d.x + d.w / 2, d.y + d.h / 2);
            return (
              <Text
                key={`${d.label}-txt`}
                x={cx - 20}
                y={cy - 6}
                text={d.label}
                fontSize={11}
                fill="#bee3f8"
                align="center"
                width={40}
              />
            );
          })}

          {/* Forklifts */}
          {forklifts.map((f) => {
            const [cx, cy] = toCanvas(f.pose.x, f.pose.y);
            const color = FORKLIFT_STATE_COLORS[f.state as ForkliftState] ?? "#888";
            return (
              <RegularPolygon
                key={f.forklift_id}
                x={cx}
                y={cy}
                sides={3}
                radius={10}
                fill={color}
                rotation={-(f.pose.yaw * 180) / Math.PI + 90}
                stroke="#fff"
                strokeWidth={1}
              />
            );
          })}
          {forklifts.map((f) => {
            const [cx, cy] = toCanvas(f.pose.x, f.pose.y);
            return (
              <Text
                key={`${f.forklift_id}-lbl`}
                x={cx - 20}
                y={cy + 12}
                text={f.forklift_id.replace("forklift_", "FL")}
                fontSize={10}
                fill="#fff"
                align="center"
                width={40}
              />
            );
          })}
        </Layer>
      </Stage>
    </div>
  );
}
