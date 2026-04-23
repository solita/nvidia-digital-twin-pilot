import type { ForkliftStatus } from "./forklift";
import type { Order } from "./order";
import type { Metrics } from "./metrics";

export type WSMessage =
  | { type: "forklift_update"; data: ForkliftStatus }
  | { type: "order_update"; data: Order }
  | { type: "task_status"; data: { task_id: string; status: string; forklift_id: string } }
  | { type: "metric_update"; data: Metrics }
  | { type: "collision"; data: { forklift_id: string; timestamp: string } }
  | { type: "snapshot"; data: { forklifts: ForkliftStatus[] } }
  | { type: "reset"; data: Record<string, never> };
