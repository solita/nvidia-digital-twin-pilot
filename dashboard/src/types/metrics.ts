export interface Metrics {
  total_orders: number;
  completed_orders: number;
  in_progress_orders: number;
  pending_orders: number;
  fleet_size: number;
  idle_forklifts: number;
  active_forklifts: number;
  dispatch_strategy: string;
}
