export interface Order {
  id: string;
  items: string[];
  priority: number;
  status: "pending" | "in_progress" | "completed" | "cancelled";
  assigned_forklift: string | null;
  created_at: string;
  completed_at: string | null;
}
