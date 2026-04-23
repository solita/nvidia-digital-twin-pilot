export enum ForkliftState {
  IDLE = 0,
  NAVIGATING_TO_SHELF = 1,
  PICKING = 2,
  NAVIGATING_TO_DOCK = 3,
  DROPPING = 4,
  ERROR = 5,
  RECOVERING = 6,
}

export interface ForkliftStatus {
  forklift_id: string;
  state: ForkliftState;
  pose: { x: number; y: number; z: number; yaw: number };
  battery_level: number;
  current_task_id: string | null;
  paused: boolean;
}

export const FORKLIFT_STATE_LABELS: Record<ForkliftState, string> = {
  [ForkliftState.IDLE]: "Idle",
  [ForkliftState.NAVIGATING_TO_SHELF]: "Navigating to Shelf",
  [ForkliftState.PICKING]: "Picking",
  [ForkliftState.NAVIGATING_TO_DOCK]: "Navigating to Dock",
  [ForkliftState.DROPPING]: "Dropping",
  [ForkliftState.ERROR]: "Error",
  [ForkliftState.RECOVERING]: "Recovering",
};

export const FORKLIFT_STATE_COLORS: Record<ForkliftState, string> = {
  [ForkliftState.IDLE]: "#22c55e",
  [ForkliftState.NAVIGATING_TO_SHELF]: "#3b82f6",
  [ForkliftState.PICKING]: "#eab308",
  [ForkliftState.NAVIGATING_TO_DOCK]: "#3b82f6",
  [ForkliftState.DROPPING]: "#eab308",
  [ForkliftState.ERROR]: "#ef4444",
  [ForkliftState.RECOVERING]: "#f97316",
};
