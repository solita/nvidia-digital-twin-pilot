import { create } from "zustand";
import type { Metrics } from "../types/metrics";

interface MetricsState {
  current: Metrics | null;
  history: Metrics[];
  setMetrics: (m: Metrics) => void;
  reset: () => void;
}

const MAX_HISTORY = 120; // ~2 minutes at 1/s polling

export const useMetricsStore = create<MetricsState>((set) => ({
  current: null,
  history: [],

  setMetrics: (m) =>
    set((state) => ({
      current: m,
      history: [...state.history.slice(-(MAX_HISTORY - 1)), m],
    })),

  reset: () => set({ current: null, history: [] }),
}));
