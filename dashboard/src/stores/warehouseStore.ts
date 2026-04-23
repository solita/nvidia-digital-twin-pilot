import { create } from "zustand";
import type { ForkliftStatus } from "../types/forklift";

interface WarehouseState {
  forklifts: Record<string, ForkliftStatus>;
  updateForklift: (status: ForkliftStatus) => void;
  setForklifts: (list: ForkliftStatus[]) => void;
  resetForklifts: () => void;
}

export const useWarehouseStore = create<WarehouseState>((set) => ({
  forklifts: {},

  updateForklift: (status) =>
    set((state) => ({
      forklifts: { ...state.forklifts, [status.forklift_id]: status },
    })),

  setForklifts: (list) =>
    set(() => ({
      forklifts: Object.fromEntries(list.map((f) => [f.forklift_id, f])),
    })),

  resetForklifts: () => set({ forklifts: {} }),
}));
