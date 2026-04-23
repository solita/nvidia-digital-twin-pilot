import { create } from "zustand";
import type { Order } from "../types/order";

interface OrderState {
  orders: Record<string, Order>;
  updateOrder: (order: Order) => void;
  setOrders: (list: Order[]) => void;
  resetOrders: () => void;
}

export const useOrderStore = create<OrderState>((set) => ({
  orders: {},

  updateOrder: (order) =>
    set((state) => ({
      orders: { ...state.orders, [order.id]: order },
    })),

  setOrders: (list) =>
    set(() => ({
      orders: Object.fromEntries(list.map((o) => [o.id, o])),
    })),

  resetOrders: () => set({ orders: {} }),
}));
