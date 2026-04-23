import { useCallback } from "react";
import { API_URL } from "../config";

async function apiFetch<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${API_URL}${path}`, {
    headers: { "Content-Type": "application/json" },
    ...init,
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(`API ${res.status}: ${text}`);
  }
  return res.json() as Promise<T>;
}

export function useApi() {
  const getForklifts = useCallback(() => apiFetch("/forklifts"), []);
  const getOrders = useCallback(() => apiFetch("/orders"), []);
  const getMetrics = useCallback(() => apiFetch("/metrics"), []);

  const createOrder = useCallback(
    (items: string[], priority: number) =>
      apiFetch("/orders", {
        method: "POST",
        body: JSON.stringify({ items, priority }),
      }),
    []
  );

  const pauseForklift = useCallback(
    (id: string) => apiFetch(`/forklifts/${encodeURIComponent(id)}/pause`, { method: "PUT" }),
    []
  );

  const resumeForklift = useCallback(
    (id: string) => apiFetch(`/forklifts/${encodeURIComponent(id)}/resume`, { method: "PUT" }),
    []
  );

  const cancelTask = useCallback(
    (taskId: string) => apiFetch(`/tasks/${encodeURIComponent(taskId)}`, { method: "DELETE" }),
    []
  );

  const resetSimulation = useCallback(
    () => apiFetch("/reset", { method: "POST" }),
    []
  );

  const updateConfig = useCallback(
    (strategy: string) =>
      apiFetch("/config", {
        method: "POST",
        body: JSON.stringify({ dispatch_strategy: strategy }),
      }),
    []
  );

  return {
    getForklifts,
    getOrders,
    getMetrics,
    createOrder,
    pauseForklift,
    resumeForklift,
    cancelTask,
    resetSimulation,
    updateConfig,
  };
}
