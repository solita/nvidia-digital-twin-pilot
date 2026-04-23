import { useEffect, useRef, useCallback } from "react";
import ReconnectingWebSocket from "reconnecting-websocket";
import type { WSMessage } from "../types/websocket";
import { useWarehouseStore } from "../stores/warehouseStore";
import { useOrderStore } from "../stores/orderStore";
import { WS_URL } from "../config";

export function useWebSocket() {
  const wsRef = useRef<ReconnectingWebSocket | null>(null);
  const updateForklift = useWarehouseStore((s) => s.updateForklift);
  const setForklifts = useWarehouseStore((s) => s.setForklifts);
  const updateOrder = useOrderStore((s) => s.updateOrder);

  useEffect(() => {
    const ws = new ReconnectingWebSocket(WS_URL, [], {
      maxRetries: Infinity,
      reconnectionDelayGrowFactor: 1.5,
      maxReconnectionDelay: 10000,
      minReconnectionDelay: 1000,
    });

    ws.onmessage = (event: MessageEvent) => {
      const msg: WSMessage = JSON.parse(event.data as string);
      switch (msg.type) {
        case "forklift_update":
          updateForklift(msg.data);
          break;
        case "order_update":
          updateOrder(msg.data);
          break;
        case "snapshot":
          setForklifts(msg.data.forklifts);
          break;
        case "reset":
          // Could trigger a full refresh from REST
          break;
      }
    };

    ws.onopen = () => console.log("WebSocket connected");
    ws.onclose = () => console.log("WebSocket disconnected, reconnecting...");

    wsRef.current = ws;
    return () => ws.close();
  }, [updateForklift, updateOrder, setForklifts]);

  const send = useCallback((msg: object) => {
    wsRef.current?.send(JSON.stringify(msg));
  }, []);

  return { send };
}
