"""Task dispatcher — assigns orders to forklifts.

Supports two strategies:
  - "nearest": Greedy assignment to the nearest idle forklift.
  - "batched": Collects orders over a window, then solves an optimal
    assignment via the Hungarian algorithm (scipy).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from scipy.optimize import linear_sum_assignment

if TYPE_CHECKING:
    from main import ForkliftState


def euclidean_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


# ── Nearest-available (greedy) ────────────────────────────────────────────────

def nearest_assign(
    order_positions: list[tuple[str, tuple[float, float]]],
    forklift_positions: list[tuple[str, tuple[float, float]]],
) -> dict[str, str]:
    """Return {forklift_id: order_id} mapping via greedy nearest assignment."""
    assignments: dict[str, str] = {}
    available = list(forklift_positions)

    for order_id, order_pos in order_positions:
        if not available:
            break
        best_idx = min(range(len(available)), key=lambda i: euclidean_distance(available[i][1], order_pos))
        fl_id, _ = available.pop(best_idx)
        assignments[fl_id] = order_id

    return assignments


# ── Batched optimal (Hungarian) ───────────────────────────────────────────────

def batch_assign(
    order_positions: list[tuple[str, tuple[float, float]]],
    forklift_positions: list[tuple[str, tuple[float, float]]],
) -> dict[str, str]:
    """Return {forklift_id: order_id} via optimal cost-matrix assignment."""
    n_forklifts = len(forklift_positions)
    n_orders = len(order_positions)
    if n_forklifts == 0 or n_orders == 0:
        return {}

    n = max(n_forklifts, n_orders)
    cost = np.full((n, n), 1e9)

    for i, (_, f_pos) in enumerate(forklift_positions):
        for j, (_, o_pos) in enumerate(order_positions):
            cost[i][j] = euclidean_distance(f_pos, o_pos)

    row_ind, col_ind = linear_sum_assignment(cost)
    return {
        forklift_positions[i][0]: order_positions[j][0]
        for i, j in zip(row_ind, col_ind)
        if i < n_forklifts and j < n_orders
    }
