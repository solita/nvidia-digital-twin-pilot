"""SQLite models and database helpers for the warehouse manager."""

import aiosqlite
import os

DATABASE_PATH = os.environ.get("DATABASE_PATH", "warehouse.db")


async def get_db() -> aiosqlite.Connection:
    db = await aiosqlite.connect(DATABASE_PATH)
    db.row_factory = aiosqlite.Row
    await db.execute("PRAGMA journal_mode=WAL")
    await db.execute("PRAGMA busy_timeout=5000")
    return db


async def init_db():
    """Create tables if they don't exist."""
    db = await get_db()
    try:
        await db.executescript(
            """
            CREATE TABLE IF NOT EXISTS orders (
                id TEXT PRIMARY KEY,
                items TEXT NOT NULL,          -- JSON array
                priority INTEGER DEFAULT 1,
                status TEXT DEFAULT 'pending', -- pending | in_progress | completed | cancelled
                assigned_forklift TEXT,
                created_at TEXT NOT NULL,
                completed_at TEXT
            );

            CREATE TABLE IF NOT EXISTS tasks (
                id TEXT PRIMARY KEY,
                order_id TEXT NOT NULL,
                forklift_id TEXT,
                status TEXT DEFAULT 'pending', -- pending | in_progress | completed | cancelled | failed
                shelf_x REAL,
                shelf_y REAL,
                shelf_z REAL,
                dock_x REAL,
                dock_y REAL,
                dock_z REAL,
                created_at TEXT NOT NULL,
                completed_at TEXT,
                FOREIGN KEY (order_id) REFERENCES orders(id)
            );

            CREATE TABLE IF NOT EXISTS metrics_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                metric_name TEXT NOT NULL,
                metric_value REAL NOT NULL,
                strategy TEXT
            );
            """
        )
        await db.commit()
    finally:
        await db.close()
