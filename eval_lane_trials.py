#!/usr/bin/env python3
import os
import struct
import sqlite3
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUTPUT_DIR = "lane_trial_eval"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def read_error_topic(bag_dir):
    db = next((f for f in os.listdir(bag_dir) if f.endswith(".db3")), None)
    if not db:
        return None, None
    conn = sqlite3.connect(f"{bag_dir}/{db}")
    topic_row = conn.execute(
        "SELECT id FROM topics WHERE name='/lane_center_error'"
    ).fetchone()
    if not topic_row:
        conn.close()
        return None, None
    topic_id = topic_row[0]
    rows = conn.execute(
        f"SELECT timestamp, data FROM messages WHERE topic_id={topic_id} ORDER BY timestamp"
    ).fetchall()
    conn.close()
    if not rows:
        return None, None
    timestamps = np.array([r[0] for r in rows], dtype=np.float64)
    timestamps = (timestamps - timestamps[0]) / 1e9  # seconds from start
    errors = np.array([struct.unpack_from("<f", r[1], 4)[0] for r in rows])

    # Clip to first 55 seconds
    mask = timestamps <= 55.0
    timestamps = timestamps[mask]
    errors = errors[mask]
    return timestamps, errors

bags = sorted([
    d for d in os.listdir(".")
    if os.path.isdir(d) and "lane" in d.lower() and "trial" in d.lower()
])

print(f"{'Bag':<40} {'N msgs':>7}  {'Mean |err|':>10}  {'Std':>8}  {'Max |err|':>10}")
print("-" * 80)

fig_all, ax_all = plt.subplots(figsize=(12, 6))

for bag in bags:
    timestamps, errors = read_error_topic(bag)
    if timestamps is None:
        print(f"{bag:<40}  no /lane_center_error topic")
        continue

    abs_err = np.abs(errors)
    mean_err = np.mean(abs_err)
    std_err  = np.std(abs_err)
    max_err  = np.max(abs_err)
    print(f"{bag:<40} {len(errors):>7}  {mean_err:>10.4f}  {std_err:>8.4f}  {max_err:>10.4f}")

    # Individual plot
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(timestamps, errors, linewidth=0.8, color="steelblue", label="error")
    ax.axhline(0, color="black", linewidth=0.6, linestyle="--")
    ax.fill_between(timestamps, errors, 0, alpha=0.2, color="steelblue")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Lane center error")
    ax.set_title(f"{bag}\nmean |err|={mean_err:.4f}  std={std_err:.4f}  max={max_err:.4f}")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"{OUTPUT_DIR}/{bag}.png", dpi=120)
    plt.close(fig)

    ax_all.plot(timestamps, errors, linewidth=0.7, label=bag, alpha=0.8)

ax_all.axhline(0, color="black", linewidth=0.8, linestyle="--")
ax_all.set_xlabel("Time (s)")
ax_all.set_ylabel("Lane center error")
ax_all.set_title("All lane trials — /lane_center_error")
ax_all.legend(fontsize=7)
fig_all.tight_layout()
fig_all.savefig(f"{OUTPUT_DIR}/all_trials_overlay.png", dpi=120)
plt.close(fig_all)

print(f"\nPlots saved to ./{OUTPUT_DIR}/")
