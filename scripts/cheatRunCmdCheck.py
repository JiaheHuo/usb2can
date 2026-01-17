import csv
import os
import matplotlib.pyplot as plt

csv_path = "/home/jhuo/robstride_usb2can_ctrl/logs/track.csv"
if not os.path.exists(csv_path):
    raise FileNotFoundError(csv_path)

# i -> series
data = {}  # i -> {"t":[], "q_m":[], "q_j":[], "q_m_des":[], "q_j_des":[], "motor_name":str, "joint_name":str}

with open(csv_path, "r", newline="", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    fields = reader.fieldnames or []

    need = ["t", "i", "q_m", "q_j", "q_m_des", "q_j_des", "motor_name", "joint_name"]
    for k in need:
        if k not in fields:
            raise RuntimeError(f"missing column '{k}', fields={fields}")

    for row in reader:
        try:
            i = int(row["i"])
            d = data.setdefault(
                i,
                {
                    "t": [],
                    "q_m": [],
                    "q_j": [],
                    "q_m_des": [],
                    "q_j_des": [],
                    "motor_name": "",
                    "joint_name": "",
                },
            )
            d["t"].append(float(row["t"]))
            d["q_m"].append(float(row["q_m"]))
            d["q_j"].append(float(row["q_j"]))
            d["q_m_des"].append(float(row["q_m_des"]))
            d["q_j_des"].append(float(row["q_j_des"]))

            if not d["motor_name"]:
                d["motor_name"] = (row.get("motor_name", "") or "").strip().strip('"')
            if not d["joint_name"]:
                d["joint_name"] = (row.get("joint_name", "") or "").strip().strip('"')
        except Exception:
            continue

ids = sorted(data.keys())
print("found idx:", ids)

# 画 12 个（通常 0~11）
ids = ids[:12]

def plot_grid(ids, y_m_key, y_j_key, title):
    fig, axes = plt.subplots(4, 3, sharex=True, figsize=(16, 10))
    fig.suptitle(title, fontsize=14)

    axes = axes.flatten()
    for k, idx in enumerate(ids):
        ax = axes[k]
        d = data[idx]
        mn = d["motor_name"] or f"motor_{idx}"
        jn = d["joint_name"] or f"joint_{idx}"

        # 不在 title 里写 idx，而是在图内标注
        ax.text(
            0.02, 0.95, f"idx={idx}",
            transform=ax.transAxes,
            va="top", ha="left",
            fontsize=10,
            bbox=dict(facecolor="white", alpha=0.65, edgecolor="none", pad=2.5),
        )

        # 曲线（不显式指定颜色，用 matplotlib 默认配色）
        ax.plot(d["t"], d[y_m_key], "-",  linewidth=1.6, label=y_m_key)   # motor: solid
        ax.plot(d["t"], d[y_j_key], "--", linewidth=1.6, label=y_j_key)   # joint: dashed

        # ax.set_title(f"{mn} / {jn}", fontsize=5)
        ax.grid(True)

        if k == 0:
            ax.legend(fontsize=9, loc="best")

    # 多余子图关掉
    for k in range(len(ids), 12):
        axes[k].axis("off")

    # 底部 x label
    for ax in axes[-3:]:
        if ax.has_data():
            ax.set_xlabel("t (s)")

    plt.tight_layout()
    return fig

# ---- Figure 1: actual position ----
plot_grid(
    ids,
    y_m_key="q_m",
    y_j_key="q_j",
    title="Actual position: motor q_m (solid) vs joint q_j (dashed)"
)

# ---- Figure 2: desired position ----
plot_grid(
    ids,
    y_m_key="q_m_des",
    y_j_key="q_j_des",
    title="Desired position: motor q_m_des (solid) vs joint q_j_des (dashed)"
)

plt.show()
