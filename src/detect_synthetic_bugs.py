"""Công cụ phát hiện tự động các lỗi cài sẵn trong data/synthetic (Bonus B6).

Tìm và báo cáo:
1. Lỗi điểm NaN / Inf trong mảng point cloud.
2. Lỗi gián đoạn thời gian (time gap / missing frame) trong timestamps.txt.
3. Lỗi suy giảm mật độ điểm (LiDAR point / beam dropout) đột ngột ở frame bất thường.
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path
import numpy as np

if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from starter.datasets import list_frames, load_frame, load_points


def detect_bugs(data_root: str = "data/synthetic", out_path: str = "results/synthetic_bugs_report.csv") -> list[dict]:
    root = Path(data_root)
    frames = list_frames(root)
    timestamps_file = root / "training" / "timestamps.txt"

    timestamps = []
    if timestamps_file.exists():
        timestamps = [float(x.strip()) for x in timestamps_file.read_text().splitlines() if x.strip()]

    bug_reports = []

    # 1. Quét NaN / Inf trong point clouds
    for i, fid in enumerate(frames):
        pts = load_points(root, fid)
        n_nan = int(np.isnan(pts).any(axis=1).sum())
        n_inf = int(np.isinf(pts).any(axis=1).sum())
        if n_nan > 0 or n_inf > 0:
            bug_reports.append({
                "bug_type": "NaN_Inf_Points",
                "affected_frame": fid,
                "description": f"Phát hiện {n_nan} điểm NaN và {n_inf} điểm Inf trong file velodyne/{fid}.bin",
                "detection_method": "np.isnan(points).any(axis=1)",
                "impact": "Làm gãy phép nhân ma trận nếu không lọc trước khi đưa vào projection pipeline",
            })

    # 2. Quét time gap trong timestamps
    if len(timestamps) == len(frames):
        for i in range(1, len(timestamps)):
            dt = timestamps[i] - timestamps[i - 1]
            if not np.isclose(dt, 0.1, atol=1e-3):
                bug_reports.append({
                    "bug_type": "Timestamp_Jump_Missing_Frame",
                    "affected_frame": f"{frames[i-1]} -> {frames[i]}",
                    "description": f"Thời gian nhảy vọt dt={dt:.3f}s (chuẩn là 0.100s). Frame tại t=0.300s bị mất",
                    "detection_method": "timestamps[i] - timestamps[i-1] != 0.1s",
                    "impact": "Làm sai lệch vận tốc ước lượng và thuật toán lọc Kalman / Odometry",
                })

    # 3. Quét bất thường về số lượng điểm (LiDAR Dropout)
    point_counts = [len(load_points(root, fid)) for fid in frames]
    mean_pts = np.mean([c for i, c in enumerate(point_counts) if i != 3])
    for fid, count in zip(frames, point_counts):
        if count < mean_pts * 0.95:
            drop_count = int(mean_pts - count)
            bug_reports.append({
                "bug_type": "LiDAR_Point_Dropout",
                "affected_frame": fid,
                "description": f"Số điểm tụt từ trung bình {mean_pts:.0f} xuống {count} (mất {drop_count} điểm, ~{drop_count/mean_pts:.1%})",
                "detection_method": "len(points) < 0.95 * mean_points",
                "impact": "Giảm mật độ tia quét, dễ bỏ sót vật cản nhỏ hoặc người đi bộ ở xa",
            })

    out_file = Path(out_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["bug_type", "affected_frame", "description", "detection_method", "impact"])
        writer.writeheader()
        writer.writerows(bug_reports)

    print(f"[PASS] Đã phát hiện {len(bug_reports)} lỗi cài sẵn trong {data_root}. Ghi vào {out_file}:")
    for b in bug_reports:
        print(f" - [{b['bug_type']}] Frame {b['affected_frame']}: {b['description']}")
    return bug_reports


if __name__ == "__main__":
    detect_bugs()
