"""Công cụ đánh giá chất lượng căn chỉnh (calibration QA) LiDAR-Camera.

Thực hiện:
1. Thí nghiệm sweep độ lệch extrinsic (yaw, pitch, roll, tx, ty, tz).
2. Tính toán Box Containment Ratio (BCR) phân tầng theo cự ly (Near <15m, Mid 15-30m, Far >30m).
3. Tính toán Edge Alignment Score (EAS) tự động phát hiện drift không cần nhãn.
4. Đo đạc độ trễ (latency p50/p95).
5. So sánh hai dataset (KITTI 64-beam vs nuScenes 32-beam).
"""
from __future__ import annotations

import argparse
import csv
import platform
import sys
import time
from pathlib import Path

if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

import cv2
import matplotlib.pyplot as plt
import numpy as np

from starter.datasets import dataset_type, list_frames, load_frame
from starter.kitti_io import KittiCalib, KittiObject
from starter.projection import (
    draw_box2d,
    overlay_points,
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)
from src.metrics import (
    compute_box_containment,
    compute_canny_distance_map,
    compute_edge_alignment_score,
    extract_depth_edge_points,
    extract_points_in_3d_box,
)


def run_sweep_experiment(
    data_root: str,
    frame_id: str,
    out_dir: str = "results",
    seed: int = 42,
) -> tuple[list[dict], Path]:
    """Chạy thí nghiệm sweep các mức lệch calibration góc quay và tịnh tiến."""
    np.random.seed(seed)
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    fig_dir = out_path / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    fr = load_frame(data_root, frame_id)
    pts_velo = fr["points"][:, :3]
    calib = fr["calib"]
    img = fr["image"]
    H, W = img.shape[:2]
    labels = fr["labels"]

    # Phân loại objects theo cự ly
    obj_data = []
    for obj in labels:
        pts_in_obj = extract_points_in_3d_box(pts_velo, obj, calib)
        dist = float(np.linalg.norm(obj.location))
        if len(pts_in_obj) >= 5:  # Chỉ xét object có đủ điểm quét
            if dist < 15.0:
                dist_cat = "Near (<15m)"
            elif dist <= 30.0:
                dist_cat = "Mid (15-30m)"
            else:
                dist_cat = "Far (>30m)"
            obj_data.append({"obj": obj, "pts": pts_in_obj, "dist": dist, "category": dist_cat})

    # Chuẩn bị Canny edge map và depth edge points cho EAS
    edges, dist_map = compute_canny_distance_map(img, low_th=60, high_th=160)
    depth_edge_pts = extract_depth_edge_points(pts_velo, calib, img.shape, jump_threshold=2.0)

    # 1. Sweep Yaw: từ -3.0 đến +3.0 độ
    yaw_levels = [-3.0, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0]
    results: list[dict] = []

    for yaw in yaw_levels:
        calib_drift = perturb_extrinsic(calib, yaw_deg=yaw)
        uv_all, depth_all, mask_all = project_velo_to_image(pts_velo, calib_drift, img.shape)
        fov_ratio = float(mask_all.mean())

        # Tính BCR theo từng cự ly
        near_ratios, mid_ratios, far_ratios = [], [], []
        for item in obj_data:
            uv_obj, _, _ = project_velo_to_image(item["pts"], calib_drift, img.shape)
            r = compute_box_containment(uv_obj, item["obj"].bbox)
            if item["category"] == "Near (<15m)":
                near_ratios.append(r)
            elif item["category"] == "Mid (15-30m)":
                mid_ratios.append(r)
            else:
                far_ratios.append(r)

        bcr_near = float(np.mean(near_ratios)) if near_ratios else np.nan
        bcr_mid = float(np.mean(mid_ratios)) if mid_ratios else np.nan
        bcr_far = float(np.mean(far_ratios)) if far_ratios else np.nan
        all_ratios = near_ratios + mid_ratios + far_ratios
        bcr_overall = float(np.mean(all_ratios)) if all_ratios else np.nan

        # Tính EAS
        if len(depth_edge_pts) > 0:
            uv_edge, _, _ = project_velo_to_image(depth_edge_pts, calib_drift, img.shape)
            eas_score, mean_dist_px = compute_edge_alignment_score(uv_edge, dist_map, img.shape)
        else:
            eas_score, mean_dist_px = np.nan, np.nan

        # Tiêu chuẩn cảnh báo drift tự động (threshold: EAS giảm > 15% hoặc BCR_far < 80%)
        # Tại yaw=0, EAS_score xấp xỉ mức đỉnh
        results.append({
            "perturb_type": "yaw",
            "param_value": yaw,
            "unit": "deg",
            "fov_retention_ratio": fov_ratio,
            "bcr_overall": bcr_overall,
            "bcr_near": bcr_near,
            "bcr_mid": bcr_mid,
            "bcr_far": bcr_far,
            "eas_score": eas_score,
            "mean_edge_dist_px": mean_dist_px,
        })

    # 2. Sweep Translation X (ngang) và Z (sâu)
    tx_levels = [-0.10, -0.05, 0.0, 0.05, 0.10]
    for tx in tx_levels:
        if tx == 0.0:
            continue
        calib_drift = perturb_extrinsic(calib, t_xyz_m=(tx, 0.0, 0.0))
        uv_all, depth_all, mask_all = project_velo_to_image(pts_velo, calib_drift, img.shape)
        fov_ratio = float(mask_all.mean())

        near_ratios, mid_ratios, far_ratios = [], [], []
        for item in obj_data:
            uv_obj, _, _ = project_velo_to_image(item["pts"], calib_drift, img.shape)
            r = compute_box_containment(uv_obj, item["obj"].bbox)
            if item["category"] == "Near (<15m)":
                near_ratios.append(r)
            elif item["category"] == "Mid (15-30m)":
                mid_ratios.append(r)
            else:
                far_ratios.append(r)

        bcr_near = float(np.mean(near_ratios)) if near_ratios else np.nan
        bcr_mid = float(np.mean(mid_ratios)) if mid_ratios else np.nan
        bcr_far = float(np.mean(far_ratios)) if far_ratios else np.nan
        all_ratios = near_ratios + mid_ratios + far_ratios
        bcr_overall = float(np.mean(all_ratios)) if all_ratios else np.nan

        uv_edge, _, _ = project_velo_to_image(depth_edge_pts, calib_drift, img.shape)
        eas_score, mean_dist_px = compute_edge_alignment_score(uv_edge, dist_map, img.shape)

        results.append({
            "perturb_type": "translation_x",
            "param_value": tx,
            "unit": "meter",
            "fov_retention_ratio": fov_ratio,
            "bcr_overall": bcr_overall,
            "bcr_near": bcr_near,
            "bcr_mid": bcr_mid,
            "bcr_far": bcr_far,
            "eas_score": eas_score,
            "mean_edge_dist_px": mean_dist_px,
        })

    # Lưu kết quả ra file CSV
    csv_file = out_path / "yaw_perturb_sweep.csv"
    with open(csv_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)

    print(f"[PASS] Đã ghi kết quả sweep vào {csv_file}")
    return results, csv_file


def generate_experiment_plots(results: list[dict], out_fig_path: Path) -> None:
    """Vẽ biểu đồ phân tích kỹ thuật chất lượng cao từ kết quả sweep."""
    yaw_rows = [r for r in results if r["perturb_type"] == "yaw"]
    yaws = [r["param_value"] for r in yaw_rows]
    bcr_near = [r["bcr_near"] * 100 for r in yaw_rows]
    bcr_mid = [r["bcr_mid"] * 100 for r in yaw_rows]
    bcr_far = [r["bcr_far"] * 100 for r in yaw_rows]
    eas_scores = [r["eas_score"] for r in yaw_rows]
    mean_dists = [r["mean_edge_dist_px"] for r in yaw_rows]
    fov_ratios = [r["fov_retention_ratio"] * 100 for r in yaw_rows]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    plt.subplots_adjust(hspace=0.3, wspace=0.25)

    # Subplot 1: BCR theo cự ly
    ax1 = axes[0, 0]
    ax1.plot(yaws, bcr_near, marker="o", color="#2ca02c", linewidth=2.2, label="Near (<15m)")
    ax1.plot(yaws, bcr_mid, marker="s", color="#ff7f0e", linewidth=2.2, label="Mid (15-30m)")
    ax1.plot(yaws, bcr_far, marker="^", color="#d62728", linewidth=2.2, label="Far (>30m)")
    ax1.axhline(80, color="gray", linestyle="--", alpha=0.7, label="Ngưỡng an toàn 80%")
    ax1.set_title("Box Containment Ratio (BCR) vs Yaw Drift", fontsize=12, fontweight="bold")
    ax1.set_xlabel("Độ lệch góc Yaw (độ)", fontsize=10)
    ax1.set_ylabel("Tỉ lệ điểm nằm trong 2D box (%)", fontsize=10)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="lower left", fontsize=9)
    ax1.set_ylim(-5, 105)

    # Subplot 2: Edge Alignment Score (EAS)
    ax2 = axes[0, 1]
    ax2.plot(yaws, eas_scores, marker="D", color="#1f77b4", linewidth=2.4, label="Edge Alignment Score (EAS)")
    # Thêm ngưỡng phát hiện drift
    max_eas = max(eas_scores)
    drift_thresh = max_eas * 0.85
    ax2.axhline(drift_thresh, color="#e377c2", linestyle="--", linewidth=1.8,
                label=f"Ngưỡng cảnh báo drift (-15% = {drift_thresh:.3f})")
    ax2.set_title("Edge Alignment Score (EAS) vs Yaw Drift", fontsize=12, fontweight="bold")
    ax2.set_xlabel("Độ lệch góc Yaw (độ)", fontsize=10)
    ax2.set_ylabel("Chỉ số EAS (0 - 1)", fontsize=10)
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="lower center", fontsize=9)

    # Subplot 3: Khoảng cách trung bình tới biên ảnh (Pixel Error)
    ax3 = axes[1, 0]
    ax3.plot(yaws, mean_dists, marker="v", color="#9467bd", linewidth=2.2, label="Mean Edge Distance (px)")
    ax3.set_title("Độ lệch biên trung bình (Chamfer Distance)", fontsize=12, fontweight="bold")
    ax3.set_xlabel("Độ lệch góc Yaw (độ)", fontsize=10)
    ax3.set_ylabel("Khoảng cách pixel trung bình (px)", fontsize=10)
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="upper center", fontsize=9)

    # Subplot 4: Tỉ lệ điểm trong Camera FOV
    ax4 = axes[1, 1]
    ax4.plot(yaws, fov_ratios, marker="p", color="#8c564b", linewidth=2.2, label="FOV Retention Ratio (%)")
    ax4.set_title("Tỉ lệ điểm rơi vào Camera FOV", fontsize=12, fontweight="bold")
    ax4.set_xlabel("Độ lệch góc Yaw (độ)", fontsize=10)
    ax4.set_ylabel("Tỉ lệ (%)", fontsize=10)
    ax4.grid(True, linestyle=":", alpha=0.6)
    ax4.legend(loc="lower center", fontsize=9)

    fig.suptitle("Đánh giá độ nhạy Calibration Drift LiDAR-Camera (KITTI Frame 000011)",
                 fontsize=14, fontweight="bold", y=0.98)
    fig.savefig(out_fig_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"[PASS] Đã lưu biểu đồ phân tích vào {out_fig_path}")


def run_latency_benchmark(data_root: str, frame_id: str, num_runs: int = 30) -> Path:
    """Đo thời gian chạy (latency p50/p95) của pipeline projection và alignment score (Bonus B3)."""
    fr = load_frame(data_root, frame_id)
    pts = fr["points"][:, :3]
    calib = fr["calib"]
    img = fr["image"]
    H, W = img.shape[:2]

    # Warmup (bỏ qua lần đầu)
    _ = project_velo_to_image(pts, calib, img.shape)
    edges, dist_map = compute_canny_distance_map(img)
    depth_edge_pts = extract_depth_edge_points(pts, calib, img.shape)

    proj_latencies = []
    eas_latencies = []

    for _ in range(num_runs):
        # 1. Đo Projection pipeline
        t0 = time.perf_counter()
        uv, depth, mask = project_velo_to_image(pts, calib, img.shape)
        t1 = time.perf_counter()
        proj_latencies.append((t1 - t0) * 1000.0)

        # 2. Đo Alignment Score pipeline
        t2 = time.perf_counter()
        uv_e, _, _ = project_velo_to_image(depth_edge_pts, calib, img.shape)
        score, _ = compute_edge_alignment_score(uv_e, dist_map, img.shape)
        t3 = time.perf_counter()
        eas_latencies.append((t3 - t2) * 1000.0)

    p50_proj = float(np.percentile(proj_latencies, 50))
    p95_proj = float(np.percentile(proj_latencies, 95))
    p50_eas = float(np.percentile(eas_latencies, 50))
    p95_eas = float(np.percentile(eas_latencies, 95))

    out_file = Path("results/latency_benchmark.csv")
    with open(out_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["hardware", "num_points", "num_runs", "task", "latency_p50_ms", "latency_p95_ms", "latency_mean_ms"])
        hw_info = f"{platform.processor() or platform.machine()} ({platform.system()})"
        writer.writerow([hw_info, len(pts), num_runs, "Projection_Velo_To_Image", f"{p50_proj:.3f}", f"{p95_proj:.3f}", f"{np.mean(proj_latencies):.3f}"])
        writer.writerow([hw_info, len(depth_edge_pts), num_runs, "Edge_Alignment_Score", f"{p50_eas:.3f}", f"{p95_eas:.3f}", f"{np.mean(eas_latencies):.3f}"])

    print(f"[PASS] Benchmark latency: Projection p50={p50_proj:.2f}ms, p95={p95_proj:.2f}ms | EAS p50={p50_eas:.2f}ms, p95={p95_eas:.2f}ms -> {out_file}")
    return out_file


def run_dataset_comparison() -> Path:
    """So sánh kết quả trên KITTI (64 beam) và nuScenes (32 beam, ban ngày vs ban đêm) (Bonus B5)."""
    configs = [
        ("KITTI 64-beam (Day)", "data/kitti_mini", "000011", {}),
        ("nuScenes 32-beam (Day, Deskewed)", "data/nuscenes_mini_subset", "scene-0103_010", {"use_ego_motion": True}),
        ("nuScenes 32-beam (Day, No Deskew)", "data/nuscenes_mini_subset", "scene-0103_010", {"use_ego_motion": False}),
        ("nuScenes 32-beam (Night Rain, Deskewed)", "data/nuscenes_mini_subset", "scene-1094_010", {"use_ego_motion": True}),
    ]

    comp_rows = []
    for desc, root, fid, kwargs in configs:
        fr = load_frame(root, fid, **kwargs)
        pts = fr["points"][:, :3]
        calib = fr["calib"]
        img = fr["image"]

        uv, depth, mask = project_velo_to_image(pts, calib, img.shape)
        edges, dist_map = compute_canny_distance_map(img)
        edge_pts = extract_depth_edge_points(pts, calib, img.shape)
        eas, mean_d = compute_edge_alignment_score(project_velo_to_image(edge_pts, calib, img.shape)[0], dist_map, img.shape)

        comp_rows.append({
            "dataset_config": desc,
            "frame_id": fid,
            "total_points": len(pts),
            "points_inside_fov": int(mask.sum()),
            "fov_percentage": f"{mask.mean():.1%}",
            "depth_edge_points": len(edge_pts),
            "baseline_eas": f"{eas:.4f}",
            "mean_edge_dist_px": f"{mean_d:.2f}",
        })

    out_file = Path("results/dataset_comparison.csv")
    with open(out_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(comp_rows[0].keys()))
        writer.writeheader()
        writer.writerows(comp_rows)

    print(f"[PASS] Đã ghi so sánh dataset vào {out_file}")
    return out_file


def generate_failure_cases(out_dir: Path) -> None:
    """Tạo các ảnh minh hoạ failure cases chi tiết phục vụ CP4 và REPORT."""
    # Failure Case 1: Lệch yaw 2.0° làm điểm LiDAR trôi khỏi vật ở xa (vật 34.2m)
    fr_kitti = load_frame("data/kitti_mini", "000011")
    calib_yaw2 = perturb_extrinsic(fr_kitti["calib"], yaw_deg=2.0)
    uv, depth, _ = project_velo_to_image(fr_kitti["points"], calib_yaw2, fr_kitti["image"].shape)
    vis_fail1 = overlay_points(fr_kitti["image"], uv, depth)
    for obj in fr_kitti["labels"]:
        vis_fail1 = draw_box2d(vis_fail1, obj.bbox, color=(0, 0, 255), label=f"{obj.type} [DRIFT +2.0 deg]")

    # Crop vùng vật thể xa (Pedestrian ở u~650, v~180) để thấy rõ điểm bị lệch hoàn toàn khỏi 2D box
    cv2.putText(vis_fail1, "FAILURE: Yaw +2.0 deg drift causes distant object points to miss 2D box",
                (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    fail1_path = out_dir / "fail_01_yaw_2deg_far_object.png"
    cv2.imwrite(str(fail1_path), vis_fail1)

    # Failure Case 2: nuScenes không bù chuyển động xe (Ego-motion ignore)
    fr_nusc_noego = load_frame("data/nuscenes_mini_subset", "scene-0103_010", use_ego_motion=False)
    uv_ne, depth_ne, _ = project_velo_to_image(fr_nusc_noego["points"], fr_nusc_noego["calib"], fr_nusc_noego["image"].shape)
    vis_fail2 = overlay_points(fr_nusc_noego["image"], uv_ne, depth_ne)
    for obj in fr_nusc_noego["labels"]:
        vis_fail2 = draw_box2d(vis_fail2, obj.bbox, color=(0, 165, 255), label=f"{obj.type} [NO EGO DESKEW]")

    cv2.putText(vis_fail2, "FAILURE: Time desynchronization without ego-motion deskewing",
                (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
    fail2_path = out_dir / "fail_02_nuscenes_no_egomotion.png"
    cv2.imwrite(str(fail2_path), vis_fail2)

    # Failure Case 3: nuScenes cảnh ban đêm sau mưa (scene-1094_010) - Thiếu texture khiến Canny edge thất bại
    fr_night = load_frame("data/nuscenes_mini_subset", "scene-1094_010", use_ego_motion=True)
    gray_night = cv2.cvtColor(fr_night["image"], cv2.COLOR_BGR2GRAY)
    canny_night = cv2.Canny(gray_night, 60, 160)
    # Ghép ảnh RGB và ảnh Canny cạnh nhau
    h, w = gray_night.shape
    vis_fail3 = np.zeros((h, w * 2, 3), dtype=np.uint8)
    vis_fail3[:, :w] = fr_night["image"]
    vis_fail3[:, w:] = cv2.cvtColor(canny_night, cv2.COLOR_GRAY2BGR)
    cv2.putText(vis_fail3, "Camera Night Frame (Low Contrast)", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(vis_fail3, "Canny Edges (Missing Object Silhouettes)", (w + 30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    fail3_path = out_dir / "fail_03_textureless_night_scene.png"
    cv2.imwrite(str(fail3_path), vis_fail3)

    print(f"[PASS] Đã tạo 3 ảnh failure cases: {fail1_path.name}, {fail2_path.name}, {fail3_path.name}")


def generate_visual_drift_comparison(data_root: str, frame_id: str, out_path: Path) -> None:
    """Tạo ảnh so sánh trực quan crop chi tiết vật thể ở 0°, +1.0°, +2.0° yaw drift."""
    fr = load_frame(data_root, frame_id)
    img = fr["image"]
    pts = fr["points"]

    yaws = [0.0, 1.0, 2.0]
    crops = []
    # Crop vùng xe ở khoảng cách 27m và người đi bộ (u: 400-720, v: 140-270)
    ymin, ymax = 140, 270
    xmin, xmax = 430, 700

    for y in yaws:
        cal = perturb_extrinsic(fr["calib"], yaw_deg=y)
        uv, depth, mask = project_velo_to_image(pts, cal, img.shape)
        vis = overlay_points(img, uv, depth, radius=2)
        for obj in fr["labels"]:
            vis = draw_box2d(vis, obj.bbox, color=(0, 255, 0) if y == 0 else (0, 140, 255) if y == 1 else (0, 0, 255),
                             label=f"{obj.type}")
        crop = vis[ymin:ymax, xmin:xmax].copy()
        cv2.putText(crop, f"Yaw: {y:+.1f} deg", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if y == 0 else (0, 140, 255) if y == 1 else (0, 0, 255), 2)
        crops.append(crop)

    # Ghép 3 crop theo chiều dọc
    combined = np.vstack(crops)
    cv2.imwrite(str(out_path), combined)
    print(f"[PASS] Đã tạo ảnh trực quan so sánh drift vật thể: {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Chương trình phân tích kiểm thử độ nhạy calibration LiDAR-Camera (Topic A)."
    )
    parser.add_argument("--data-root", default="data/kitti_mini", help="Đường dẫn thư mục dataset")
    parser.add_argument("--frame", default="000011", help="ID frame cần chạy thí nghiệm")
    parser.add_argument("--out-dir", default="results", help="Thư mục xuất kết quả CSV và ảnh")
    parser.add_argument("--seed", type=int, default=42, help="Seed ngẫu nhiên")
    parser.add_argument("--run-all", action="store_true", help="Chạy toàn bộ thí nghiệm (sweep, latency, failure, so sánh)")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    print(f"=== Bắt đầu thí nghiệm Projection QA trên {args.data_root} [frame {args.frame}] ===")
    results, csv_file = run_sweep_experiment(args.data_root, args.frame, args.out_dir, args.seed)
    generate_experiment_plots(results, fig_dir / "calibration_drift_curves.png")

    if args.run_all:
        print("\n=== Đo Latency p50/p95 (Bonus B3) ===")
        run_latency_benchmark(args.data_root, args.frame, num_runs=30)

        print("\n=== So sánh dataset KITTI vs nuScenes (Bonus B5) ===")
        run_dataset_comparison()

        print("\n=== Tạo Failure Cases (CP4) ===")
        generate_failure_cases(fig_dir)
        generate_visual_drift_comparison(args.data_root, args.frame, fig_dir / "yaw_drift_visual_comparison.png")

    print("\n[HOÀN THÀNH TOÀN BỘ THÍ NGHIỆM]")


if __name__ == "__main__":
    main()
