"""Mô-đun tính toán các metric đánh giá chất lượng căn chỉnh (calibration) LiDAR-Camera.

Các metric được hỗ trợ:
1. Box Containment Ratio (BCR): Tỉ lệ điểm LiDAR thuộc vật thể 3D nằm trong 2D bounding box tương ứng.
2. Edge Alignment Score (EAS): Điểm tương quan giữa đường viền độ sâu (depth edge) của LiDAR
   và đường biên ảnh (Canny edge) thông qua hàm Gaussian trên Distance Transform.
3. FOV Retention Ratio: Tỉ lệ điểm LiDAR rơi vào khung ảnh hợp lệ.
"""
from __future__ import annotations

import cv2
import numpy as np
from starter.kitti_io import KittiCalib, KittiObject
from starter.projection import velo_to_cam, project_velo_to_image


def extract_points_in_3d_box(points_velo: np.ndarray, obj: KittiObject, calib: KittiCalib) -> np.ndarray:
    """Trích xuất các điểm LiDAR (velodyne frame) nằm trong 3D bounding box của obj (camera frame)."""
    if len(points_velo) == 0:
        return np.empty((0, points_velo.shape[1]), dtype=points_velo.dtype)

    pts_cam = velo_to_cam(points_velo[:, :3], calib)
    h, w, l = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)

    dx = pts_cam[:, 0] - obj.location[0]
    dy = pts_cam[:, 1] - obj.location[1]
    dz = pts_cam[:, 2] - obj.location[2]

    # Biến đổi toạ độ ngược về hệ trục cục bộ của box
    x_loc = c * dx - s * dz
    y_loc = dy
    z_loc = s * dx + c * dz

    in_box = (np.abs(x_loc) <= l / 2.0) & (y_loc >= -h) & (y_loc <= 0.0) & (np.abs(z_loc) <= w / 2.0)
    return points_velo[in_box]


def compute_box_containment(uv: np.ndarray, bbox: np.ndarray) -> float:
    """Tính tỉ lệ điểm chiếu (u, v) rơi vào trong 2D bounding box [x1, y1, x2, y2]."""
    if len(uv) == 0:
        return 0.0
    x1, y1, x2, y2 = bbox
    in_box = (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
    return float(in_box.mean())


def compute_canny_distance_map(image: np.ndarray, low_th: int = 60, high_th: int = 160) -> tuple[np.ndarray, np.ndarray]:
    """Tạo Canny edge map và Euclidean Distance Transform map từ ảnh RGB."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, low_th, high_th)
    dist_map = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    return edges, dist_map


def extract_depth_edge_points(points_velo: np.ndarray, calib: KittiCalib,
                              image_shape: tuple[int, ...],
                              jump_threshold: float = 2.0,
                              max_depth: float = 50.0) -> np.ndarray:
    """Trích xuất các điểm biên độ sâu (depth discontinuity / silhouette) từ point cloud.
    
    Phương pháp:
    1. Chiếu điểm velodyne sang camera plane với calib gốc.
    2. Điền z_cam vào buffer độ sâu 2D (chọn điểm gần nhất cho mỗi pixel).
    3. Dùng phép lọc hình thái học (morphological dilation - erosion) để tìm bước nhảy độ sâu.
    4. Giữ lại các điểm có bước nhảy độ sâu vượt qua ngưỡng jump_threshold.
    """
    uv, depth, mask = project_velo_to_image(points_velo, calib, image_shape)
    if len(uv) == 0:
        return np.empty((0, points_velo.shape[1]), dtype=points_velo.dtype)

    H, W = image_shape[:2]
    depth_buffer = np.zeros((H, W), dtype=np.float32)
    u_idx = np.clip(np.round(uv[:, 0]).astype(int), 0, W - 1)
    v_idx = np.clip(np.round(uv[:, 1]).astype(int), 0, H - 1)

    # Đưa độ sâu gần nhất vào buffer
    for u, v, d in zip(u_idx, v_idx, depth):
        if depth_buffer[v, u] == 0 or d < depth_buffer[v, u]:
            depth_buffer[v, u] = d

    kernel_size = 7 if len(uv) > 10000 else 21
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    d_max = cv2.dilate(depth_buffer, kernel)
    d_min_mask = np.where(depth_buffer > 0, depth_buffer, 999.0).astype(np.float32)
    d_min = cv2.erode(d_min_mask, kernel)
    d_jump = d_max - d_min

    is_edge = (d_jump[v_idx, u_idx] >= jump_threshold) & (depth <= max_depth)
    valid_velo_points = points_velo[mask]
    return valid_velo_points[is_edge]


def compute_edge_alignment_score(uv: np.ndarray, dist_map: np.ndarray,
                                 image_shape: tuple[int, ...],
                                 sigma: float = 3.0) -> tuple[float, float]:
    """Tính Edge Alignment Score (EAS) và khoảng cách trung bình tới biên ảnh gần nhất.
    
    EAS = mean(exp(- d^2 / (2 * sigma^2)))
    """
    if len(uv) == 0:
        return 0.0, float("nan")

    H, W = image_shape[:2]
    u_idx = np.clip(np.round(uv[:, 0]).astype(int), 0, W - 1)
    v_idx = np.clip(np.round(uv[:, 1]).astype(int), 0, H - 1)

    d = dist_map[v_idx, u_idx]
    score = float(np.mean(np.exp(-(d**2) / (2 * (sigma**2)))))
    mean_dist = float(np.mean(d))
    return score, mean_dist
