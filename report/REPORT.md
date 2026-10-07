# Báo cáo Day 6: Kiểm tra Calibration LiDAR-Camera bằng Projection và Alignment Score

- **Họ tên:** Nguyễn Thế Khang
- **MSSV:** 2A202602964
- **Lớp:** K4-Track4
- **Link repo:** https://github.com/khangnguyenthe18/K4-Track4-Day06-NguyenTheKhang-2A202602964-3D-From-Point-Clouds
- **Topic:** A — Kiểm tra calibration LiDAR-camera bằng projection (LiDAR-camera projection QA)
- **Dataset:** data/synthetic, data/kitti_mini, data/nuscenes_mini_subset
- **Các frame đã dùng:** KITTI: 000011, 000021, 000049, 000004; nuScenes: scene-0103_010, scene-1094_010; Synthetic: 000000, 000003

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

Một câu khẳng định kỹ thuật có thể kiểm chứng:

Độ lệch góc yaw cảm biến LiDAR-camera từ 1.0° trở lên khiến hơn 20% điểm LiDAR thuộc vật thể rơi ra ngoài 2D bounding box ở cự ly > 25 m và làm giảm chỉ số Edge-Alignment Score (EAS) trên 30%, cho phép thiết kế cơ chế tự động phát hiện calibration drift trực tuyến mà không cần nhãn 3D ground-truth.


## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

| Cấu hình / mức perturb | Metric 1 | Metric 2 | Ghi chú |
|---|---|---|---|
| [ĐIỀN] | | | |

![demo](../results/figures/[ĐIỀN].png)

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch.

```bash
[ĐIỀN]
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
