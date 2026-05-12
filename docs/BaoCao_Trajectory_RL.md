# THIẾT KẾ QUY HOẠCH QUỸ ĐẠO VÀ TÍCH HỢP HỌC TĂNG CƯỜNG

Hệ thống sử dụng **hai phương pháp** quy hoạch quỹ đạo, tùy theo chế độ vận hành:

| | Chế độ Auto (FSM) | Chế độ AI (SAC) |
|-|-------------------|-----------------|
| **Quỹ đạo** | Nội suy hình thang vận tốc | Gia số delta liên tục từ mạng Neural |
| **Ưu điểm** | Ổn định, dự đoán được | Linh hoạt, thích ứng vị trí ngẫu nhiên |
| **Nhược điểm** | Cứng nhắc, phải lập trình từng bước | Cần huấn luyện 10M bước |

---

## 1. Chế độ Auto — Máy trạng thái FSM + Trajectory Executor

### 1.1 Chuỗi 11 trạng thái tuần tự

Robot thực hiện Pick & Place theo chuỗi:

```
IDLE → DETECT → APPROACH → DESCEND → PICK → LIFT → MOVE_TO_BIN → PLACE → RELEASE → RETREAT → DONE
```

| Trạng thái | Hành động | Vận tốc |
|-----------|----------|---------|
| DETECT | Camera ảo quét tọa độ vật | — |
| APPROACH | Bay đến phía trên vật | 0.12 m/s |
| DESCEND | Hạ mũi kẹp xuống sát vật | 0.05 m/s |
| PICK | Kích hoạt Vacuum, chờ 0.3s ổn định | — |
| LIFT | Nâng vật lên độ cao an toàn | 0.10 m/s |
| MOVE_TO_BIN | Bay ngang đến phía trên thùng | 0.15 m/s |
| PLACE | Hạ vật xuống miệng thùng | 0.05 m/s |
| RELEASE | Nhả Vacuum, vật rơi vào thùng | — |
| RETREAT | Quay về Home | 0.15 m/s |

**An toàn:** Timeout 15 giây/state + WorkspaceValidator kiểm tra vùng cấm trước khi di chuyển.

### 1.2 Bộ thực thi quỹ đạo (Trajectory Executor)

Hoạt động như một "kim đĩa than" — mỗi bước vật lý (1/240s), đọc ra 1 điểm trên quỹ đạo tại thời điểm `t` và đẩy 6 góc khớp xuống PyBullet:

```python
def update(self):
    self._t += (1.0 / 240.0) * self._speed_scale
    point = self._traj.get_point(self._t)
    self._env.set_joint_positions(point['q'], max_velocity=3.0)
```

**Biên dạng vận tốc hình thang (Trapezoidal):**
- Giai đoạn 1: Tăng tốc tuyến tính từ v=0 đến v_max.
- Giai đoạn 2: Giữ vận tốc đều (Cruise).
- Giai đoạn 3: Giảm tốc tuyến tính về v=0 trước khi chạm đích.

### 1.3 Chuyển đổi tọa độ

FSM làm việc với tọa độ Cartesian (XYZ), nhưng servo chỉ hiểu góc khớp. Tại mỗi waypoint:

```
Tọa độ XYZ đích → IK giải ra 6 góc khớp → PyBullet POSITION_CONTROL
```

---

## 2. Chế độ AI — Thuật toán SAC (Soft Actor-Critic)

### 2.1 Thuật toán và siêu tham số

Sử dụng SAC — thuật toán Off-Policy, Maximum Entropy (Haarnoja et al., 2019), mạnh nhất hiện tại cho điều khiển robot liên tục.

| Tham số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| `learning_rate` | 3×10⁻⁴ | Tốc độ học Adam |
| `gamma` | 0.99 | Hệ số chiết khấu (task dài ~300 bước) |
| `batch_size` | 256 | Kích thước mini-batch |
| `tau` | 0.005 | Polyak averaging (cập nhật Target Network) |
| `buffer_size` | 1,000,000 | Replay Buffer 1M transition |
| `ent_coef` | auto_0.1 | Entropy tự động |
| `net_arch` | [256, 256] | MLP 2 lớp ẩn |
| Tổng bước huấn luyện | 10,000,000 | 16 env song song, ~14-16 giờ |

### 2.2 Observation Space — 20 chiều

| Chỉ số | Nội dung | Kích thước |
|--------|---------|-----------|
| [0:3] | Vị trí EE (x, y, z) | 3D |
| [3:6] | Vị trí vật thể (x, y, z) | 3D |
| [6:9] | Vector tương đối EE → Object | 3D |
| [9:12] | Vector tương đối Object → Bin | 3D |
| [12:16] | Quaternion vật thể | 4D |
| [16] | Trạng thái Gripper (0=rảnh, 1=giữ) | 1D |
| [17:20] | Góc Euler EE (roll, pitch, yaw) | 3D |

Góc Euler EE (chỉ số 17-20) giúp AI biết mũi kẹp đang thẳng hay nghiêng — bắt buộc để gắp chính xác. Toàn bộ vector được chuẩn hóa bởi `VecNormalize` (mean=0, std=1).

### 2.3 Action Space — 7 chiều

| Chỉ số | Nội dung | Hệ số nhân |
|--------|---------|-----------|
| [0:3] | Gia số dịch chuyển Δx, Δy, Δz | × CART_DELTA_MAX |
| [3:6] | Gia số xoay ΔRoll, ΔPitch, ΔYaw | × 0.08 rad (~4.5°/step) |
| [6] | Không sử dụng (Hybrid Gripper) | — |

AI xuất ra gia số nhỏ (delta) mỗi bước, vị trí mới = vị trí cũ + delta. Tọa độ mới được chuyển qua IK để dịch thành góc khớp. AI hoạt động giống "phi công lái joystick" điều hướng mũi kẹp liên tục.

### 2.4 Kiến trúc Hybrid Gripper

AI **KHÔNG** quyết định đóng/mở kẹp. Môi trường tự động xử lý:

| Pha | Điều kiện kích hoạt | Gripper |
|-----|---------------------|---------|
| Phase 0 — Approach | EE gần vật < 4.5cm | Tự động hút |
| Phase 1 — Carry | Đang giữ vật | Giữ chặt |
| Phase 2 — Place | EE gần bin < 5cm | Tự động nhả |

**Tại sao thiết kế này?** Nếu cho AI tự quyết định kẹp, nó sẽ phát hiện "lỗ hổng" trong hàm thưởng — gắp rồi nhả liên tục để nhận thưởng lẻ tẻ mà không bao giờ mang vật đi (Reward Hacking). Hybrid Gripper loại bỏ triệt để hiện tượng này, để AI chỉ tập trung học điều hướng.

### 2.5 Hệ thống thưởng theo Pha (Phase-Based Reward)

**Phase 0 — Approach:**
- Dense: `max(0, 2.0 - khoảng_cách_đến_vật × 8.0)` (tối đa +2.0/step)
- Bonus gắp thành công: **+50 điểm** (1 lần)

**Phase 1 — Carry:**
- Bonus nâng ≥ 20cm: **+30 điểm** (1 lần)
- Dense: `max(0, 2.0 - khoảng_cách_XY_đến_bin × 5.0)`
- Phạt bay thấp (EE < 60cm): **-3.0/step**

**Phase 2 — Place:**
- Dense: `max(0, 3.0 - khoảng_cách_3D_đến_bin × 10.0)`
- Bonus vật rơi vào thùng: **+500 điểm** (lớn nhất, khuyến khích hoàn thành)

**Time Penalty:** -0.05/step (khuyến khích hoàn thành nhanh).

---

## 3. Sự kết hợp giữa Toán học Cổ điển và AI

AI hoạt động ở tầng cao — suy nghĩ chiến lược "bay đi đâu?" (tọa độ XYZ). Bộ IK hoạt động ở tầng thấp — thực thi "xoay khớp bao nhiêu độ?" (6 góc Joint). Bộ FK cung cấp tọa độ EE hiện tại cho Observation Space.

AI không cần tự tìm ra quan hệ phi tuyến giữa 6 góc xoay và vị trí không gian — bài toán đó đã được giải chính xác bởi ma trận DH.
