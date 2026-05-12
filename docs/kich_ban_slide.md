# KỊCH BẢN SLIDE THUYẾT TRÌNH ĐỒ ÁN MÔN HỌC
# Robot UR5e — Pick & Place với Học Tăng Cường (SAC)
# Tổng cộng: ~20 slides

---

## SLIDE 1 — TRANG BÌA
**Nội dung:**
- Tên đề tài: "Hệ thống Điều khiển & Mô phỏng Robot UR5e — Ứng dụng Học Tăng Cường cho bài toán Pick & Place"
- Họ tên sinh viên
- GVHD
- Khoa / Trường
- Năm 2026

**Hình ảnh:** Ảnh chụp giao diện HMI + hình 3D robot UR5e trong PyBullet

---

## SLIDE 2 — NỘI DUNG TRÌNH BÀY
**Nội dung:**
1. Đặt vấn đề & Mục tiêu
2. Cơ sở lý thuyết (FK, IK, RL)
3. Thiết kế hệ thống
4. Chế độ Auto (FSM)
5. Chế độ AI (SAC RL)
6. Kết quả & Demo
7. Kết luận & Hướng phát triển

---

## SLIDE 3 — ĐẶT VẤN ĐỀ
**Nội dung:**
- Robot cộng tác (Cobot) cần thực hiện thao tác Pick & Place trong sản xuất
- Thách thức: vật thể nằm ngẫu nhiên, cần tính quỹ đạo tránh va chạm, 6 bậc tự do
- Câu hỏi nghiên cứu: "Liệu AI (RL) có thể tự học điều khiển Cobot Pick & Place mà không cần lập trình cứng quỹ đạo?"

**Hình ảnh:** Sơ đồ so sánh 2 hướng:
```
┌────────────────────┐     ┌────────────────────┐
│  TRUYỀN THỐNG      │     │  HỌC TĂNG CƯỜNG    │
│  FSM + IK + Traj   │ VS  │  SAC Deep RL       │
│  ✓ Ổn định         │     │  ✓ Tự thích nghi   │
│  ✗ Cứng nhắc       │     │  ✗ Training lâu    │
└────────────────────┘     └────────────────────┘
```

---

## SLIDE 4 — MỤC TIÊU ĐỒ ÁN
**Nội dung (Bullet points):**
1. ✅ Xây dựng mô phỏng 3D robot UR5e (PyBullet)
2. ✅ Tự lập trình FK/IK theo DH convention (không dùng black-box)
3. ✅ 3 chế độ điều khiển: Manual · Auto · AI
4. ✅ Train AI bằng SAC + Curriculum Learning
5. ✅ Giao diện HMI chuyên nghiệp (PyQt5)

**Hình ảnh:** Screenshot giao diện HMI với 4 tab

---

## SLIDE 5 — CÔNG NGHỆ SỬ DỤNG
**Nội dung (Bảng/Icons):**

| Thành phần | Công nghệ |
|---|---|
| 🐍 Ngôn ngữ | Python 3.10+ |
| 🎮 Vật lý | PyBullet |
| 🖥️ Giao diện | PyQt5 |
| 🧠 RL | Stable-Baselines3 (SAC) |
| 🔥 Deep Learning | PyTorch |
| 📐 Giải IK | SciPy (L-BFGS-B) |

---

## SLIDE 6 — KIẾN TRÚC HỆ THỐNG
**Nội dung:** Sơ đồ block diagram

```
┌─────────────────────────────────────────────────┐
│               HMI (PyQt5)                       │
│  [Manual] [Trajectory] [Auto/FSM] [AI/RL]       │
└──────────────────┬──────────────────────────────┘
                   │ Queue
┌──────────────────▼──────────────────────────────┐
│           SimBridge (Thread)                     │
│  Manual Jog │ FSM+IK+Traj │ SAC RL Agent        │
└──────────────────┬──────────────────────────────┘
                   │
┌──────────────────▼──────────────────────────────┐
│         PyBullet Physics (240 Hz)                │
│   UR5e URDF · Object · Bin · Vacuum Gripper      │
└─────────────────────────────────────────────────┘
```

---

## SLIDE 7 — ĐỘNG HỌC THUẬN (FK)
**Nội dung:**
- Quy ước DH: nhân chuỗi 6 ma trận biến đổi thuần nhất 4×4
- Công thức: `T_EE = T₁ × T₂ × T₃ × T₄ × T₅ × T₆`
- T_i = `Rot_z(θ_i) × Trans_z(d_i) × Trans_x(a_i) × Rot_x(α_i)`

**Bảng tham số DH của UR5e:**
| Khớp | a (m) | d (m) | α (rad) | Chức năng |
|---|---|---|---|---|
| 1 | 0.0000 | 0.1625 | π/2 | Shoulder Pan |
| 2 | −0.4250 | 0.0000 | 0 | Shoulder Lift |
| 3 | −0.3922 | 0.0000 | 0 | Elbow |
| 4 | 0.0000 | 0.1333 | π/2 | Wrist 1 |
| 5 | 0.0000 | 0.0997 | −π/2 | Wrist 2 |
| 6 | 0.0000 | 0.0996 | 0 | Wrist 3 |

**Hình ảnh đề xuất:** 
- Hình minh họa 3D robot UR5e với các trục vector XYZ gắn tại từng khớp.

---

## SLIDE 7B — HIỆN THỰC BẢNG DH VÀO PYTHON & KIỂM CHỨNG MATLAB
**Nội dung (Bảo vệ trước Hội đồng):**
- **Đưa vào Python:** Bảng DH được thiết lập theo quy tắc gán trục cơ bản, sau đó viết hàm `dh_transform(a, d, alpha, theta)` bằng `numpy` để nhân liên tiếp 6 ma trận 4x4 theo đúng công thức $T_0^6$.
- **Giải thích thông số:** Tại sao $a_4, a_5, a_6 = 0$? Vì ở cụm cổ tay, các trục quay cắt nhau tại 1 điểm (Spherical Wrist), triệt tiêu đường vuông góc chung. Số liệu chính xác được tham chiếu từ gốc URDF.
- **Kiểm chứng độc lập (Verify):** 
  - Đưa ngược thông số DH sang phần mềm Matlab (Robotics System Toolbox / Peter Corke).
  - Cho robot chạy về vị trí Home Pose trên cả 2 môi trường.
  - **Kết quả:** Ma trận T cuối cùng trên Python tự code và Matlab trùng khớp 100% (sai số = 0), chứng minh phần lõi Động học do nhóm tự lập trình là tuyệt đối chính xác!

**Hình ảnh đề xuất:**
- (Trái) Snippet code mảng `dh_table` trong file `forward_kinematics.py` của bạn.
- (Phải) Ảnh chụp màn hình Command Window của Matlab in ra ma trận kết quả kế bên Terminal của Python.

---

## SLIDE 8 — ĐỘNG HỌC NGHỊCH (IK)
**Nội dung:**
- Bài toán: biết (x,y,z) + hướng → tìm [q1...q6]
- Hybrid Solver 2 lớp:
  - Lớp 1: Analytical (giải tích) → < 1ms, 8 nghiệm
  - Lớp 2: Numerical (L-BFGS-B) → < 10ms, dùng khi Lớp 1 fail

**Hình ảnh:** Flowchart:
```
Input (xyz, euler) → Analytical Solver
                         │
                    Thành công? ──Yes──→ Chọn nghiệm gần HOME nhất
                         │
                        No
                         │
                    Numerical Solver (L-BFGS-B)
                         │
                    Output: [q1...q6]
```

---

## SLIDE 9 — QUY HOẠCH QUỸ ĐẠO
**Nội dung:**
- Joint Space: nội suy góc khớp, velocity profile hình thang
- Cartesian Space: nội suy XYZ → gọi IK mỗi điểm → ghép Joint Traj
- Đảm bảo v_max, a_max, chuyển tiếp mượt

**Hình ảnh:** Đồ thị velocity profile hình thang (trapezoidal):
```
velocity
  │     ___________
  │    /           \
  │   /             \
  │  /               \
  │ /                 \
  └──────────────────── time
  accelerate  cruise  decelerate
```

---

## SLIDE 10 — CHẾ ĐỘ AUTO (FSM)
**Nội dung:**
- 11 trạng thái tuần tự
- Mỗi state chạy 1 trajectory, khi xong → chuyển state tiếp
- Timeout 15s/state → ERROR nếu kẹt

**Hình ảnh:** Sơ đồ FSM:
```
IDLE → DETECT → APPROACH → DESCEND → PICK
                                       │
DONE ← RETREAT ← RELEASE ← PLACE ← LIFT ← MOVE_TO_BIN
```

---

## SLIDE 11 — ỨNG DỤNG AI VÀO ĐIỀU HƯỚNG QUỸ ĐẠO
**Nội dung:**
- Tại sao dùng AI? Chế độ Auto rất tốt nhưng khi vật bị xê dịch khỏi vị trí lập trình sẵn, hệ thống FSM sẽ báo lỗi. AI giúp robot tự xoay sở tìm đường linh hoạt.
- Lựa chọn thuật toán: Nhóm chọn Soft Actor-Critic (SAC). Điểm mạnh của SAC là điều khiển góc xoay động cơ theo tín hiệu liên tục (Continuous Action) vô cùng trơn tru, phù hợp điều khiển cánh tay robot.
- Kiến trúc mạng: 1 mạng Actor (chuyên ra quyết định bay đi đâu) và 2 mạng Critics (chấm điểm xem đường bay đó tốt hay dở).

**Hình ảnh:** Sơ đồ vòng lặp Agent (SAC) ↔ Environment:
```
       ┌──── Continuous Action (ΔXYZ, ΔRPY) ────►┐
       │                                         │
   [Actor SAC]                              [PyBullet]
       │                                         │
       └◄────────── Reward & 20D State ──────────┘
```

---

## SLIDE 12 — CƠ CHẾ HYBRID GRIPPER & PHYSICS CLAMP
**Nội dung:**
- Để AI không bị "ngáo", nhóm áp dụng 2 đột phá cơ điện tử thay vì phó mặc 100% cho AI:
- Đột phá 1 (Hybrid Gripper): AI KHÔNG được quyền bật/tắt giác hút chân không. Việc này do cảm biến tiệm cận phụ trách (cách <4.5cm thì hút, tới nắp thùng thì nhả). AI chỉ tập trung 100% vào việc lái cánh tay.
- Đột phá 2 (Physics-level Euler Clamp): Gốc tọa độ cổ tay bị kẹp chặt bằng thuật toán vật lý. Góc nghiêng (Roll-Pitch) chỉ được dao động ±15°, ép cánh tay luôn ở tư thế 90° chúc thẳng xuống mặt bàn chuẩn xác như Auto.

**Hình ảnh:**
```
[Physics Clamp] ──► Giữ thẳng tay 90°
[Hybrid Gripper] ─► Cảm biến tự động hút/nhả
       │
      Tạo môi trường an toàn để AI chỉ tập trung học Lái (Navigate)
```

---

## SLIDE 13 — OBSERVATION & ACTION SPACE
**Nội dung:**

**Observation (20D):**
| 0-2 | Vị trí EE (xyz) |
| 3-5 | Vị trí vật (xyz) |
| 6-8 | Vector EE→Vật |
| 9-11 | Vector Vật→Bin |
| 12-15 | Quaternion hướng vật |
| 16 | Trạng thái gripper |
| 17-19 | EE euler (roll, pitch, yaw) |

**Action (7D):**
| 0-2 | Δxyz (±5cm/step) |
| 3-5 | ΔRoll/Pitch/Yaw (±4.5°/step) |
| 6 | *(Không sử dụng — Hybrid Gripper tự xử lý)* |

---

## SLIDE 14 — THIẾT KẾ HÀM REWARD (Phần quan trọng nhất!)
**Nội dung:**
- Vấn đề: Reward Hacking — AI gian lận ăn điểm
- Giải pháp: Phase-Based Reward

**Hình ảnh:** Bảng 3 pha:
```
┌─────────────────────────────────┐
│  PHA 0 — APPROACH & GRASP      │
│  Thưởng gần vật: +2.0          │
│  ★ GẮP ĐƯỢC: +50 → Pha 1      │
├─────────────────────────────────┤
│  PHA 1 — CARRY                  │
│  Nâng ≥ 20cm: +30              │
│  Bay thấp: −3 (tránh va chạm)  │
│  Gần bin XY < 15cm → Pha 2     │
├─────────────────────────────────┤
│  PHA 2 — PLACE                  │
│  Hạ đúng vị trí: +3.0          │
│  ★ VÀO BIN: +500 → DONE ✓     │
└─────────────────────────────────┘
```

**Lưu ý đột phá:** Nhờ 2 cơ chế Hybrid Gripper tự hút/nhả và Physics Clamp giữ thẳng tay (vừa trình bày ở slide trước), hàm Reward của nhóm được tối giản cực kỳ gọn nhẹ, AI hoàn toàn không có cơ hội "Reward Hacking" (gian lận điểm).

---

## SLIDE 15 — QUÁ TRÌNH TRAINING
**Nội dung:**
- Train từ đầu (from scratch) — 1 lần duy nhất, KHÔNG dùng Curriculum Learning
- Script: `train_17d_place.py` — train trực tiếp toàn bộ quy trình Pick & Place
- Hội tụ tốt nhờ 3 đột phá: Hybrid Gripper + Phase-Based Reward + Physics Clamp
- File `train_17d_grasp.py` tồn tại như bản thiết kế Curriculum nhưng không cần sử dụng

**Hình ảnh:**
```
┌─────────────────────────────────────────────┐
│  train_17d_place.py — FROM SCRATCH          │
│                                             │
│  10M steps │ 16 envs │ ~3 tiếng │ ~600 FPS  │
│                                             │
│  ✓ Hybrid Gripper: AI chỉ học bay           │
│  ✓ Phase-Based Reward: 3 giai đoạn          │
│  ✓ Physics Clamp ±15°: tư thế thẳng đứng   │
│  ✓ VecNormalize: observation + reward       │
│                                             │
│  Kết quả: 100% success rate                 │
└─────────────────────────────────────────────┘
```

---

## SLIDE 16 — KẾT QUẢ TRAINING
**Nội dung:**

| Metric | Giá trị |
|---|---|
| Script | train_17d_place.py (from scratch) |
| Steps | 10,000,000 |
| Envs song song | 16 (SubprocVecEnv) |
| Thời gian | ~3 tiếng |
| Success rate | 100% (Tuyệt đối) |
| FPS | ~600 |
| Hardware | Core i7, 16GB RAM |
| Output | best_model.zip + vecnormalize.pkl |

**Hình ảnh:** Screenshot TensorBoard (nếu có) hoặc bảng log training cuối cùng

---

## SLIDE 17 — SO SÁNH AUTO vs AI
**Nội dung:**

| Tiêu chí | Auto (FSM) | AI (SAC) |
|---|---|---|
| Thành công | 100% ✓ | 100% ✓ (Đã chứng minh) |
| Lập trình quỹ đạo | Cứng nhắc ✗ | Tự tối ưu ✓ |
| Thích nghi | Không ✗ | Có ✓ |
| Tư thế gắp | Thẳng chuẩn công nghiệp | Thẳng chuẩn (nhờ Clamp vật lý) |
| Kết luận | Mất thời gian code logic dài | Linh hoạt, thông minh, code ngắn |

**Hình ảnh:** 2 ảnh so sánh quỹ đạo:
- Auto: đường thẳng vuông góc (approach → descend → lift → move)
- AI: đường cong mượt parabol tự nhiên

---

## SLIDE 18 — DEMO TRỰC TIẾP
**Nội dung:**
- Chạy `python -m hmi.app`
- Demo 3 chế độ: Manual → Auto → AI
- Nhấn mạnh: AI tự tìm đường, không lập trình trước

**Hành động:** Mở phần mềm, chạy live demo cho hội đồng xem

---

## SLIDE 19 — HẠN CHẾ & HƯỚNG PHÁT TRIỂN
**Nội dung:**

**Hạn chế:**
- AI yếu ngoài vùng train (Out-of-Distribution)
- Tọa độ vật từ PyBullet (chưa dùng camera thật)
- Chưa Sim-to-Real

**Hướng phát triển:**
1. Domain Randomization → mở rộng vùng hoạt động
2. Camera RealSense + Point Cloud → thay thế omniscient
3. ur_rtde → kết nối robot UR5e thật
4. Multi-Object → phân loại theo màu/hình

---

## SLIDE 20 — CẢM ƠN
**Nội dung:**
- Cảm ơn GVHD
- Cảm ơn hội đồng
- Q&A

---

# GHI CHÚ CHO NGƯỜI LÀM SLIDE:
1. Tông màu đề xuất: Nền tối (dark theme) + accent xanh dương/tím
2. Font: Roboto hoặc Inter (hiện đại, dễ đọc)
3. Mỗi slide nên có ít nhất 1 hình ảnh/sơ đồ
4. Các sơ đồ ASCII ở trên nên được vẽ lại bằng shape/diagram chuyên nghiệp
5. Slide quan trọng nhất: Slide 14 (Reward Design) — đây là phần nghiên cứu cốt lõi
6. Nên thêm animation cho các sơ đồ FSM và RL loop
