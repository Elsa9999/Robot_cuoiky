# NỘI DUNG CHI TIẾT ĐỒ ÁN MÔN HỌC
# Hệ thống Điều khiển & Mô phỏng Robot UR5e — Ứng dụng Học Tăng Cường cho bài toán Pick & Place

---

## CHƯƠNG 1: GIỚI THIỆU

### 1.1 Đặt vấn đề
Trong sản xuất công nghiệp hiện đại, robot cánh tay (manipulator) đóng vai trò then chốt trong các dây chuyền lắp ráp, đóng gói và phân loại sản phẩm. Thao tác Pick & Place — gắp vật thể từ vị trí này và đặt vào vị trí khác — là một trong những tác vụ cơ bản nhất nhưng cũng đặt ra nhiều thách thức:

- **Vật thể nằm ở vị trí ngẫu nhiên**, có thể đứng thẳng hoặc lăn ngang trên bề mặt.
- **Robot cần tính toán quỹ đạo** di chuyển tránh va chạm với các vật cản xung quanh.
- **Cánh tay 6 bậc tự do (6-DOF)** yêu cầu giải bài toán Động học Nghịch (Inverse Kinematics) phức tạp.

Phương pháp truyền thống sử dụng lập trình cứng (hard-coded trajectory) giải quyết ổn định nhưng thiếu tính linh hoạt khi môi trường thay đổi. Học Tăng Cường (Reinforcement Learning — RL) là hướng tiếp cận hiện đại cho phép robot tự học từ kinh nghiệm tương tác với môi trường, không cần lập trình trước quỹ đạo.

### 1.2 Mục tiêu đồ án
1. Xây dựng môi trường mô phỏng 3D robot UR5e trên PyBullet.
2. Triển khai bộ giải Động học Thuận (FK) và Nghịch (IK) theo quy ước Denavit-Hartenberg.
3. Thiết kế 3 chế độ điều khiển: Manual (tay), Auto (FSM + Trajectory Planning), AI (SAC RL).
4. Huấn luyện agent AI thực hiện Pick & Place bằng SAC với Phase-Based Reward + Hybrid Gripper.
5. So sánh hiệu suất giữa chế độ Auto (deterministic) và AI (adaptive).

### 1.3 Phạm vi đồ án
- **Robot:** Universal Robots UR5e — 6 bậc tự do (DOF), tải trọng 5kg.
- **Môi trường:** Mô phỏng PyBullet 240Hz, URDF model chuẩn.
- **Vật thể:** Hình trụ (cylinder), bán kính 2cm, chiều cao 6cm, khối lượng 100g.
- **Gripper:** Giác hút chân không mô phỏng bằng Fixed Constraint.
- **Giao diện:** HMI PyQt5 với 4 tab: Manual, Trajectory, Auto, AI.

### 1.4 Công nghệ sử dụng
| Thành phần | Công nghệ | Phiên bản |
|---|---|---|
| Ngôn ngữ lập trình | Python | 3.10+ |
| Engine vật lý | PyBullet | 3.2+ |
| Giao diện người dùng | PyQt5 | 5.15+ |
| Thuật toán RL | Stable-Baselines3 (SAC) | 2.0+ |
| Deep Learning backend | PyTorch | 2.0+ |
| Giải phương trình phi tuyến | SciPy (L-BFGS-B) | 1.10+ |
| Trực quan hóa training | TensorBoard | 2.0+ |

### 1.5 Cấu trúc thư mục dự án
```
do_an_robot_v2/
├── kinematics/                 # Động học — Tự code bằng NumPy/SciPy
│   ├── forward_kinematics.py    # FK — 6 góc khớp → vị trí EE
│   ├── inverse_kinematics.py    # IK Hybrid — Analytical + L-BFGS-B
│   ├── trajectory.py            # Quy hoạch quỹ đạo Joint/Cartesian
│   └── workspace_validator.py   # Kiểm tra giới hạn vùng làm việc
├── simulation/                 # Mô phỏng vật lý PyBullet
│   ├── environment.py           # Thế giới vật lý (bàn, bin, robot, vật)
│   ├── gripper.py               # Giác hút chân không (Vacuum Gripper)
│   ├── object_detector.py       # Camera ảo (Raycast)
│   ├── pick_place_sm.py         # FSM 11 trạng thái cho chế độ Auto
│   ├── trajectory_executor.py   # Bộ thực thi quỹ đạo (240Hz)
│   └── manual_controller.py     # Điều khiển tay bằng HMI
├── hmi/                        # Giao diện người dùng PyQt5
│   ├── app.py                   # Entry point chính
│   ├── main_window.py           # Cửa sổ chính + layout
│   ├── sim_bridge.py            # Cầu nối HMI ↔ PyBullet (QThread)
│   └── widgets/                 # 7 panels giao diện
├── utils/
│   └── transforms.py            # Chuyển hệ tọa độ DH ↔ PyBullet
├── urdf/                       # Mô hình 3D robot UR5e (URDF + mesh)
├── models_rl_17d/              # Model SAC đã train (best_model.zip)
├── train_17d_grasp.py          # Script train Phase 1
├── train_17d_place.py          # Script train Phase 2
├── run_demo.py                 # Chạy demo AI (inference)
└── docs/                       # Tài liệu
```

---

## CHƯƠNG 2: CƠ SỞ LÝ THUYẾT

### 2.1 Động học Robot — Quy ước Denavit-Hartenberg (DH)

#### 2.1.1 Động học Thuận (Forward Kinematics — FK)
Động học Thuận là bài toán xác định vị trí và hướng của đầu công tác (End-Effector — EE) khi biết giá trị các góc khớp.

**Công thức:**
Ma trận biến đổi thuần nhất 4×4 cho mỗi khớp:

T_i = Rot_z(θ_i) × Trans_z(d_i) × Trans_x(a_i) × Rot_x(α_i)

Ma trận tổng hợp từ gốc đến EE:

T_EE = T_1 × T_2 × T_3 × T_4 × T_5 × T_6

**Bảng thông số DH của UR5e:**

| Khớp | a (m) | d (m) | α (rad) | Chức năng |
|------|-------|-------|---------|-----------|
| 1 | 0.0000 | 0.1625 | π/2 | Shoulder Pan |
| 2 | −0.4250 | 0.0000 | 0 | Shoulder Lift |
| 3 | −0.3922 | 0.0000 | 0 | Elbow |
| 4 | 0.0000 | 0.1333 | π/2 | Wrist 1 |
| 5 | 0.0000 | 0.0997 | −π/2 | Wrist 2 |
| 6 | 0.0000 | 0.0996 | 0 | Wrist 3 |

#### 2.1.2 Động học Nghịch (Inverse Kinematics — IK)
Bài toán ngược: tìm 6 góc khớp [q1...q6] khi biết vị trí (x,y,z) và hướng mong muốn của EE.

**Bộ giải Hybrid 2 lớp được triển khai:**
- **Lớp 1 — Analytical Solver:** Tính nghiệm giải tích kín từ hình học lượng giác. Tốc độ < 1ms. Trả về tối đa 8 nghiệm, chọn nghiệm gần HOME nhất. Ưu tiên vì nhanh và chính xác.
- **Lớp 2 — Numerical Solver (L-BFGS-B):** Khi Lớp 1 thất bại (singularity, out-of-workspace), thuật toán tối ưu hóa L-BFGS-B tiếp quản. Minimize đồng thời lỗi vị trí (position error) và lỗi hướng (orientation error). Tốc độ < 10ms.

**Tiêu chí chọn nghiệm:**
- Gần nhất với cấu hình hiện tại (tránh unwinding nhiều vòng).
- Nằm trong giới hạn khớp (±2π rad).
- Không gây singularity (det(Jacobian) > ε).

### 2.2 Quy hoạch Quỹ đạo (Trajectory Planning)

#### 2.2.1 Joint Space Trajectory
- Nội suy trực tiếp các góc khớp từ cấu hình bắt đầu đến cấu hình kết thúc.
- Velocity profile hình thang (Trapezoidal): giai đoạn tăng tốc → vận tốc đều → giảm tốc.
- Giới hạn vận tốc tối đa v_max = 1.0 rad/s và gia tốc a_max = 0.5 rad/s².
- Đảm bảo chuyển động mượt mà, không gây giật cơ (jerk).

#### 2.2.2 Cartesian Space Trajectory
- Chia đoạn thẳng từ điểm bắt đầu → kết thúc thành N điểm trung gian trong không gian Cartesian.
- Mỗi điểm trung gian được giải IK để tìm góc khớp tương ứng.
- Ghép các góc khớp thành Joint Trajectory liên tục.
- Ưu điểm: EE di chuyển theo đường thẳng chính xác (quan trọng khi tiếp cận vật theo trục Z).

### 2.3 Điều Hướng Tự Động Bằng Học Tăng Cường (RL)

Thay vì sử dụng các thuật toán nội suy quỹ đạo Cartesian/Joint cứng nhắc (như trình bày ở phần 2.2), hệ thống được tích hợp Học Tăng Cường để tay máy có thể tự "mò đường" (Navigate) một cách linh hoạt trong không gian 3D, đặc biệt khi vật thể bị xê dịch khỏi tọa độ cho trước.

#### 2.3.1 Khái niệm cơ bản
Học Tăng Cường xem thuật toán AI như một Blackbox (Agent) tương tác với Môi trường Vật lý (Environment):
1. Agent quan sát trạng thái (State) từ các cảm biến/tọa độ.
2. Agent xuất tín hiệu điều khiển (Action) xuống các khớp động cơ.
3. Môi trường kiểm tra va chạm, sai số và trả về Điểm thưởng/phạt (Reward).
4. Agent tự động cập nhật mạng nơ-ron để tối đa hóa điểm số.

#### 2.3.2 Ứng dụng Thuật toán Soft Actor-Critic (SAC)
SAC là thuật toán Off-Policy thuộc họ Actor-Critic, tối ưu hóa mục tiêu entropy-regularized (entropy-regularized objective):

π* = argmax_π E[Σ γ^t (r_t + α H(π(·|s_t)))]

Trong đó:
- γ: hệ số chiết khấu (discount factor) = 0.99
- α: hệ số entropy, tự điều chỉnh (auto-tuning)
- H(π): entropy của chính sách — khuyến khích đa dạng hành động

Dù dựa trên nền tảng toán học phức tạp, đồ án tập trung khai thác các đặc tính cơ điện tử của thuật toán này khi áp dụng vào thực tế:

**Tại sao chọn SAC cho Robot UR5e?**
- **Continuous Action (Hành động liên tục):** Khác với các AI xử lý logic rời rạc, SAC xuất ra dải giá trị số thực liên tục. Điều này cực kỳ phù hợp để làm tín hiệu cấp cho các bộ điều khiển nội suy góc xoay động cơ servo, giúp cánh tay di chuyển trơn tru.
- **Sample-efficient:** Khả năng tận dụng lại dữ liệu cũ (Off-policy) giúp robot học nhanh hơn nhiều so với thuật toán On-Policy thông thường.
- **Tự cân bằng Khám phá:** Nhờ tối đa hóa thành phần Entropy H(π), robot không bao giờ đi đúng một đường cố định mà luôn lân la tìm quỹ đạo bay mới để tối ưu hóa.

**Kiến trúc mạng điều hướng:**
- **Mạng Actor (Quyết định):** MLP [256 → 256]. Nhận đầu vào là tọa độ không gian (20D) → Xuất ra vector vận tốc (7D).
- **Mạng Critic 1 & 2 (Chấm điểm):** MLP [256 → 256] → Q-value. Áp dụng Twin Critic để chống hiện tượng đánh giá quá mức (overestimation).

---

## CHƯƠNG 3: THIẾT KẾ HỆ THỐNG

### 3.1 Kiến trúc tổng thể
Hệ thống chia thành 4 tầng:
1. **Tầng Giao diện (HMI):** PyQt5, chạy trên Main Thread. 4 tab chức năng + 2 panel phụ.
2. **Tầng Điều khiển (SimBridge):** QThread riêng chạy ở 240Hz, xử lý lệnh từ HMI qua command_queue và trả kết quả qua state_queue. Điều phối 3 chế độ: Manual, Auto (FSM), AI (SAC).
3. **Tầng Thuật toán:** FK/IK Solver, Trajectory Planner, FSM Controller, SAC Agent.
4. **Tầng Vật lý (PyBullet):** Mô phỏng 240Hz, collision detection, constraint-based gripper.

#### Giao diện HMI (hmi/)
| Tab / Panel | File | Chức năng |
|---|---|---|
| Joint Panel | `joint_panel.py` | 6 thanh trượt điều khiển từng góc khớp q1-q6 |
| Cartesian Panel | `cartesian_panel.py` | Nhập XYZ + RPY, nút Jog ±1cm từng trục |
| Trajectory Panel | `trajectory_panel.py` | Tạo waypoints, chọn Joint/Cartesian, điều chỉnh tốc độ |
| Auto Panel | `auto_panel.py` | Start/Stop FSM, hiển thị 11 trạng thái + cycle count |
| AI Panel | `ai_panel.py` | Start/Stop SAC, hiển thị success count |
| Status Panel | `status_panel.py` | Real-time: XYZ, 6 góc khớp, gripper, chế độ |
| Log Panel | `log_panel.py` | Console log sự kiện: IK, lỗi, AI reward |

#### SimBridge — Cầu nối HMI ↔ PyBullet (hmi/sim_bridge.py)
- Chạy trên QThread riêng ở 240Hz, độc lập với UI thread để không gây lag.
- Nhận lệnh từ GUI qua `command_queue` (set_joints, jog_cartesian, start_auto, start_ai...).
- Trả trạng thái qua `state_queue` mỗi 24 steps (~10Hz): tọa độ EE, góc khớp, gripper, mode.
- Load SAC model khi khởi tạo, tự phát hiện 17D/20D model + VecNormalize stats.

### 3.1.1 Chuyển đổi Hệ tọa độ (utils/transforms.py)
Robot UR5e trong PyBullet bị **xoay 180°** quanh trục Z so với hệ DH chuẩn, và đế đặt trên bàn cao **0.42m**:
- `local_to_world(pos, euler)`: Đảo dấu X,Y + cộng 0.42m vào Z + trừ π khỏi Yaw.
- `world_to_local(pos, euler)`: Phép biến đổi ngược.
- Gọi mỗi khi truyền tọa độ giữa FK/IK ↔ HMI/PyBullet.

### 3.1.2 Giác hút Chân không (simulation/gripper.py)
**Nguyên lý:** Trong thực tế, giác hút dùng bơm chân không tạo áp suất âm. Trong mô phỏng, dùng PyBullet Constraint loại `JOINT_FIXED` để "dán cứng" vật vào EE.

- **Kích hoạt:** Tính offset vật so với EE trong hệ tọa độ cục bộ (Local Frame) bằng `invertTransform` + `multiplyTransforms`, sau đó tạo Constraint với `maxForce=500N`. Bước tính offset **cực kỳ quan trọng** — nếu không, vật bị kéo giật về tâm EE gây Physics Explosion.
- **Nhả:** Xóa Constraint → vật rơi tự do theo trọng lực.
- **Visual indicator:** Vẽ vòng tròn 3D quanh đầu hút — xanh lá (đang hút) / đỏ (đã nhả).

### 3.1.3 Camera ảo — Raycast Detector (simulation/object_detector.py)
- Bắn tia laser ảo từ EE thẳng xuống (trục Z âm), tầm xa 0.5m.
- Nếu chạm vật → Vẽ tia xanh + trả về tọa độ (X,Y,Z) chính xác.
- Nếu không chạm → Vẽ tia đỏ + trả về None.
- Tính sẵn 3 tư thế gắp: Approach (trên vật 15cm), Pick (cách vật 1cm), Lift (trên vật 20cm).

### 3.1.4 Curriculum Difficulty (simulation/environment.py)
| Level | Bán kính spawn | Vật nằm ngang | Mô tả |
|---|---|---|---|
| 0 | 12-25cm từ HOME | Không | Dễ — vật đứng thẳng, gần tay |
| 1 | 5-25cm từ HOME | 50% xác suất | Vừa — vật có thể lăn ngang |
| 2 | Full WORK_ZONE | 50% xác suất | Khó — random hoàn toàn |

### 3.2 Chế độ Manual
- Người dùng điều khiển trực tiếp EE qua giao diện HMI.
- Jog Cartesian: dịch chuyển EE theo từng trục X, Y, Z với bước tùy chỉnh.
- Jog Joint: thay đổi trực tiếp từng góc khớp q1-q6.
- Workspace Validator kiểm tra giới hạn an toàn (X: 0.20-0.75m, Y: -0.30-0.30m, Z: 0.44-0.95m).

### 3.3 Chế độ Auto (FSM + Trajectory Planning)
Cỗ máy trạng thái hữu hạn (Finite State Machine) điều phối 11 trạng thái:

IDLE → DETECT → APPROACH → DESCEND → PICK → LIFT → MOVE_TO_BIN → PLACE → RELEASE → RETREAT → DONE

| Trạng thái | Hành động | Loại trajectory | Tốc độ |
|---|---|---|---|
| APPROACH | Di chuyển đến 15cm phía trên vật | Cartesian | 0.12 m/s |
| DESCEND | Hạ thẳng đứng xuống vị trí gắp | Cartesian | 0.05 m/s |
| PICK | Dwell 0.3s + kích hoạt giác hút | - | - |
| LIFT | Nâng vật lên 20cm | Cartesian | 0.08 m/s |
| MOVE_TO_BIN | Bay ngang đến phía trên bin | Cartesian | 0.15 m/s |
| PLACE | Hạ xuống vị trí thả | Cartesian | 0.05 m/s |
| RELEASE | Dwell 0.5s + tháo giác hút | - | - |
| RETREAT | Về HOME_POSE | Joint | 1.0 rad/s |

Timeout mỗi state: 15 giây. Nếu quá thời gian → ERROR.

### 3.4 Chế độ AI (Tự động điều hướng linh hoạt)

#### 3.4.1 Không gian Quan sát (Observation — 20D)
| Index | Ý nghĩa | Chiều |
|---|---|---|
| 0-2 | Vị trí EE (x, y, z) | 3 |
| 3-5 | Vị trí vật thể (x, y, z) | 3 |
| 6-8 | Vector tương đối EE → Vật | 3 |
| 9-11 | Vector tương đối Vật → Bin | 3 |
| 12-15 | Quaternion hướng vật (qx, qy, qz, qw) | 4 |
| 16 | Trạng thái gripper (0 hoặc 1) | 1 |
| 17-19 | EE euler (roll, pitch, yaw) | 3 |

Quaternion (4D) cho phép AI nhận biết vật đang đứng hay nằm ngang. EE euler (3D) giúp AI giám sát tư thế cổ tay.

#### 3.4.2 Không gian Hành động (Action — 7D)
| Index | Ý nghĩa | Phạm vi | Đơn vị |
|---|---|---|---|
| 0-2 | Δx, Δy, Δz (dịch chuyển EE) | [-1, 1] × 5cm | cm/step |
| 3-5 | ΔRoll, ΔPitch, ΔYaw (xoay cổ tay) | [-1, 1] × 4.5° | °/step |
| 6 | *(Không sử dụng — Hybrid Gripper tự xử lý)* | [-1, 1] | - |

**Nguyên lý Hybrid Gripper:** AI KHÔNG điều khiển gripper. Pha 0: gripper tự gắp khi EE gần vật < 4.5cm. Pha 1: giữ chặt. Pha 2: tự nhả khi EE gần bin < 5cm. Loại bỏ hoàn toàn Reward Hacking về gripper timing.

#### 3.4.3 Thiết kế Hàm Reward — Phase-Based Architecture
Đây là phần quan trọng nhất và đã trải qua 3 lần thiết kế lại:

**Lần 1 — Sparse Reward:** Chỉ thưởng +500 khi rác vào bin. Kết quả: AI mò mẫm quá lâu, không hội tụ.

**Lần 2 — Dense Reward (Parabolic Shaping):** Phạt bay thấp, phạt xa bin. Kết quả: AI bị Reward Hacking — không bật gripper để tránh bị phạt.

**Lần 3 — Phase-Based Reward (phiên bản hiện tại):**
Chia rõ 3 giai đoạn + time penalty mỗi step: −0.05

PHA 0 — APPROACH & GRASP:
  - Dense reward tiến gần vật: max(0, 2.0 − dist × 8.0)
  - Thưởng gắp thành công: +50 → chuyển sang Pha 1

PHA 1 — CARRY:
  - Thưởng nâng cao (one-time): +30 khi nâng ≥ 20cm
  - Phạt bay thấp (EE < 0.60m): −3.0 (tránh va chạm thành thùng)
  - Dense reward tiến về bin: max(0, 2.0 − dist_xy × 5.0)
  - Khi đã nâng + gần bin (< 15cm XY) → chuyển Pha 2

PHA 2 — PLACE:
  - Dense reward hạ chính xác: max(0, 3.0 − dist_3d × 10.0)
  - Thưởng thả thành công vào bin: +500 → KẾT THÚC

#### 3.4.4 Siêu tham số SAC (Hyperparameters)

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| learning_rate | 3e-4 | Tốc độ học Adam (chuẩn SAC paper) |
| gamma | 0.99 | Hệ số chiết khấu — task dài (~300 steps) |
| batch_size | 256 | Kích thước batch |
| tau | 0.005 | Polyak averaging (chuẩn paper) |
| buffer_size | 1,000,000 | Replay buffer — đủ cho 10M steps off-policy |
| learning_starts | 10,000 | Warm-up exploration trước khi học |
| gradient_steps | 1 | Gradient updates/step — ổn định |
| ent_coef | auto_0.1 | Entropy tự điều chỉnh, khởi tạo 0.1 |
| net_arch | [256, 256] | Kiến trúc Actor/Critic MLP |
| use_sde | False | State-Dependent Exploration OFF |

#### 3.4.5 Quá trình Training (train_17d_place.py)

Hệ thống chỉ thực hiện **một lần training duy nhất** từ đầu (from scratch), không sử dụng Curriculum Learning hay Transfer Learning:

- **Script:** `train_17d_place.py` — train trực tiếp toàn bộ quy trình Pick & Place.
- **Observation:** 20D (thêm 3D EE euler để giám sát tư thế cổ tay).
- **MAX_STEPS:** 300 (~12.5 giây simulation mỗi episode).
- **Đột phá 1 — Hybrid Gripper:** AI KHÔNG điều khiển gripper, chỉ học navigation. Gripper tự gắp/nhả theo Phase. Loại bỏ hoàn toàn Reward Hacking.
- **Đột phá 2 — Physics-Level Euler Clamp (±15°):** Trong hàm `move_ee_cartesian()`, Roll được kẹp quanh ±π (±15°), Pitch kẹp quanh 0 (±15°). AI luôn giữ tư thế thẳng đứng mà không cần hàm phạt phức tạp.
- **VecNormalize:** Chuẩn hóa observation (mean=0, std=1) và reward tự động (`clip_obs=10, clip_reward=10`).
- **Training:** 10 triệu steps, 16 parallel envs (SubprocVecEnv), ~3 tiếng trên Core i7, 16GB RAM.
- **Tốc độ:** ~600 FPS nhờ song song hóa 16 luồng CPU.
- **EvalCallback:** Đánh giá mỗi 200K steps, 20 episodes. Lưu `best_model.zip` + `vecnormalize.pkl` khớp nhau.
- **Kết quả:** 100% success rate, tư thế thẳng đứng y hệt Auto Mode.

**Ghi chú:** File `train_17d_grasp.py` tồn tại như bản thiết kế Curriculum Learning 2 giai đoạn, nhưng thực tế không sử dụng vì `train_17d_place.py` đã hội tụ tốt từ đầu nhờ Hybrid Gripper + Phase-Based Reward.

#### 3.4.6 Safety Layers (Lớp bảo vệ khi Inference)
- **Jam Detector:** Nếu EE di chuyển < 2mm trong 200 frames liên tiếp (~3.3 giây) → release gripper + Auto-Home (Phase 3→4: nâng lên → về HOME bằng Joint interpolation). Vật thể KHÔNG bị xóa → AI thử lại.
- **Timeout Detector:** Nếu AI chưa gắp được vật sau 600 frames (~10 giây) → tự động về HOME rồi thử lại. Đếm số lần retry.
- **Workspace Validator:** Clip tọa độ EE vào giới hạn an toàn (X: 0.20-0.75m, Y: -0.30-0.30m, Z: 0.44-0.95m). Bỏ qua validator trong AI mode (robot cần bay vào vùng bin).
- **Dwell Time (0.5s):** Dừng nhịp 15 frames sau khi gắp, mô phỏng thời gian bơm áp suất chân không.
- **Retract Logic (2 pha):** Pha 1: Kéo thẳng EE lên trời (Z > 0.25m) thoát thành thùng. Pha 2: Nội suy Joint Space đưa tay về HOME_POSE (tốc độ 5%/frame). Khi error < 0.05 rad → spawn vật mới.
- **Anti-Unwinding:** Normalize góc IK về [-2π, 2π], chọn góc tương đương gần nhất với joint hiện tại để tránh robot quấn tay nhiều vòng.

---

## CHƯƠNG 4: TRIỂN KHAI & KẾT QUẢ

### 4.1 Môi trường mô phỏng
**Cấu hình vật lý:**
- PyBullet 240 Hz, GUI với camera 3D có thể xoay.
- Trọng lực: g = −9.81 m/s². Real-time simulation OFF (step-by-step để kiểm soát hoàn toàn).

**Địa hình:**
- Bàn gỗ: mặt bàn ở Z = 0.42m (chân 0.40m + dày 0.02m), lateral friction = 1.0.
- Bin thùng rác: 4 thành + 1 đáy, tâm tại [0.65, −0.28, 0.42m], kích thước nửa 9.6×7.1cm.
- Vật cylinder xanh dương: bán kính 2cm, chiều cao 6cm, khối lượng 100g, lateral friction = 0.8.
- Workspace đánh dấu bằng đường viền đỏ trên mặt bàn (WORK_ZONE: X 0.3-0.7m, Y −0.15-0.15m).

**Robot:**
- UR5e load từ URDF chuẩn (ur5e_final.urdf), 6 khớp quay.
- PD controller: Kp = 0.2–0.3, Kd = 0.8–1.0, max force 28–150N tùy khớp.
- HOME_POSE: [0, −π/2, π/2, −π/2, −π/2, 0] rad.
- Gripper mô phỏng bằng JOINT_FIXED constraint, khoảng cách gắp < 4.5cm, maxForce = 500N.

**AI Inference Pipeline (khi chạy HMI):**
1. Load `best_model.zip` + `vecnormalize.pkl` (phải khớp nhau).
2. Mỗi step: thu thập 20D obs → normalize → SAC predict → action 7D → `move_ee_cartesian()` + Hybrid Gripper logic.
3. Hybrid Gripper: Phase 0 — tự gắp khi < 4.5cm | Phase 1 — giữ chặt | Phase 2 — tự nhả khi gần bin.
4. Check `is_in_bin()`: vật nằm trong 5cm từ tâm bin + Z < 0.55m → success.
5. Retract: nâng lên → về HOME → spawn vật mới → lặp lại vô hạn.

### 4.2 Kết quả Training
**Training duy nhất (train_17d_place.py — from scratch):**
- Steps: 10,000,000 | Envs: 16 | Thời gian: ~3 tiếng
- Success rate: 100% (kiểm chứng trên 50 chu kỳ ngẫu nhiên) | FPS: ~600
- Tư thế thao tác (Orientation): Thẳng đứng y hệt Auto Mode (nhờ Physics Clamp ±15°).
- Output: `models_rl_17d/seed42/best_model.zip` + `vecnormalize.pkl`

### 4.3 So sánh Auto vs AI

| Tiêu chí | Auto (FSM) | AI (SAC RL) |
|---|---|---|
| Tỉ lệ thành công | 100% | 100% |
| Cần lập trình quỹ đạo | Có | Không (tự hành vi) |
| Thích nghi vật mới | Cứng nhắc | Rất cao |
| Tốc độ cycle | Nhanh (tối ưu) | Nổi trội (~25 steps/chu kỳ) |
| Tư thế làm việc | Thẳng 90 độ | Thẳng 90 độ (Giới hạn vật lý) |
| Mượt mà | Tuyệt đối | Cực kỳ mượt mà |

### 4.4 Hạn chế
1. AI chỉ hoạt động tốt trong vùng WORK_ZONE đã train (Out-of-Distribution failure).
2. Tọa độ vật lấy trực tiếp từ PyBullet (omniscient) — đời thực cần Camera 3D.
3. Chưa test Sim-to-Real trên robot UR5e thật.
4. Reward Hacking vẫn có thể xảy ra nếu thay đổi môi trường.

---

## CHƯƠNG 5: KẾT LUẬN & HƯỚNG PHÁT TRIỂN

### 5.1 Kết luận
Đồ án đã triển khai thành công:
- Hệ thống mô phỏng 3D robot UR5e hoàn chỉnh trên PyBullet.
- Bộ giải IK Hybrid (Analytical + Numerical) tự lập trình bằng NumPy/SciPy.
- 3 chế độ điều khiển trên cùng một nền tảng: Manual, Auto (FSM), AI (SAC).
- Agent RL đạt 100% success rate trong training thông qua Phase-Based Reward và Curriculum Learning.
- Giao diện HMI chuyên nghiệp với PyQt5 hỗ trợ vận hành và giám sát.

### 5.2 Hướng phát triển
1. Domain Randomization: mở rộng vùng spawn, random kích thước/hình dạng vật.
2. Camera Integration: thay omniscient bằng RealSense D435 + Point Cloud Segmentation.
3. Sim-to-Real Transfer: kết nối robot thật qua thư viện ur_rtde.
4. Multi-Object Sorting: gắp nhiều vật, phân loại theo màu sắc/hình dáng.
5. RLHF: tinh chỉnh reward bằng phản hồi từ người vận hành.
