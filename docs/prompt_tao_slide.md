# PROMPT TẠO SLIDE THUYẾT TRÌNH — Copy & Paste vào AI (Gamma / SlidesAI / ChatGPT)

---

## HƯỚNG DẪN SỬ DỤNG
1. Copy toàn bộ nội dung bên dưới (từ dòng "BẮT ĐẦU PROMPT" trở xuống).
2. Paste vào công cụ AI tạo slide (ví dụ: gamma.app, slidesai.io, hoặc ChatGPT + export PPT).
3. AI sẽ tự động tạo bộ slide ~20 trang hoàn chỉnh.

---

## ═══ BẮT ĐẦU PROMPT ═══

Hãy tạo cho tôi bộ slide PowerPoint thuyết trình đồ án môn học với **20 slides**, tông màu **nền tối (dark theme)** + accent **xanh dương/tím**, font **Roboto hoặc Inter**. Mỗi slide có ít nhất 1 hình minh họa hoặc sơ đồ. Ngôn ngữ: **Tiếng Việt**.

### THÔNG TIN CHUNG
- **Tên đề tài:** "Hệ thống Điều khiển & Mô phỏng Robot UR5e — Ứng dụng Học Tăng Cường cho bài toán Pick & Place"
- **Sinh viên:** [Điền tên]
- **GVHD:** [Điền tên]
- **Trường / Khoa:** [Điền tên]
- **Năm:** 2026

---

### NỘI DUNG TỪNG SLIDE

**SLIDE 1 — TRANG BÌA:**
Tên đề tài, tên sinh viên, GVHD, trường, năm. Hình nền: Robot cộng tác (Cobot) 6 bậc tự do.

**SLIDE 2 — NỘI DUNG TRÌNH BÀY:**
Mục lục 7 phần:
1. Đặt vấn đề & Mục tiêu
2. Cơ sở lý thuyết (FK, IK, RL)
3. Thiết kế hệ thống
4. Chế độ Auto (FSM)
5. Chế độ AI (SAC RL)
6. Kết quả & Demo
7. Kết luận & Hướng phát triển

**SLIDE 3 — ĐẶT VẤN ĐỀ:**
- Robot cộng tác (Cobot) cần thao tác Pick & Place trong sản xuất.
- Thách thức: vật nằm ngẫu nhiên, 6 bậc tự do, quỹ đạo tránh va chạm.
- Câu hỏi: "AI (RL) có thể tự học điều khiển Cobot mà không cần lập trình cứng?"
- Sơ đồ so sánh: Truyền thống (FSM+IK, ổn định nhưng cứng nhắc) vs Học Tăng Cường (SAC, tự thích nghi nhưng training lâu).

**SLIDE 4 — MỤC TIÊU:**
5 mục tiêu (dùng icon ✅):
1. Mô phỏng 3D robot UR5e (PyBullet)
2. Tự lập trình FK/IK theo DH convention
3. 3 chế độ: Manual · Auto · AI
4. Train AI bằng SAC (Huấn luyện 1 phase từ đầu)
5. Giao diện HMI chuyên nghiệp (PyQt5)

**SLIDE 5 — CÔNG NGHỆ SỬ DỤNG:**
Bảng 6 hàng:
| Thành phần | Công nghệ |
|---|---|
| Ngôn ngữ | Python 3.10+ |
| Vật lý | PyBullet |
| Giao diện | PyQt5 |
| RL | Stable-Baselines3 (SAC) |
| Deep Learning | PyTorch |
| Giải IK | SciPy (L-BFGS-B) |

**SLIDE 6 — KIẾN TRÚC HỆ THỐNG:**
Sơ đồ 3 tầng (block diagram):
- Tầng trên: HMI PyQt5 (4 tab: Manual, Trajectory, Auto, AI)
- Tầng giữa: SimBridge Thread (xử lý lệnh, điều phối 3 chế độ)
- Tầng dưới: PyBullet Physics 240Hz (UR5e URDF, Object, Bin, Vacuum Gripper)
- Kết nối bằng Queue giữa các tầng.

**SLIDE 7 — ĐỘNG HỌC THUẬN (FK):**
- Quy ước DH: nhân chuỗi 6 ma trận 4×4
- Công thức: T_EE = T₁ × T₂ × T₃ × T₄ × T₅ × T₆
- Bảng DH 6 khớp:
  | Khớp | a (m) | d (m) | α (rad) |
  |---|---|---|---|
  | 1 | 0.0000 | 0.1625 | π/2 |
  | 2 | −0.4250 | 0.0000 | 0 |
  | 3 | −0.3922 | 0.0000 | 0 |
  | 4 | 0.0000 | 0.1333 | π/2 |
  | 5 | 0.0000 | 0.0997 | −π/2 |
  | 6 | 0.0000 | 0.0996 | 0 |

**SLIDE 7B — KIỂM CHỨNG DH:**
- Bảng DH được code bằng NumPy (hàm dh_transform).
- Kiểm chứng độc lập: so sánh kết quả tính tay (NumPy) với tọa độ trả về từ PyBullet → sai số = 0.
- Hình minh họa: code snippet Python bên trái, khung hình mô phỏng PyBullet bên phải.

**SLIDE 8 — ĐỘNG HỌC NGHỊCH (IK):**
- Hybrid Solver 2 lớp:
  - Lớp 1: Analytical (giải tích) → < 1ms, 8 nghiệm
  - Lớp 2: Numerical (L-BFGS-B) → < 10ms, dự phòng khi Lớp 1 fail
- Flowchart: Input → Analytical → Thành công? → Yes: Chọn nghiệm gần HOME / No: Numerical → Output [q1..q6]

**SLIDE 9 — QUY HOẠCH QUỸ ĐẠO:**
- Joint Space: nội suy góc khớp trực tiếp
- Cartesian Space: nội suy XYZ → IK mỗi điểm
- Velocity profile hình thang (trapezoidal): Tăng tốc → Đều ga → Giảm tốc
- Vẽ đồ thị velocity profile.

**SLIDE 10 — CHẾ ĐỘ AUTO (FSM):**
- 11 trạng thái tuần tự, timeout 15s/state
- Sơ đồ: IDLE → DETECT → APPROACH → DESCEND → PICK → LIFT → MOVE_TO_BIN → PLACE → RELEASE → RETREAT → DONE
- Mỗi state chạy 1 trajectory Cartesian/Joint.

**SLIDE 11 — ỨNG DỤNG AI VÀO ĐIỀU HƯỚNG QUỸ ĐẠO:**
- Tại sao dùng AI? Thay vì lập trình tọa độ cứng nhắc, robot tự học cách bay đến mục tiêu linh hoạt.
- Thuật toán: Soft Actor-Critic (SAC). Chọn SAC vì nó xuất ra lệnh điều khiển động cơ liên tục (Continuous Action) cực kỳ trơn tru, rất phù hợp cho Robot cộng tác (Cobot).
- Kiến trúc mạng nơ-ron: Actor (quyết định bước đi) + 2 Critics (đánh giá quỹ đạo).

**SLIDE 12 — CƠ CHẾ HYBRID GRIPPER & PHYSICS CLAMP:**
Để AI có thể hoạt động như một Cobot thực thụ, nhóm đã đưa ra 2 đột phá cơ điện tử:
1. Cơ chế Hybrid Gripper: AI không được phép bật/tắt giác hút (tránh lỗi nhả rác bừa bãi). Thay vào đó, cảm biến tiệm cận sẽ tự hút khi cách vật <4.5cm, và tự nhả khi đến trên nắp thùng rác. AI chỉ làm nhiệm vụ **Điều hướng tay máy**.
2. Physics-level Euler Clamp: Để đảm bảo tay máy không xoắn lộn xộn, gốc tọa độ bị kẹp cơ học ±15°. Trục Roll-Pitch luôn bị ép thẳng 90° chúc xuống mặt bàn.

**SLIDE 13 — OBSERVATION & ACTION SPACE:**
Observation (20D):
| Index | Ý nghĩa |
|---|---|
| 0-2 | Vị trí EE (xyz) |
| 3-5 | Vị trí vật (xyz) |
| 6-8 | Vector EE→Vật |
| 9-11 | Vector Vật→Bin |
| 12-15 | Quaternion hướng vật |
| 16 | Trạng thái gripper |
| 17-19 | EE euler (roll, pitch, yaw) |

Action (7D):
| 0-2 | Δxyz (±5cm/step) |
| 3-5 | ΔRoll/Pitch/Yaw (±4.5°/step) |
| 6 | Không sử dụng (Hybrid Gripper tự xử lý) |

**SLIDE 14 — THIẾT KẾ HÀM REWARD (Slide quan trọng nhất!):**
Phase-Based Reward — 3 giai đoạn + time penalty −0.05/step:

PHA 0 — APPROACH & GRASP:
- Dense reward gần vật: max(0, 2.0 − dist × 8.0)
- Gắp thành công: +50 → Pha 1

PHA 1 — CARRY:
- Nâng ≥ 20cm (one-time): +30
- Phạt bay thấp (EE < 0.60m): −3.0
- Dense reward gần bin: max(0, 2.0 − dist_xy × 5.0)
- Gần bin XY < 15cm → Pha 2

PHA 2 — PLACE:
- Dense reward hạ xuống: max(0, 3.0 − dist_3d × 10.0)
- VÀO BIN: +500 → DONE ✓

Nhờ cơ chế Hybrid Gripper tự hút/nhả và Physics Clamp kẹp thẳng đứng, hàm Reward được tối giản cực kỳ gọn nhẹ, AI không thể Reward Hacking.

**SLIDE 15 — QUÁ TRÌNH TRAINING:**
Train từ đầu (from scratch) — 1 lần duy nhất, KHÔNG dùng Curriculum Learning:
- Script: `train_17d_place.py`
- 10M steps, 16 envs song song, ~3 tiếng
- Hội tụ tốt nhờ 3 đột phá: Hybrid Gripper + Phase-Based Reward + Physics Clamp
- File `train_17d_grasp.py` tồn tại như bản thiết kế Curriculum nhưng không cần sử dụng
- Sơ đồ: Block diagram với 4 yếu tố then chốt → 100% success

**SLIDE 16 — KẾT QUẢ TRAINING:**
Bảng:
| Metric | Giá trị |
|---|---|
| Script | train_17d_place.py (from scratch) |
| Steps | 10,000,000 |
| Envs | 16 (SubprocVecEnv) |
| Thời gian | ~3 tiếng |
| Success rate | 100% |
| FPS | ~600 |
| Hardware | Core i7, 16GB RAM |
| Output | best_model.zip + vecnormalize.pkl |

**SLIDE 17 — SO SÁNH AUTO vs AI:**
Bảng:
| Tiêu chí | Auto (FSM) | AI (SAC) |
|---|---|---|
| Thành công | 100% | 100% |
| Lập trình quỹ đạo | Có (cứng nhắc) | Không (tự học) |
| Thích nghi | Không | Có |
| Tư thế | Thẳng 90° | Thẳng 90° (Physics Clamp) |
2 hình so sánh: Auto đường vuông góc vs AI đường cong mượt.

**SLIDE 18 — DEMO TRỰC TIẾP:**
- Chạy `python -m hmi.app`
- Demo 3 chế độ: Manual → Auto → AI
- Nhấn mạnh: AI tự tìm đường, không lập trình trước

**SLIDE 19 — HẠN CHẾ & HƯỚNG PHÁT TRIỂN:**
Hạn chế:
- AI yếu ngoài vùng train (Out-of-Distribution)
- Tọa độ vật từ PyBullet (chưa dùng camera thật)
- Chưa Sim-to-Real

Hướng phát triển:
1. Domain Randomization
2. Camera RealSense + Point Cloud
3. ur_rtde → robot UR5e thật
4. Multi-Object sorting

**SLIDE 20 — CẢM ƠN:**
Cảm ơn GVHD, hội đồng. Q&A.

## ═══ KẾT THÚC PROMPT ═══
