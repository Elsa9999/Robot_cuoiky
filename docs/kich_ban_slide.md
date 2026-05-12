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
4. ✅ Train AI bằng SAC (Huấn luyện 1 phase từ đầu)
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

## SLIDE 7 — ĐẶT TRỤC VÀ BẢNG DH
**Nội dung:**
- Thiết lập hệ trục tọa độ XYZ tại từng khớp theo đúng quy tắc Standard Denavit-Hartenberg (DH Chuẩn).
- Trích xuất bảng tham số cấu trúc (a, d, α) dựa trên tài liệu chuẩn (Hawkins 2013, Andersen 2018) và đối chiếu khớp 100% với file thiết kế `ur5e.urdf` của hãng.
- **Lưu ý hội đồng:** Các thông số a4, a5, a6 = 0 vì cụm cổ tay của UR5e là Spherical Wrist (các trục cắt nhau tại 1 điểm, không có khoảng cách vuông góc chung).

**Hình ảnh đề xuất:** 
- (Trái) Bảng DH 6 khớp với các thông số a, d, alpha, theta.
- (Phải) Hình minh họa 3D robot UR5e với các vector trục XYZ màu Đỏ-Xanh lá-Xanh dương gắn tại từng khớp tương ứng.

---

## SLIDE 8 — ĐỘNG HỌC THUẬN (FK) & KIỂM CHỨNG
**Nội dung:**
- **Giải thuật:** Nhân liên tiếp 6 ma trận biến đổi thuần nhất 4x4.
- Công thức: T_EE = T1 x T2 x T3 x T4 x T5 x T6
- **Code Python:** Viết hàm `forward_kinematics(q)` sử dụng thư viện `NumPy` để tính toán ma trận T cực nhanh.
- **Kiểm chứng độc lập (Verify):** 
  - So sánh kết quả tính tay (NumPy) với Engine Vật lý PyBullet.
  - So sánh đối chiếu chéo với phần mềm Matlab (Robotics System Toolbox).
  - Kết quả: Sai số Euclidean ~ 0 (Hoàn toàn trùng khớp!).

**Hình ảnh đề xuất:**
- Một ảnh ghép gồm: (1) Đoạn code nhân ma trận `T = T @ Ti`, (2) Ảnh chụp Terminal Terminal Python báo "UNIT TEST PASS", và (3) Ảnh chụp Command Window của Matlab in ra cùng 1 kết quả ma trận T.

---

## SLIDE 9 — ĐỘNG HỌC NGHỊCH (IK) & KIỂM CHỨNG
**Nội dung:**
- Bài toán: biết vị trí (x,y,z) và hướng cần vươn tới → tìm 6 góc quay [q1...q6]
- **Giải pháp:** Xây dựng Hybrid Solver 2 lớp cực kỳ tối ưu:
  - **Lớp 1:** Analytical (giải tích) → tốc độ < 1ms, đưa ra 8 nghiệm. Tự động chọn nghiệm ít phải xoay khớp nhất.
  - **Lớp 2:** Numerical (phương pháp số L-BFGS-B từ SciPy) → tốc độ < 10ms. Kích hoạt dự phòng khi giải tích thất bại.
- **Kiểm chứng Round-Trip (Vòng lặp kín):**
  - Chạy code tính xuôi: Góc Q gốc → FK → Tọa độ XYZ.
  - Chạy code tính ngược: Tọa độ XYZ → IK → Giải ra góc Q'.
  - Tính lại lần cuối: Góc Q' → FK → Tọa độ XYZ'.
  - Kết quả: Sai số giữa XYZ gốc và XYZ' < 1mm (0.001m) → Chứng minh FK và IK đều đúng toán học 100%!

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

## SLIDE 10 — QUY HOẠCH QUỸ ĐẠO
**Nội dung:**
- **Joint Space (Không gian khớp):** Nội suy mượt mà các góc khớp để robot không bị giật. Sử dụng Trapezoidal Velocity Profile (Đồ thị vận tốc hình thang).
- **Cartesian Space (Không gian làm việc):** Nội suy đường thẳng XYZ, gọi IK liên tục tại mỗi điểm trung gian để ghép thành quỹ đạo mượt (Joint Traj).
- Đảm bảo giới hạn vận tốc tối đa (v_max) và gia tốc tối đa (a_max).

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

## SLIDE 11 — CHẾ ĐỘ AUTO (FSM)
**Nội dung:**
- Máy trạng thái hữu hạn (Finite State Machine) gồm 11 bước tuần tự khép kín.
- Mỗi state thực thi một quỹ đạo, hoàn thành sẽ tự động chuyển sang state tiếp theo.
- **Tính an toàn:** Tích hợp Timeout 15s/state. Nếu robot bị kẹt (Jam Detector) hoặc chờ quá lâu → Tự động báo ERROR, không để motor gồng quá tải.

**Hình ảnh:** Sơ đồ FSM:
```
IDLE → DETECT → APPROACH → DESCEND → PICK
                                       │
DONE ← RETREAT ← RELEASE ← PLACE ← LIFT ← MOVE_TO_BIN
```

---

## SLIDE 12 — TẠI SAO CẦN AI (RL)?
**Nội dung:**
- **Hạn chế của Auto:** Chế độ Auto chạy theo quỹ đạo cố định (đường đi gấp khúc cứng nhắc). Nếu vật thể vô tình bị xê dịch trong lúc tay máy đang di chuyển, tay máy sẽ đi mù quáng đến vị trí cũ và gắp hụt.
- **Giải pháp AI:** Nhóm chọn thuật toán Reinforcement Learning (Soft Actor-Critic - SAC) để thay thế khối FSM.
- **Ưu điểm SAC:** Khả năng tự thích nghi theo thời gian thực (Real-time Adaptation). Nếu vật thể di chuyển, AI lập tức cập nhật "tầm nhìn" (Observation) và bẻ lái tự đuổi theo vật thể bằng một quỹ đạo cong mượt mà.

**Hình ảnh:** Sơ đồ vòng lặp Agent (SAC) ↔ Environment:
```
       ┌──── Continuous Action (ΔXYZ, ΔRPY) ────►┐
       │                                         │
   [Actor SAC]                              [PyBullet]
       │                                         │
       └◄────────── Reward & 20D State ──────────┘
```

---

## SLIDE 13 — CƠ CHẾ HYBRID VACUUM & PHYSICS CLAMP
**Nội dung:**
- Đặt vấn đề: Huấn luyện AI lái 6 bậc tự do (6-DoF) Pick & Place từ số 0 là vô cùng khó, AI hay bị "ngáo".
- Để giải quyết, nhóm áp dụng **2 đột phá cơ điện tử**:
- **Đột phá 1 (Hybrid Vacuum):** AI KHÔNG được quyền điều khiển bật/tắt giác hút chân không. Việc này giao cho cảm biến tiệm cận (cách <4.5cm thì hút). AI chỉ tập trung 100% vào việc "lái".
- **Đột phá 2 (Physics-level Euler Clamp):** Gốc tọa độ cổ tay bị kẹp chặt bằng thuật toán vật lý. Góc nghiêng (Roll-Pitch) chỉ được dao động ±15°, ép cánh tay luôn ở tư thế chúc thẳng xuống mặt bàn chuẩn xác như Auto.

**Hình ảnh:**
```
[Physics Clamp] ──► Giữ thẳng tay (cấm xoay loạn xạ)
[Hybrid Vacuum] ──► Cảm biến tự động hút/nhả
       │
      Tạo môi trường an toàn để AI học cực nhanh!
```

---

## SLIDE 14 — OBSERVATION & ACTION SPACE
**Nội dung:**

**Observation Space (20 차원 - 20D):**
- Mắt AI nhìn thấy 20 thông số mỗi bước:
- Vị trí EE, Vị trí vật (6D)
- Vector khoảng cách EE→Vật và Vật→Bin (6D)
- Hướng vật (Quaternion 4D) + Trạng thái hút (1D)
- Góc nghiêng EE (Euler 3D)

**Action Space (7 차원 - 7D):**
- Tín hiệu điều khiển AI xuất ra:
- Delta XYZ (±5cm/step): Vận tốc tịnh tiến
- Delta Roll/Pitch/Yaw (±4.5°/step): Vận tốc xoay
- *Hành động thứ 7 (Bơm chân không) bị vô hiệu hóa bởi Hybrid Vacuum.*

---

## SLIDE 15 — THIẾT KẾ HÀM REWARD (Phần quan trọng nhất!)
**Nội dung:**
- Vấn đề: "Reward Hacking" — AI lợi dụng kẽ hở hàm thưởng để ăn điểm lặp đi lặp lại.
- Giải pháp: **Phase-Based Reward** (Thưởng theo 3 Giai đoạn).

**Hình ảnh:** Bảng 3 pha:
```
┌─────────────────────────────────┐
│  PHA 0 — APPROACH & GRASP      │
│  Thưởng gần vật: +2.0          │
│  ★ GẮP ĐƯỢC: +50 → Chuyển Pha 1│
├─────────────────────────────────┤
│  PHA 1 — CARRY                  │
│  Nâng lên > 20cm: +30          │
│  Phạt bay thấp: −3 (tránh bàn) │
│  Bay tới bin: +2.0 → Chuyển Pha2│
├─────────────────────────────────┤
│  PHA 2 — PLACE                  │
│  Hạ vào bin: +3.0              │
│  ★ THẢ VÀO BIN: +500 → DONE ✓  │
└─────────────────────────────────┘
```
- **Lưu ý:** Nhờ kết hợp Hybrid Vacuum + Physics Clamp, hàm Reward được đơn giản hóa đi rất nhiều, triệt tiêu hoàn toàn khả năng Reward Hacking. AI hiểu rõ mục tiêu từng phase.

---

## SLIDE 16 — QUÁ TRÌNH TRAINING (TRAIN TỪ SCRATCH)
**Nội dung:**
- Điểm khác biệt: **Train thẳng từ đầu (from scratch) trong 1 Phase duy nhất!**
- Không sử dụng phương pháp Curriculum Learning (Chia nhỏ môi trường) phức tạp và tốn thời gian.
- Tại sao làm được? Nhờ 3 đột phá đã trình bày: `Hybrid Vacuum` + `Physics Clamp` + `Phase-Based Reward` → Môi trường đã được cách ly rủi ro hoàn hảo.

**Hình ảnh:**
```
┌─────────────────────────────────────────────┐
│  Script: train_17d_place.py                 │
│                                             │
│  10M steps │ 16 envs (song song) │ ~600 FPS │
│                                             │
│  ✓ Mô phỏng tăng tốc 16 lần                 │
│  ✓ VecNormalize: Chuẩn hóa Observation      │
│  ✓ Entropy-Regularized SAC                  │
└─────────────────────────────────────────────┘
```

---

## SLIDE 17 — KẾT QUẢ TRAINING
**Nội dung:**

| Metric | Kết Quả Đạt Được |
|---|---|
| Số bước huấn luyện | 10,000,000 steps |
| Thời gian huấn luyện | ~3 tiếng (CPU Core i7) |
| Lỗi hội tụ | Giảm dần và ổn định sau 6M steps |
| **Success rate** | **100% (Tuyệt đối sau khi hội tụ)** |
| Sản phẩm đầu ra | `best_model.zip` & `vecnormalize.pkl` |

**Hình ảnh:** Screenshot đồ thị TensorBoard (Reward tăng dần, Episode Length hội tụ về ngưỡng lý tưởng).

---

## SLIDE 18 — SO SÁNH AUTO vs AI
**Nội dung:**

| Tiêu chí | Chế độ Auto (FSM) | Chế độ AI (SAC) |
|---|---|---|
| Tỉ lệ gắp thả | 100% | 100% |
| Quỹ đạo | Góc cạnh vuông vức, cố định | Đường cong parabol tự tối ưu |
| Thích nghi | Không thể (Lỗi nếu vật rơi) | Tự động đổi hướng đuổi theo vật |
| Lập trình | Code cực dài, phức tạp | Code ngắn, để AI tự học quy luật |

**Hình ảnh:**
- Sơ đồ 2 quỹ đạo: Một đường nét đứt vuông góc (Auto) và một đường nét liền hình parabol (AI).

---

## SLIDE 19 — DEMO TRỰC TIẾP
**Nội dung:**
- Chạy hệ thống thực tế trên phần mềm HMI nhóm tự phát triển: `python -m hmi.app`
- Trình diễn chế độ Manual (Dùng thanh trượt điều khiển IK/FK).
- Trình diễn chế độ Auto (FSM mượt mà).
- Trình diễn chế độ AI (Kéo vật thể đi chỗ khác, xem AI tự tìm đường đến gắp).
- Trình diễn hệ thống Fail-safe: Cố tình kéo robot ra ngoài không gian làm việc → Robot kích hoạt Auto-Home ngay lập tức.

**Hành động:** Chuyển sang màn hình PyBullet + PyQt5 để demo live!

---

## SLIDE 20 — HẠN CHẾ & HƯỚNG PHÁT TRIỂN
**Nội dung:**

**Hạn chế:**
- AI bị "ngáo" nếu vật bị đặt ngoài vùng không gian huấn luyện (Out-of-Distribution).
- Đang dùng tọa độ tuyệt đối từ môi trường mô phỏng (omniscient), chưa dùng camera (Vision).
- Chưa thể hiện Sim-to-Real trên tay máy UR5e thật.

**Hướng phát triển:**
1. Áp dụng Domain Randomization mở rộng vùng học tập.
2. Tích hợp Camera RealSense D435 xử lý Point Cloud.
3. Sử dụng thư viện `ur_rtde` đẩy tín hiệu điều khiển thẳng từ AI xuống tủ điện robot thật.
4. Gắp thả đa vật thể phân loại theo màu sắc/hình dáng.

---

## SLIDE 21 — CẢM ƠN
**Nội dung:**
- Cảm ơn Thầy/Cô hướng dẫn đã hỗ trợ sát sao.
- Cảm ơn Hội đồng đã lắng nghe.
- Q&A (Mời Hội đồng đặt câu hỏi).

---

# GHI CHÚ DÀNH CHO THIẾT KẾ SLIDE:
1. Tổng số: 21 slides.
2. Tông màu đề xuất: Nền tối (dark/navy theme) + chữ trắng/xám sáng + accent xanh dương/cam.
3. Font chữ: Roboto, Inter hoặc Montserrat (hiện đại, to rõ, dễ nhìn qua máy chiếu).
4. Các sơ đồ code (ASCII) ở trên nên được vẽ lại bằng hình học (Shapes) trong PowerPoint/Canva.
5. Tuyệt đối nhấn mạnh Slide 13, 14, 15 (Hybrid Vacuum, Reward, Train từ Scratch) vì đây là "bài tẩy" của nhóm!
6. Chuẩn bị sẵn file `best_model.zip` mở để chạy live (Slide 19).
