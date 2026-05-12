# KỊCH BẢN SLIDE THUYẾT TRÌNH ĐỒ ÁN MÔN HỌC
# Robot UR5e — Pick & Place với Học Tăng Cường (SAC)
# Tổng cộng: ~20 slides

---

## SLIDE 1 — TRANG BÌA
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
- Tên đề tài: "Hệ thống Điều khiển & Mô phỏng Robot UR5e — Ứng dụng Học Tăng Cường cho bài toán Pick & Place"
- Họ tên sinh viên
- GVHD
- Khoa / Trường
- Năm 2026

**Hình ảnh:** Ảnh chụp giao diện HMI + hình 3D robot UR5e trong PyBullet

---

## SLIDE 2 — NỘI DUNG TRÌNH BÀY
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
1. Đặt vấn đề & Mục tiêu
2. Cơ sở lý thuyết (FK, IK, RL)
3. Thiết kế hệ thống
4. Chế độ Auto (FSM)
5. Chế độ AI (SAC RL)
6. Kết quả & Demo
7. Kết luận & Hướng phát triển

---

## SLIDE 3 — ĐẶT VẤN ĐỀ
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
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
**Nội dung hiển thị trên Slide (TỐI GIẢN):** Sơ đồ block diagram

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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
- Thiết lập hệ trục tọa độ XYZ tại từng khớp theo đúng quy tắc Standard Denavit-Hartenberg (DH Chuẩn).
- Trích xuất bảng tham số cấu trúc (a, d, α) dựa trên tài liệu chuẩn (Hawkins 2013, Andersen 2018) và đối chiếu khớp 100% với file thiết kế `ur5e.urdf` của hãng.
- **Lưu ý hội đồng:** Các thông số a4, a5, a6 = 0 vì cụm cổ tay của UR5e là Spherical Wrist (các trục cắt nhau tại 1 điểm, không có khoảng cách vuông góc chung).

**Hình ảnh đề xuất:** 
- (Trái) Bảng DH 6 khớp với các thông số a, d, alpha, theta.
- (Phải) Hình minh họa 3D robot UR5e với các vector trục XYZ màu Đỏ-Xanh lá-Xanh dương gắn tại từng khớp tương ứng.

---

## SLIDE 8 — ĐỘNG HỌC THUẬN (FK) & KIỂM CHỨNG
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
- **Giải thuật:** Nhân liên tiếp 6 ma trận biến đổi thuần nhất 4x4.
- Công thức: T_EE = T1 x T2 x T3 x T4 x T5 x T6
- **Code Python:** Viết hàm `forward_kinematics(q)` sử dụng thư viện `NumPy` để tính toán ma trận T cực nhanh.
- **Kiểm chứng độc lập (Verify):** 
  - So sánh kết quả tính tay (NumPy) với vị trí thực tế trên Engine Vật lý PyBullet.
  - Kết quả: Sai số Euclidean ~ 0 (Hoàn toàn trùng khớp!).

**Hình ảnh đề xuất:**
- Một ảnh ghép gồm: (1) Đoạn code nhân ma trận `T = T @ Ti`, và (2) Ảnh chụp Terminal Terminal Python báo "UNIT TEST PASS" cùng với khung hình PyBullet tương ứng.

---

## SLIDE 9 — ĐỘNG HỌC NGHỊCH (IK) & KIỂM CHỨNG
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
- **FSM (Finite State Machine):** Gồm 11 state tuần tự khép kín.
- **Quỹ đạo:** Mỗi state thực thi một quỹ đạo định trước.
- **Fail-safe:** Timeout 15s/state + Jam Detector (Chống kẹt motor).

**Hình ảnh:** Sơ đồ FSM:
```
IDLE → DETECT → APPROACH → DESCEND → PICK
                                       │
DONE ← RETREAT ← RELEASE ← PLACE ← LIFT ← MOVE_TO_BIN
```

---

## SLIDE 12 — TẠI SAO CẦN AI (RL)?
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
- **Hạn chế Auto:** Quỹ đạo cứng nhắc. Dễ gắp hụt nếu vật thể xê dịch.
- **Giải pháp:** Học tăng cường (Soft Actor-Critic - SAC).
- **Ưu điểm AI:** Thích nghi thời gian thực (Real-time Adaptation). Tự động bẻ lái đuổi theo vật thể mượt mà.

**Hình ảnh:** Sơ đồ vòng lặp Agent (SAC) ↔ Environment:
```
       │                                         │
   [Actor SAC]                              [PyBullet]
       │                                         │
       └◄────────── Reward & 20D State ──────────┘
```

---

## SLIDE 13 — CƠ CHẾ HYBRID VACUUM & PHYSICS CLAMP
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**

**Observation Space (20D):**
- Tọa độ EE & Vật thể (6D).
- Vector khoảng cách tương đối (6D).
- Hướng vật (Quaternion - 4D).
- Góc nghiêng EE (3D) + Trạng thái hút (1D).

**Action Space (7D):**
- Vận tốc tịnh tiến XYZ (3D).
- Vận tốc xoay Roll-Pitch-Yaw (3D).
- *Hành động số 7 (Bơm hút) vô hiệu hóa.*

---

## SLIDE 15 — THIẾT KẾ HÀM REWARD (Phần quan trọng nhất!)
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**

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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**

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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
- Chạy hệ thống thực tế trên phần mềm HMI nhóm tự phát triển: `python -m hmi.app`
- Trình diễn chế độ Manual (Dùng thanh trượt điều khiển IK/FK).
- Trình diễn chế độ Auto (FSM mượt mà).
- Trình diễn chế độ AI (Kéo vật thể đi chỗ khác, xem AI tự tìm đường đến gắp).
- Trình diễn hệ thống Fail-safe: Cố tình kéo robot ra ngoài không gian làm việc → Robot kích hoạt Auto-Home ngay lập tức.

**Hành động:** Chuyển sang màn hình PyBullet + PyQt5 để demo live!

---

## SLIDE 20 — HẠN CHẾ & HƯỚNG PHÁT TRIỂN
**Nội dung hiển thị trên Slide (TỐI GIẢN):**

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
**Nội dung hiển thị trên Slide (TỐI GIẢN):**
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
