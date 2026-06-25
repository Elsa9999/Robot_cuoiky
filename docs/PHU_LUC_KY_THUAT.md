# PHỤ LỤC KỸ THUẬT

Tài liệu này cung cấp các thông tin kỹ thuật chi tiết của dự án xây dựng môi trường học tăng cường SAC cho robot UR5e thực hiện gắp thả vật thể trong PyBullet. Các nội dung dưới đây nhằm phục vụ mục đích kiểm chứng, tái lập kết quả và phát triển phần mềm mà không làm tăng số trang của bài báo chính.

## 1. Mục đích phụ lục

Phụ lục kỹ thuật này lưu trữ cấu trúc thư mục, đặc tả thiết kế không gian trạng thái, không gian hành động, cấu hình thuật toán huấn luyện và hướng dẫn chi tiết các bước vận hành hệ thống. Điều này đảm bảo tính minh bạch khoa học và khả năng tái lập của nghiên cứu.

## 2. Cấu trúc các file chính của project

Project được tổ chức thành các thư mục chức năng cụ thể như sau:

* `train_17d_place.py`: Kịch bản huấn luyện chính sử dụng thuật toán SAC kết hợp song song hóa môi trường và chuẩn hóa VecNormalize.
* `run_demo.py`: Kịch bản tải chính sách đã huấn luyện và chạy thử nghiệm suy diễn trực quan trong môi trường PyBullet GUI.
* `simulation/environment.py`: Định nghĩa lớp môi trường mô phỏng kế thừa từ Gymnasium, tích hợp bộ giải động học và logic tay gắp lai.
* `kinematics/forward_kinematics.py`: Thuật toán động học thuận giải tích sử dụng bảng thông số Denavit Hartenberg của UR5e.
* `kinematics/inverse_kinematics.py`: Bộ giải động học nghịch kết hợp phương pháp giải tích tách tâm cổ tay và tối ưu hóa số học L-BFGS-B dự phòng.
* `hmi/sim_bridge.py`: Cầu nối truyền thông và điều khiển phục vụ giao diện người máy HMI.
* `tests/`: Thư mục chứa các kịch bản kiểm thử tự động bằng thư viện pytest.
* `docs/paper/`: Mã nguồn LaTeX của bài báo khoa học cùng các hình ảnh kết quả đi kèm.

## 3. Đặc tả cấu hình hệ thống học tăng cường

### Không gian quan sát (Observation Space)
Vector quan sát gồm 20 chiều liên tục biểu diễn trạng thái của hệ thống:
* 3 chiều đầu: Tọa độ Cartesian hiện tại của đầu công tác.
* 3 chiều tiếp theo: Tọa độ Cartesian hiện tại của vật thể cần gắp.
* 3 chiều tiếp theo: Vector vị trí tương đối từ đầu công tác tới vật thể.
* 3 chiều tiếp theo: Vector vị trí tương đối từ vật thể tới thùng chứa.
* 4 chiều tiếp theo: Quaternion biểu diễn hướng xoay của vật thể.
* 1 chiều tiếp theo: Trạng thái đóng mở của tay gắp chân không.
* 3 chiều cuối: Các góc Euler (Roll, Pitch, Yaw) của đầu công tác.

### Không gian hành động (Action Space)
Vector hành động gồm 7 chiều liên tục trong miền [-1.0, 1.0]:
* $[\Delta x, \Delta y, \Delta z]$: Lệnh dịch chuyển tịnh tiến vi phân (giới hạn tối đa 0.05 m).
* $[\Delta \phi_r, \Delta \phi_p, \Delta \phi_y]$: Lệnh xoay góc khớp cổ tay vi phân (giới hạn tối đa 0.08 rad).
* Trạng thái đóng mở tay gắp: Được định cấu hình tự động thông qua cơ chế tay gắp lai.

### Thuật toán Soft Actor Critic (SAC)
* Mạng neural: Actor và Critic sử dụng kiến trúc MLP song song với kích thước [256, 256], hàm kích hoạt ReLU.
* Tần số cập nhật mạng: 1 lần sau mỗi bước môi trường.
* Learning rate: $3 \cdot 10^{-4}$ sử dụng bộ tối ưu hóa Adam.
* Hệ số chiết khấu $\gamma$: 0.99.
* Tham số cập nhật mạng mục tiêu $\tau$: 0.005.
* Kích thước lô mẫu (Batch size): 256.
* Bộ nhớ đệm (Replay buffer): 1,000,000 bước.
* Hệ số entropy $\alpha$: Tự động điều chỉnh theo entropy mục tiêu.

### Chuẩn hóa VecNormalize
Cả trạng thái quan sát và điểm thưởng đều được chuẩn hóa động về phân phối chuẩn thông qua thống kê trung bình và độ lệch chuẩn lũy kế. Các giá trị quan sát sau đó được cắt lọc trong miền [-10.0, 10.0].

### Cơ chế tay gắp lai (Hybrid Gripper)
* Tự động tạo liên kết cứng (fixed constraint) kẹp giữ vật thể khi khoảng cách từ đầu công tác đến vật dưới 0.045 m.
* Tự động xóa bỏ liên kết để thả rơi vật thể khi khoảng cách từ đầu công tác đến thùng chứa dưới 0.05 m.

## 4. Hướng dẫn chạy demo và kiểm thử

### Cài đặt môi trường
Trước khi vận hành, cần cài đặt các thư viện phụ thuộc bằng pip:
```bash
pip install pybullet gymnasium stable-baselines3 torch pytest
```

### Chạy suy diễn Demo trực quan
Để chạy demo hiển thị cánh tay robot gắp thả vật thể nằm ngang trong PyBullet GUI:
```bash
python run_demo.py
```

### Chạy bộ kiểm thử tự động
Hệ thống tích hợp các bài kiểm thử để bảo đảm tính ổn định toán học và vật lý:
```bash
pytest tests/test_smoke.py
pytest tests/test_trajectory.py
pytest tests/test_dynamics.py
pytest tests/test_pick_place.py
```

## 5. Cách sinh lại các biểu đồ kết quả

### Vẽ đường cong học tập (eval_curves.png)
Đường cong học tập được vẽ dựa trên tệp kết quả lưu trong quá trình huấn luyện:
1. Chạy script vẽ đồ thị bằng lệnh:
   ```bash
   python scratch/make_plots.py
   ```
2. Script sẽ đọc dữ liệu từ đường dẫn `logs_rl_17d/seed42/evaluations.npz`, trích xuất các mảng tỷ lệ thành công, phần thưởng và số bước thực hiện để lưu thành tệp hình ảnh tại `docs/paper/figures/eval_curves.png`.

### Tạo chuỗi hoạt động (demo_sequence.png)
Biểu đồ chuỗi hoạt động được ghép từ bốn hình ảnh chụp lại màn hình PyBullet GUI tại các thời điểm tương ứng với đóng vai trò biểu diễn trực quan bốn pha hoạt động chính bao gồm khởi tạo, tiếp cận gắp vật, vận chuyển trên cao và thả vật vào thùng chứa.

## 6. Giới hạn hiện tại của môi trường mô phỏng

* Môi trường mô phỏng PyBullet chỉ được thiết lập ở cấu hình phân phối vật thể ngẫu nhiên với độ khó trung bình. Các trường hợp có chướng ngại vật di động chưa được tích hợp.
* Thông tin vị trí vật thể được đọc trực tiếp từ dữ liệu mô phỏng. Khi triển khai thực tế, cần ước lượng thông tin này thông qua camera và các giải pháp thị giác máy tính.
* Chính sách điều khiển chưa được kiểm chứng trên thiết bị thật, đòi hỏi các nghiên cứu sâu hơn về chuyển giao sim-to-real nhằm thu hẹp sai lệch mô hình.
