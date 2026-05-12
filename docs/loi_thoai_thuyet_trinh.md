# KỊCH BẢN LỜI NÓI (SPEAKER NOTES) - BẢO VỆ ĐỒ ÁN
*Tài liệu này chứa từng câu chữ chi tiết bạn sẽ cầm đọc/học thuộc khi đứng thuyết trình.*

---

**[SLIDE 1 - TIÊU ĐỀ] (Mở đầu điềm tĩnh, rõ ràng)**
"Kính thưa quý thầy cô, em tên là [Tên của bạn]. Hôm nay, em rất vinh dự được trình bày đề tài đồ án môn học của mình: 'Hệ thống Điều khiển & Mô phỏng Robot UR5e — Ứng dụng Học Tăng Cường cho bài toán Pick & Place'. Mong quý thầy cô lắng nghe và góp ý."

**[SLIDE 2 - ĐẶT VẤN ĐỀ]**
"Để bắt đầu, chúng ta hãy nhìn vào thực tế sản xuất công nghiệp. Thao tác Pick & Place — tức là gắp và đặt vật thể — là một trong những tác vụ phổ biến nhất. Tuy nhiên, nó luôn đặt ra thách thức lớn: vật thể thường nằm ở các vị trí ngẫu nhiên, và việc lên lịch trình quỹ đạo cho cánh tay 6 bậc tự do né tránh va chạm đòi hỏi những thuật toán rất phức tạp."

**[SLIDE 3 - MỤC TIÊU ĐỒ ÁN]**
"Chính vì lý do đó, mục tiêu của đồ án em là xây dựng một trạm làm việc mô phỏng cho robot UR5e. Điểm nhấn của hệ thống là việc tích hợp cả 2 phương pháp điều khiển: Điều khiển lập trình cứng thông thường (Auto FSM) và phương pháp tự học thông minh thông qua thuật toán Học Tăng Cường - Reinforcement Learning. Mục tiêu lớn nhất là chứng minh hệ thống AI có khả năng hoạt động tốt ngang ngửa hoặc vượt trội so với lập trình cứng."

**[SLIDE 4 - PHẠM VI & MÔI TRƯỜNG]**
"Về phạm vi, em sử dụng bộ mô phỏng vật lý PyBullet chạy ở tốc độ 240Hz để đảm bảo tính chân thực. Đối tượng tác động là robot kéo và các vật thể hình trụ 100g, sử dụng một đầu hút chân không (Vacuum Suction Cup) được mô phỏng động lực học để gắp vật."

**[SLIDE 5 - CÔNG NGHỆ ÁP DỤNG]**
"Hệ thống của em được phát triển hoàn toàn bằng Python, với trí tuệ nhân tạo được xây dựng trên framework Stable-Baselines3 sử dụng PyTorch. Để người dùng cuối dễ thao tác, em cũng đã đóng gói một giao diện HMI chuyên nghiệp bằng thư viện PyQt5."

**[SLIDE 6 - KIẾN TRÚC TỔNG THỂ]**
"Nhìn vào sơ đồ kiến trúc này, hệ thống được chia thành 4 tầng rõ rệt hoạt động độc lập: Tầng Giao diện HMI trên cùng, Tầng Thuật toán xử lý Động học, Tầng Thuật toán AI, và Tầng dưới cùng là phần lõi mô phỏng trực tiếp PyBullet. Các dữ liệu được xử lý phi song song tránh nghẽn mạch."

**[SLIDE 7 - ĐẶT TRỤC VÀ BẢNG DH] (Chỉ tay vào bảng và hình)**
"Về mặt toán học, nhóm thiết lập hệ trục tọa độ tại từng khớp theo đúng quy tắc Standard Denavit-Hartenberg (DH Chuẩn). Bảng tham số (a, d, alpha) được nhóm tham khảo từ các tài liệu chuẩn quốc tế và đối chiếu khớp 100% với file thiết kế cơ khí (URDF) từ chính hãng Universal Robots."

**[SLIDE 8 - ĐỘNG HỌC THUẬN FK & KIỂM CHỨNG]**
"Từ bảng DH, em xây dựng hàm Động học thuận bằng cách nhân liên tiếp 6 ma trận biến đổi 4x4. Để hội đồng yên tâm về tính chính xác, nhóm đã kiểm chứng độc lập kết quả tính tay bằng NumPy so với phần mềm Matlab (Robotics System Toolbox) và engine PyBullet. Sai số hoàn toàn bằng 0."

**[SLIDE 9 - ĐỘNG HỌC NGHỊCH IK & KIỂM CHỨNG] (Nhấn mạnh sự kỹ tính của kỹ sư)**
"Đối với bài toán Động học Nghịch, em đã lập trình một bộ giải Hybrid 2 lớp: Đầu tiên giải bằng Phương pháp Giải tích để lấy tốc độ dưới 1 mili-giây. Nếu gặp điểm mù (singularity), thuật toán Số học L-BFGS-B sẽ tự kích hoạt dự phòng. Đặc biệt, nhóm đã tự kiểm chứng bằng phương pháp Round-Trip vòng lặp kín: nạp góc vào FK ra tọa độ, nạp tọa độ đó vào IK để giải ngược lại ra góc mới. Sai số tính toán chưa tới 1 mm, khẳng định 2 hàm Động học đúng tuyệt đối 100%!"

**[SLIDE 10 - QUY HOẠCH QUỸ ĐẠO]**
"Để robot không bị giật cứng khi di chuyển, em triển khai nội suy quỹ đạo Joint Space theo hình thang (Trapezoidal profile) để tăng/giảm tốc mượt mà. Đặc biệt, nội suy Cartesian Linearity được dùng ở giai đoạn cắm đầu gắp xuống vật để đảm bảo tay máy chuyển động theo một đường tịnh tiến thẳng tuyệt đối."

**[SLIDE 11 - CHẾ ĐỘ AUTO FSM]**
"Đây là chế độ điều khiển theo lập trình truyền thống. Hệ thống vận hành như một cỗ máy trạng thái (FSM) gồm 11 bước tuần tự: từ APPROACH, DESCEND, đến PICK và PLACE. Nó hoạt động với độ thành công 100% kèm cơ chế bảo vệ Jam Detector chống cháy motor."

**[SLIDE 12 - TẠI SAO CẦN AI (RL)?]**
"Dù Auto FSM chạy mượt, nhưng nhược điểm lớn nhất là sự 'cứng nhắc'. Nếu trong lúc robot đang thò tay xuống gắp, ta vô tình xê dịch vật thể đi chỗ khác, Auto sẽ đi mù quáng đến vị trí cũ và gắp hụt. Đó là lý do nhóm sử dụng Học Tăng Cường (SAC). AI có khả năng thích nghi thời gian thực, liên tục 'nhìn' thấy vật thể và tự động bẻ lái quỹ đạo đuổi theo vật một cách linh hoạt nhất."

**[SLIDE 13 - CƠ CHẾ HYBRID VACUUM & PHYSICS CLAMP] (Nhấn mạnh tư duy Cơ điện tử)**
"Để AI không bay lượn hoang dại, nhóm áp dụng 2 đột phá cơ điện tử. Thứ nhất là Hybrid Vacuum: Cảm biến tiệm cận sẽ tự động kích hoạt van hút chân không khi tay máy cách vật < 4.5cm. AI hoàn toàn không cần học cách bật tắt bơm hút. Thứ hai là Physics Clamp: Nhóm kẹp cứng trục cổ tay không cho phép dao động quá ±15 độ. Dù AI có bay đường nào, tay máy vẫn luôn chúc thẳng đứng 90 độ y hệt chuẩn công nghiệp."

**[SLIDE 14 - OBSERVATION & ACTION]**
"Đầu vào của AI là vector 20 chiều dữ liệu trạng thái. Về đầu ra, Robot xuất hành động 7 chiều gồm dịch chuyển XYZ và xoay cổ tay Roll-Pitch-Yaw. Đặc biệt, chiều thứ 7 — lệnh bật tắt hút chân không — hoàn toàn KHÔNG được AI sử dụng do cơ chế Hybrid Vacuum tự động lo liệu, loại bỏ 100% rủi ro AI ăn gian điểm thưởng (Reward Hacking)."

**[SLIDE 15 - THIẾT KẾ HÀM REWARD] (Nhấn mạnh sự sáng tạo ở đây)**
"Nhờ 2 cơ chế Hybrid Vacuum và Clamp vật lý vừa trình bày ở Slide trước, việc thiết kế Hàm Phần Thưởng của nhóm trở nên cực kỳ đơn giản và kín kẽ. Hàm được chia làm 3 pha rõ rệt: Thưởng khi bay lại gần vật, Thưởng khi nhấc bổng vật lên, và Thưởng cực lớn khi mang thả vào đúng tọa độ thùng rác."

**[SLIDE 16 — QUÁ TRÌNH TRAINING]**
"Về quá trình huấn luyện, nhóm chỉ thực hiện một lần training duy nhất từ đầu — from scratch. Không cần dùng kỹ thuật Curriculum Learning phức tạp chia nhỏ giai đoạn vì môi trường đã được nhóm thiết kế quá tối ưu và an toàn tuyệt đối bởi 3 cơ chế: Hybrid Vacuum, Physics Clamp và Phase-Based Reward."

**[SLIDE 17 - KẾT QUẢ TRAINING]**
"Như đồ thị trên slide, em chạy 10 triệu steps với 16 môi trường song song đa luồng. Tốc độ mô phỏng đạt khoảng 600 FPS, và toàn bộ quá trình training mất khoảng 3 tiếng trên máy tính phổ thông Core i7. Kết quả: tỷ lệ thả vật thành công đạt 100 phần trăm tuyệt đối. Output cuối cùng là một mô hình Nơ-ron đã hội tụ hoàn toàn."

**[SLIDE 18 - SO SÁNH AUTO vs AI]**
"So sánh hai phương pháp: Chế độ Auto cần lập trình cứng từng bước quỹ đạo. Trong khi đó, AI chỉ cần định nghĩa mục tiêu (Reward) và để mạng Nơ-ron tự tìm đường, tạo ra các đường bay parabol mượt mà và đặc biệt là khả năng đuổi theo vật thể nếu vật bị xê dịch bất ngờ."

**[SLIDE 19 - DEMO TRỰC TIẾP] (Nếu thầy cô cho chạy Demo)**
"Sau đây, em xin phép chạy phần mềm giao diện HMI. (Vừa thao tác vừa nói) Đầu tiên là chuyển sang Tab Auto để robot đi theo đường thẳng vuông vức... Sau đó em đổi qua chế độ AI Mode, cố tình dùng chuột kéo vật thể ra xa, quý thầy cô có thể thấy robot lập tức bẻ lái tự tìm đường đuổi theo và gắp vật thành công."

**[SLIDE 20 - HẠN CHẾ & HƯỚNG PT]**
"Tuy nhiên, đồ án vẫn còn một số hạn chế. Hiện tại AI dùng tọa độ ảo tuyệt đối được cung cấp từ phần mềm mô phỏng. Tương lai, nhóm đề xuất tích hợp Camera chiều sâu như Intel RealSense để AI tự trích xuất tọa độ từ Point Cloud, làm cơ sở để có thể nhúng mô hình trực tiếp lên tay máy UR5e thật dưới xưởng sản xuất."

**[SLIDE 21 - LỜI CẢM ƠN] (Mỉm cười, cúi đầu chào)**
"Đồ án môn học này sẽ không thể hoàn thiện nếu thiếu đi sự hướng dẫn tận tình của Giảng viên. Đồng thời, em xin gửi lời cảm ơn quý thầy cô đã dành thời gian lắng nghe phần trình bày. Em xin kết thúc và bắt đầu phần giải đáp câu hỏi từ Hội đồng ạ!"
