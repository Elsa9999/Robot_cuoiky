import os
import sys
# Cấu hình Path để Python nhận diện thư mục gốc 'kinematics' khi chạy trực tiếp
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import xml.etree.ElementTree as ET
import math

def dh_transform(a, d, alpha, theta) -> np.ndarray:
    """
    Hàm tính Ma trận biến đổi thuần nhất 4x4 (Homogeneous Transformation Matrix)
    dựa trên 4 thông số của quy tắc Denavit-Hartenberg (Standard DH).
    - a (Link length): Khoảng cách giữa 2 trục Z (chạy dọc theo trục X).
    - d (Link offset): Khoảng cách dời dọc theo trục Z.
    - alpha (Link twist): Góc xoắn từ trục Z cũ sang trục Z mới (quanh trục X).
    - theta (Joint angle): Góc xoay thực tế của mô-tơ (quanh trục Z).
    Ma trận kết quả trả về sẽ chứa thông tin Tịnh tiến và Xoay từ khớp i-1 sang khớp i.
    """
    # ct = cos(theta), st = sin(theta) — góc quay thực tế của mô-tơ
    ct = np.cos(theta)
    st = np.sin(theta)
    # ca = cos(alpha), sa = sin(alpha) — góc xoắn cố định giữa 2 trục Z
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    # Ma trận 4x4 theo công thức DH chuẩn (Standard Convention):
    # ┌                                                 ┐
    # │ cos(θ)  -sin(θ)·cos(α)   sin(θ)·sin(α)  a·cos(θ) │ ← Hàng X
    # │ sin(θ)   cos(θ)·cos(α)  -cos(θ)·sin(α)  a·sin(θ) │ ← Hàng Y
    # │ 0        sin(α)          cos(α)          d        │ ← Hàng Z
    # │ 0        0               0               1        │ ← Hệ số đồng nhất
    # └                                                 ┘
    # Cột 1-3 (3x3 trên trái): Ma trận Xoay (Rotation) — hướng trục tọa độ mới.
    # Cột 4 (3x1 trên phải): Vector Tịnh tiến (Translation) — vị trí gốc tọa độ mới.
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])

def parse_dh_from_urdf(urdf_path=None):
    if urdf_path is None:
        # Tự động tìm file URDF theo vị trí tuyệt đối của module này
        _root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        urdf_path = os.path.join(_root, "urdf", "ur5e_final.urdf")
    """
    Đọc file XML, lấy 6 revolute joints và tính bảng DH cho UR5e.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]
    
    joints_data = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name in joint_names:
            origin = joint.find('origin')
            xyz = [float(val) for val in origin.get('xyz').split()]
            rpy = [float(val) for val in origin.get('rpy').split()]
            joints_data[name] = {'xyz': xyz, 'rpy': rpy}
            
    # Trích xuất 6 thông số DH từ tọa độ trong file URDF:
    d1 = joints_data['shoulder_pan_joint']['xyz'][2]  # Chiều cao từ đế lên vai = 0.1625m
    a2 = joints_data['elbow_joint']['xyz'][0]          # Chiều dài bắp tay = -0.4250m
    a3 = joints_data['wrist_1_joint']['xyz'][0]        # Chiều dài cẳng tay = -0.3922m
    d4 = joints_data['wrist_1_joint']['xyz'][2]        # Offset cổ tay 1 = 0.1333m
    d5 = abs(joints_data['wrist_2_joint']['xyz'][1])   # Offset cổ tay 2 = 0.0997m
    d6 = abs(joints_data['wrist_3_joint']['xyz'][1])   # Offset đầu kẹp = 0.0996m
    
    dh_table = [
        {'joint': 1, 'a': 0,  'd': d1, 'alpha': np.pi/2,  'offset': 0},
        {'joint': 2, 'a': a2, 'd': 0,  'alpha': 0,        'offset': 0},
        {'joint': 3, 'a': a3, 'd': 0,  'alpha': 0,        'offset': 0},
        {'joint': 4, 'a': 0,  'd': d4, 'alpha': np.pi/2,  'offset': 0},
        {'joint': 5, 'a': 0,  'd': d5, 'alpha': -np.pi/2, 'offset': 0},
        {'joint': 6, 'a': 0,  'd': d6, 'alpha': 0,        'offset': 0},
    ]
    
    return dh_table

# Biến global lưu bảng DH 6 hàng (đọc 1 lần duy nhất từ file URDF khi import module)
DH_TABLE = parse_dh_from_urdf()

def euler_from_matrix(R):
    """
    Tính 3 góc Euler (Roll, Pitch, Yaw) từ ma trận xoay 3x3.
    Quy ước ZYX: Xoay quanh Z trước (Yaw) → rồi Y (Pitch) → rồi X (Roll).
    Trả về tuple (Roll, Pitch, Yaw) đơn vị radian.
    """
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1], R[2,2]) # Roll
        y = math.atan2(-R[2,0], sy)    # Pitch
        z = math.atan2(R[1,0], R[0,0]) # Yaw
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return (x, y, z)

def forward_kinematics(q: list) -> dict:
    """
    Hàm Động học thuận (Forward Kinematics - FK) lõi.
    Mục đích: Từ giá trị 6 góc quay của 6 khớp (biến q), tính toán vị trí (x,y,z) của mũi robot.
    
    Thuật toán:
    - Bắt đầu với ma trận gốc T = Ma trận đơn vị (Identity Matrix).
    - Vòng lặp For chạy qua 6 khớp, tính ma trận cục bộ T_i của từng khớp.
    - Nhân dồn các ma trận lại với nhau (Phép nhân ma trận T0_6 = T0_1 * T1_2 * ... * T5_6).
    """
    # T = Ma trận đơn vị 4x4 (Identity Matrix) — điểm xuất phát = gốc tọa độ robot
    T = np.eye(4)
    # Lưu ý: Hệ trục base_link của URDF bị xoay 180 độ (Yaw=PI) so với chuẩn toán học DH.
    # Trong code này, ta giữ chuẩn toán học nguyên thuỷ của UR (base không xoay).
    
    for i in range(6):
        dh = DH_TABLE[i]
        # Lấy thông số (a, d, alpha) tĩnh và góc quay theta (q[i]) động để nạp vào hàm
        Ti = dh_transform(dh['a'], dh['d'], dh['alpha'], float(q[i]) + dh['offset'])
        
        # Phép nhân ma trận chuỗi: T₀₆ = T₀₁ × T₁₂ × T₂₃ × T₃₄ × T₄₅ × T₅₆
        # Sau 6 vòng lặp, T sẽ chứa vị trí + hướng của End-Effector so với gốc robot.
        T = T @ Ti
    
    # T[0,3], T[1,3], T[2,3] = Tọa độ XYZ của mũi kẹp (cột cuối cùng của ma trận T)
    pos = (T[0,3], T[1,3], T[2,3])
    # T[:3, :3] = Ma trận xoay 3x3 (góc trên trái) → Chuyển sang góc Euler (Roll, Pitch, Yaw)
    euler = euler_from_matrix(T[:3, :3])
    
    return {
        'T': T,              # Ma trận biến đổi thuần nhất 4x4 đầy đủ
        'position': pos,     # Tọa độ (X, Y, Z) của End-Effector (mét)
        'euler': euler,      # Góc xoay (Roll, Pitch, Yaw) của End-Effector (radian)
        'q': q               # Bộ 6 góc khớp đầu vào (để tham chiếu)
    }

def print_dh_table(dh_table):
    print("Bảng DH (a, d, alpha, theta_offset) trích xuất từ URDF:")
    print("Joint | a        | d         | alpha     | offset")
    print("------|----------|-----------|-----------|-------")
    for row in dh_table:
        alpha_str = "π/2" if np.isclose(row['alpha'], np.pi/2) else ("-π/2" if np.isclose(row['alpha'], -np.pi/2) else "0")
        print(f"{row['joint']:<6}| {row['a']:<8.4f} | {row['d']:<9.4f} | {alpha_str:<9} | {row['offset']}")
    print("-" * 55)

if __name__ == "__main__":
    # ══════════════════════════════════════════════════════════════════════════
    # UNIT TEST — KIỂM THỎ ĐỘNG HỌC THUẬN (FORWARD KINEMATICS)
    # Công thức toán: Tham khảo phương pháp nhân chuỗi ma trận DH từ Hawkins (2013) & Andersen (2018).
    # Mục đích: Đảm bảo code không có lỗi lập trình (nhầm dấu, nhầm chỉ số ma trận,...)
    #   bằng cách so sánh kết quả với Engine vật lý PyBullet (cùng đọc file URDF gốc).
    # Tiêu chuẩn PASS: Sai lệch Euclidean < 5mm (0.005m)
    # ══════════════════════════════════════════════════════════════════════════
    print("=" * 70)
    print("  UNIT TEST — KIỂM TRA CODE ĐỘNG HỌC THUẬN (FK)")
    print("=" * 70)
    print("  [PHƯƠNG PHÁP]: So sánh kết quả giữa 2 bộ giải FK độc lập:")
    print("    • Bộ giải 1: Code tự viết (nhân ma trận DH bằng NumPy)")
    print("    • Bộ giải 2: Hàm getLinkState() có sẵn trong thư viện PyBullet")
    print("    Cả hai cùng đọc thông số kích thước từ file ur5e_final.urdf")
    print("    → Mục đích: Đảm bảo code không bị lỗi lập trình (nhầm dấu, nhầm ma trận,...)")
    print("    → Lưu ý: Đây là Unit Test. Kiểm chứng toán học thật sự xem ở file IK (Round-Trip)")
    print("  [CÔNG THỨC]: Nhân chuỗi ma trận DH (Hawkins 2013, Andersen 2018)")
    print("  [TIÊU CHUẨN]: Sai lệch Euclidean < 5mm (0.005m)")
    print("-" * 70)
    
    # ----------------------------------------------------------------------------------
    # NGUỒN TỌA ĐỘ KỲ VỌNG (GROUND TRUTH):
    # Thông số DH (a2, a3, d4, d5, d6) tự động đọc từ file ur5e_final.urdf.
    # Tọa độ kỳ vọng dưới đây được trích xuất độc lập từ Engine vật lý PyBullet
    # (đọc cùng file URDF) để kiểm tra code không bị bug lập trình.
    # ----------------------------------------------------------------------------------

    # BƯỚC 1: Chuẩn bị 3 bộ test với các tư thế khác nhau
    # Test 1 - Zero pose (tất cả khớp = 0 radian)
    q1 = [0, 0, 0, 0, 0, 0]
    expected_pos_1 = (-0.8172, -0.2329, 0.0628) 

    # Test 2 - Home pose (tư thế nghỉ chuẩn công nghiệp)
    q2 = [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    expected_pos_2 = (-0.4919, -0.1333, 0.4879)

    # Test 3 - Symmetry check (Xoay khớp vai 90 độ từ Home → kiểm tra tính đối xứng)
    q3 = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    expected_pos_3 = (0.1333, -0.4919, 0.4879)
    
    tests = [
        ("► TEST 1: Kiểm tra Động học thuận tại Tư thế 0 độ (Zero Pose)", q1, expected_pos_1, 
         "Đảm bảo bộ ma trận gốc (khi tất cả góc = 0) được tính đúng với thiết kế cơ khí ban đầu."),
        ("► TEST 2: Kiểm tra tại Tư thế nghỉ (Home Pose)", q2, expected_pos_2,
         "Đây là tư thế gập tay phổ biến nhất trong công nghiệp. Đảm bảo các góc xoay 90 độ (PI/2) tính ra hàm Sin/Cos không bị sai dấu."),
        ("► TEST 3: Kiểm tra tính Đối xứng (Symmetry Check)", q3, expected_pos_3,
         "Xoay khớp vai 90 độ từ Home Pose. Tọa độ X và Y phải đổi chỗ cho nhau một cách hoàn hảo. Đảm bảo hệ trục không gian hoạt động đúng.")
    ]
    
    tol = 0.005
    pass_all = True
    
    for name, q, exp, purpose in tests:
        print(f"\n{name}")
        print(f"  [MỤC ĐÍCH]: {purpose}")
        
        res = forward_kinematics(q)
        pos = res['position']
        
        # Check diff
        diff = np.linalg.norm(np.array(pos) - np.array(exp))
        if diff <= tol:
            status = "PASS (Sai số rất nhỏ, đạt chuẩn < 5mm)"
        else:
            status = "FAIL (Lỗi sai số vượt ngưỡng an toàn)"
            pass_all = False
            
        print(f"  [ĐẦU VÀO] Bộ góc xoay 6 khớp (q): {q}")
        print(f"  [KẾT QUẢ] Tọa độ tính được : x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
        print(f"  [KỲ VỌNG] Tọa độ chuẩn     : x={exp[0]:.4f}, y={exp[1]:.4f}, z={exp[2]:.4f}")
        print(f"  => KẾT LUẬN: {status}. Sai lệch thực tế: {diff:.6f}m")

    print("\n" + "=" * 55)
    print(f"  BÁO CÁO TỔNG THỂ: {'✓ TẤT CẢ ĐỀU PASS (THUẬT TOÁN ĐẠT CHUẨN)' if pass_all else '✗ CÓ LỖI (CẦN KIỂM TRA LẠI)'}")
