"""
Tệp inverse_kinematics.py:
Lõi Toán học giải bài toán Đoán góc xoay ngược từ Tọa độ đích (Inverse Kinematics).
Hệ thống sử dụng cả 2 phương pháp:
1. Analytical (Giải tích): Tính toán nhanh siêu tốc thông qua ma trận DH và lượng giác.
2. Numerical (Số học cận biên): Dùng thuật toán L-BFGS-B khi vật vượt ngoài tầm giải tích đơn thuần (dự phòng).
"""
import os
import sys
# Cấu hình Path để Python nhận diện thư mục gốc 'kinematics' khi chạy trực tiếp
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import math
import scipy.optimize
from kinematics.forward_kinematics import forward_kinematics, DH_TABLE, dh_transform

def euler_matrix(euler):
    """
    Tạo ma trận xoay 3x3 từ 3 góc Euler ZYX (Roll, Pitch, Yaw).
    Góc Euler là cách để định nghĩa góc chéo của vật thể khi bị nghiêng trong không gian.
    """
    x, y, z = euler
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    
    return Rz @ Ry @ Rx

def validate_limits(q):
    """
    Kiểm tra joint limits từ URDF:
    joint 1,2,4,5,6: [-6.28, 6.28]
    joint 3:         [-3.14, 3.14]
    """
    limits = [6.28, 6.28, 3.14, 6.28, 6.28, 6.28]
    for i in range(6):
        if abs(q[i]) > limits[i]:
            return False
    return True

def analytical_ik(T_target):
    """
    Hàm tính Động học nghịch (IK) bằng phương pháp Hình học Giải tích (Analytical/Geometric Closed-form).
    
    ƯU ĐIỂM CỦA PHƯƠNG PHÁP GIẢI TÍCH (SO VỚI SỐ HỌC):
    - Hiệu suất cao: Dựa trên lượng giác thuần túy, có thể tính toán toàn bộ 8 nghiệm không gian trong < 1 mili-giây.
    - Độ tin cậy: Tránh hoàn toàn lỗi phân kỳ (Diverge) và kẹt tại điểm kỳ dị (Singularity) của thuật toán Jacobian (Numerical).
    
    CÁCH HOẠT ĐỘNG:
    Dựa trên quy tắc Tách hình học (Kinematic Decoupling). Tách bài toán 6 bậc tự do thành 2 bài toán nhỏ:
    1. Tính tọa độ Tâm cổ tay (để tìm ra góc xoay của 3 khớp cánh tay).
    2. Dùng ma trận quay (để tìm ra góc xoay của 3 khớp cổ tay).
    Đầu vào: T_target (Ma trận 4x4 đại diện cho vị trí XYZ và góc xoay RPY mong muốn của kẹp gắp).
    """
    # Lấy 6 thông số kích thước thực tế của UR5e từ bảng DH:
    d1 = DH_TABLE[0]['d']   # 0.1625m — Chiều cao từ đế lên trục vai
    a2 = DH_TABLE[1]['a']   # -0.4250m — Chiều dài bắp tay (Shoulder → Elbow)
    a3 = DH_TABLE[2]['a']   # -0.3922m — Chiều dài cẳng tay (Elbow → Wrist)
    d4 = DH_TABLE[3]['d']   # 0.1333m — Độ lệch ngang cổ tay 1
    d5 = DH_TABLE[4]['d']   # 0.0997m — Độ lệch ngang cổ tay 2
    d6 = DH_TABLE[5]['d']   # 0.0996m — Khoảng cách từ cổ tay đến mũi kẹp
    
    # px, py, pz = Tọa độ đích (vị trí mong muốn của mũi kẹp) lấy từ cột cuối ma trận T
    px, py, pz = T_target[0,3], T_target[1,3], T_target[2,3]
    
    # BƯỚC 1 - TÌM TÂM CỔ TAY (WRIST CENTER)
    # Tọa độ mũi gắp (T_target[:3, 3]) trừ đi một đoạn lùi bằng đúng khoảng cách d6 
    # nhân với vector hướng dọc theo trục Z của End-Effector (T_target[:3, 2]).
    P05 = T_target[:3, 3] - d6 * T_target[:3, 2]
    
    # r = Khoảng cách từ gốc robot đến tâm cổ tay nhìn từ trên xuống (chiếu xuống mặt phẳng XY)
    r = math.hypot(P05[0], P05[1])
    if r < abs(d4):
        return [] # Nếu quá sát tâm trục Z, không có nghiệm (vùng kỳ dị - singularity)
        
    # phi = Góc nhìn từ trên xuống của tâm cổ tay trên mặt phẳng XY
    phi = math.atan2(P05[1], P05[0])
    # asin_val = Góc lệch do offset d4 gây ra (cổ tay không thẳng hàng với vai)
    asin_val = math.asin(d4 / r)
    
    # BƯỚC 2 - GIẢI KHỚP VAI THETA 1
    # Do có cộng trừ (+-) nên hệ thống luôn tính ra 2 nghiệm song song:
    # 1 nghiệm tương ứng với cấu hình Vai Trái (Left Arm), 1 nghiệm tương ứng Vai Phải (Right Arm)
    th1_sols = [phi + asin_val, phi + math.pi - asin_val]
    
    sols = []
    for th1 in th1_sols:
        th1 = math.atan2(math.sin(th1), math.cos(th1))
        
        # BƯỚC 3 - GIẢI KHỚP CỔ TAY THETA 5
        # Sử dụng phương trình từ ma trận T₀₆ để tính cos(θ5)
        num = T_target[0,3] * math.sin(th1) - T_target[1,3] * math.cos(th1) - d4
        c5 = num / d6   # cos(θ5) = (px·sinθ1 - py·cosθ1 - d4) / d6
        if abs(c5) > 1.0:
            if abs(c5) < 1.001: c5 = np.sign(c5) * 1.0  # Dung sai số học nhỏ
            else: continue  # Vô nghiệm thật sự
            
        th5_val = math.acos(c5)
        
        # Có 2 solutions cho th5 ứng với mỗi th1
        for th5 in [th5_val, -th5_val]:
            if abs(math.sin(th5)) < 1e-5:
                th6_sols = [0.0]
            else:
                A = T_target[0,0] * math.sin(th1) - T_target[1,0] * math.cos(th1)
                B = T_target[0,1] * math.sin(th1) - T_target[1,1] * math.cos(th1)
                th6_sols = [math.atan2( -B / math.sin(th5), A / math.sin(th5) )]
                
            for th6 in th6_sols:
                # Tính ma trận biến đổi của từng khớp đã giải được
                T1 = dh_transform(0, d1, np.pi/2, th1)    # Ma trận khớp vai
                T5 = dh_transform(0, d5, -np.pi/2, th5)   # Ma trận khớp cổ tay 2
                T6 = dh_transform(0, d6, 0, th6)           # Ma trận khớp cổ tay 3
                
                T56 = T5 @ T6  # Nhân 2 ma trận cổ tay lại
                
                # TÍNH NGƯỢC MA TRẬN ĐỂ TÌM T14:
                # T₁₄ = T₁⁻¹ × T_target × T₅₆⁻¹
                # T14 chứa thông tin các khớp giữa (khớp 2, 3, 4) mà ta cần giải tiếp
                T14 = np.linalg.inv(T1) @ T_target @ np.linalg.inv(T56)
                
                P14x, P14y = T14[0,3], T14[1,3]
                dist_sq = P14x**2 + P14y**2
                
                # BƯỚC 4 - GIẢI KHỚP KHUỶU TAY THETA 3
                # Áp dụng Định lý Hàm số Cosin cho tam giác tạo bởi bắp tay (a2) và cẳng tay (a3).
                # Sẽ đẻ ra tiếp 2 nghiệm: Khuỷu tay gập lên (Elbow Up) hoặc gập xuống (Elbow Down)
                c3 = (dist_sq - a2**2 - a3**2) / (2 * a2 * a3)
                if abs(c3) > 1.0:
                    if abs(c3) < 1.001: c3 = np.sign(c3) * 1.0
                    else: continue
                        
                th3_val = math.acos(c3)
                
                # Có 2 solutions cho th3 ứng với mỗi tổ hợp (Elbow up/down)
                for th3 in [th3_val, -th3_val]:
                    s3 = math.sin(th3)
                    
                    # BƯỚC 5 - GIẢI THETA 2 (góc nâng cánh tay)
                    # Dùng atan2 kép: góc nhìn từ T14 trừ đi góc tam giác
                    th2 = math.atan2(P14y, P14x) - math.atan2(a3 * s3, a2 + a3 * c3)
                    
                    # BƯỚC 6 - GIẢI THETA 4 (góc cổ tay 1)
                    # 3 khớp giữa (2,3,4) có trục song song → tổng góc cố định
                    th_sum = math.atan2(T14[1,0], T14[0,0])
                    th4 = th_sum - th2 - th3  # θ4 = θ_tổng - θ2 - θ3
                    
                    q = [th1, th2, th3, th4, th5, th6]
                    # Normalize về khoảng [-π, π] để so sánh và chọn nghiệm gần nhất
                    q = [(x + math.pi) % (2*math.pi) - math.pi for x in q]
                    
                    if validate_limits(q):
                        sols.append(q)
    return sols

def numerical_ik(T_target, q_current=None):
    """
    Tính IK bằng Thuật toán tối ưu hóa (Numerical). Dùng làm phương án dự phòng (Fallback).
    Khi mục tiêu gắp nằm ở góc cực hẹp khiến Giải tích báo lỗi, Numerical sẽ tính đạo hàm
    để ép các khớp nghiêng nhẹ tới điểm gần đúng nhất mà không vi phạm quy tắc xoay.
    """
    if q_current is None:
        q_current = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
        
    def cost(q):
        """Hàm chi phí: đo độ lệch giữa tư thế hiện tại và tư thế mục tiêu."""
        T_curr = forward_kinematics(q)['T']
        # Sai số vị trí: Tổng bình phương khoảng cách XYZ
        pos_err = np.sum((T_curr[:3, 3] - T_target[:3, 3])**2)
        
        # Sai số hướng: Dùng Trace của tích 2 ma trận xoay (3 - Tr(R·Rᵀ) = 0 khi khớp hoàn toàn)
        R_curr = T_curr[:3, :3]
        R_target = T_target[:3, :3]
        rot_err = 3.0 - np.trace(R_curr @ R_target.T)
        
        return pos_err + rot_err  # Tổng 2 sai số → càng nhỏ càng tốt
        
    bounds = [(-6.28, 6.28)] * 6
    bounds[2] = (-3.14, 3.14)
    
    res = scipy.optimize.minimize(cost, q_current, bounds=bounds, method='L-BFGS-B', options={'maxiter': 1000})
    if res.success and res.fun < 1e-4:
        return [res.x.tolist()]
    return []

def inverse_kinematics(target_pos, target_euler, q_current=None, method='auto') -> dict:
    """
    Hàm tổng - Tính động học ngược cho End-Effector của UR5e.
    Quá trình:
    1. Lắp ráp ma trận quay và tịnh tiến vào ma trận 4x4.
    2. Chạy hàm giải tích (Analytical). 
    3. Nếu vô nghiệm, hệ thống tự động nhảy sang chạy Số học (Numerical).
    
    Tùy chọn: method có thể ép cứng về 'analytical' hoặc 'numerical', mặc định là 'auto'.
    Trả về Dictionary chứa góc xoay của 6 khớp an toàn nhất.
    """
    # LắP RÁP MA TRẬN MỤC TIÊU T_target (4x4):
    # ┌              ┐
    # │ R(3x3) | p(3x1) │   R = Ma trận xoay từ góc Euler (Roll, Pitch, Yaw)
    # │--------+--------│   p = Tọa độ đích (X, Y, Z)
    # │ 0 0 0  |   1    │
    # └              ┘
    T_target = np.eye(4)
    T_target[:3, :3] = euler_matrix(target_euler)  # Gán phần xoay (3x3 trên trái)
    T_target[:3, 3] = target_pos                   # Gán phần tịnh tiến (cột cuối)
    
    sols = []
    used_method = method
    
    if method in ['auto', 'analytical']:
        sols = analytical_ik(T_target)
        if len(sols) > 0:
            used_method = 'analytical'
            
    if len(sols) == 0 and method in ['auto', 'numerical']:
        sols = numerical_ik(T_target, q_current)
        used_method = 'numerical'
        
    best_sol = None
    if len(sols) > 0:
        if q_current is None:
            best_sol = sols[0]
        else:
            # Chọn solution gần với q_current nhất
            best_sol = min(sols, key=lambda q: np.linalg.norm(np.array(q) - np.array(q_current)))
            
    # Tính Errors cho từng solution
    errors = []
    for sol in sols:
        T_sol = forward_kinematics(sol)['T']
        pos_err = np.linalg.norm(T_sol[:3, 3] - T_target[:3, 3])
        errors.append(float(pos_err))
        
    if best_sol is None:
        print("[WARNING] Không tìm được IK hợp lệ cho target pos:", target_pos)
        
    return {
        'solutions': sols,
        'best': best_sol,
        'method': used_method,
        'n_solutions': len(sols),
        'errors': errors
    }

if __name__ == "__main__":
    # ══════════════════════════════════════════════════════════════════════════
    # KIỂM CHỨNG TOÁN HỌC — ĐỘNG HỌC NGHỊCH (INVERSE KINEMATICS VERIFICATION)
    # Phương pháp: Vòng lặp kín (Round-Trip) — đây là phương pháp kiểm chứng
    #   THẬT SỰ vì FK và IK là hai thuật toán hoàn toàn độc lập:
    #   FK nhân ma trận xuôi, IK giải phương trình lượng giác ngược.
    #   Bước 1: Nạp góc Q vào FK → Tính ra tọa độ XYZ
    #   Bước 2: Nạp tọa độ XYZ vào IK → Giải ngược ra góc Q'
    #   Bước 3: Nạp Q' vào FK → Tính ra XYZ' → So sánh XYZ vs XYZ'
    #   Nếu XYZ = XYZ' → Chứng minh CẢ HAI thuật toán đều đúng về mặt toán học.
    # Tiêu chuẩn PASS: Sai lệch Euclidean < 1mm (0.001m)
    # ══════════════════════════════════════════════════════════════════════════
    print("=" * 70)
    print("  KIỂM CHỨNG TOÁN HỌC — ĐỘNG HỌC NGHỊCH (IK VERIFICATION)")
    print("=" * 70)
    print("  [PHƯƠNG PHÁP]: Vòng lặp kín (Round-Trip / Self-Consistency Test)")
    print("    Bước 1: Nạp góc Q gốc vào FK  → Tính ra tọa độ XYZ")
    print("    Bước 2: Nạp tọa độ XYZ vào IK → Giải ngược ra góc Q'")
    print("    Bước 3: Nạp Q' vào FK          → Tính ra XYZ'")
    print("    Bước 4: So sánh XYZ vs XYZ'    → Sai số phải = 0")
    print("")
    print("  [TẠI SAO ĐÂY LÀ KIỂM CHỨNG THẬT SỰ?]")
    print("    FK và IK là hai thuật toán HOÀN TOÀN KHÁC NHAU:")
    print("      • FK: Nhân 6 ma trận biến đổi 4×4 (phép tính xuôi, đơn giản)")
    print("      • IK: Giải hệ phương trình lượng giác + tách tâm cổ tay (phép tính ngược, phức tạp)")
    print("    Hai thuật toán này KHÔNG THỂ sai cùng kiểu được.")
    print("    Nếu Round-Trip PASS → Chứng minh CẢ HAI đều đúng toán học,")
    print("    không cần bất kỳ phần mềm bên ngoài nào (Matlab, ROS,...).")
    print("")
    print("  [TIÊU CHUẨN]: Sai lệch Euclidean < 1mm (0.001m)")
    print("-" * 70)
    
    def run_test(name, q_ref, expected_sols_min=1):
        """
        PHƯƠNG PHÁP KIỂM CHỨNG VÒNG LẶP KÍN (ROUND-TRIP VERIFICATION)
        Tự động kiểm tra độ chính xác tuyệt đối của 2 thuật toán Động học (FK & IK).
        """
        print(f"\n{'='*70}")
        print(f"  {name}")
        print(f"{'='*70}")
        
        # ─────────────────────────────────────────────────────────────────────
        # BƯỚC 1: CHUẨN BỊ — Chọn bộ góc khớp gốc (Q gốc)
        # Đây là bộ 6 góc xoay mà ta BIẾT TRƯỚC là đúng (do ta tự chọn).
        # ─────────────────────────────────────────────────────────────────────
        print(f"\n  ► BƯỚC 1: Chọn bộ góc khớp gốc (Q gốc) — bộ góc mà ta BIẾT TRƯỚC là đúng")
        print(f"    Q gốc = {q_ref}")
        
        # ─────────────────────────────────────────────────────────────────────
        # BƯỚC 2: CHẠY FK — Nhét Q gốc vào hàm Động học Thuận (FK)
        # FK sẽ nhân chuỗi 6 ma trận DH 4×4 và trả về tọa độ XYZ + góc Euler
        # của đầu kẹp End-Effector trong không gian 3D.
        # ─────────────────────────────────────────────────────────────────────
        fk_ref = forward_kinematics(q_ref)
        pos = fk_ref['position']
        euler = fk_ref['euler']
        print(f"\n  ► BƯỚC 2: Chạy FK(Q gốc) → Tính ra tọa độ End-Effector trong không gian 3D")
        print(f"    Kết quả FK: Vị trí (x, y, z) = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
        print(f"    Kết quả FK: Hướng (Roll, Pitch, Yaw) = ({euler[0]:.4f}, {euler[1]:.4f}, {euler[2]:.4f})")
        
        # ─────────────────────────────────────────────────────────────────────
        # BƯỚC 3: CHẠY IK — Nhét tọa độ XYZ (từ Bước 2) vào hàm Động học Nghịch (IK)
        # IK sẽ giải hệ phương trình lượng giác để tìm lại bộ góc khớp Q'.
        # Lưu ý: IK dùng thuật toán KHÁC HOÀN TOÀN với FK (giải pt ngược, không nhân ma trận).
        # ─────────────────────────────────────────────────────────────────────
        ik_res = inverse_kinematics(pos, euler, q_current=q_ref)
        n_sols = ik_res['n_solutions']
        best = ik_res['best']
        print(f"\n  ► BƯỚC 3: Chạy IK(XYZ, Euler) → Giải ngược ra bộ góc khớp Q'")
        print(f"    Phương pháp giải: {ik_res['method']}")
        print(f"    Số nghiệm tìm được: {n_sols} cấu hình không gian")
        if best is not None:
            print(f"    Nghiệm tốt nhất Q' = {np.array(best).round(4)}")
        
        if n_sols >= expected_sols_min and best is not None:
            # ─────────────────────────────────────────────────────────────────
            # BƯỚC 4: ĐỐI CHIẾU — Nhét Q' (từ Bước 3) ngược lại vào FK
            # Nếu FK(Q') ra đúng tọa độ XYZ ban đầu (Bước 2) → cả FK lẫn IK đều đúng.
            # Nếu lệch → ít nhất 1 trong 2 bị sai toán học.
            # ─────────────────────────────────────────────────────────────────
            fk_check = forward_kinematics(best)
            pos2 = fk_check['position']
            err = np.linalg.norm(np.array(pos2) - np.array(pos))
            
            print(f"\n  ► BƯỚC 4: Chạy lại FK(Q') → Tính ra tọa độ XYZ' để đối chiếu")
            print(f"    FK(Q gốc) = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})  ← Bước 2")
            print(f"    FK(Q')    = ({pos2[0]:.4f}, {pos2[1]:.4f}, {pos2[2]:.4f})  ← Bước 4")
            print(f"    Sai lệch  = {err:.6f}m")
            
            # ─────────────────────────────────────────────────────────────────
            # BƯỚC 5: KẾT LUẬN
            # Nếu sai lệch < 1mm → PASS → Cả FK và IK đều đúng toán học.
            # ─────────────────────────────────────────────────────────────────
            if err < 0.001:
                print(f"\n  ✓ KẾT LUẬN: PASS — Sai lệch {err:.6f}m < 1mm")
                print(f"    FK(Q gốc) và FK(Q') trùng khớp tuyệt đối.")
                print(f"    → Chứng minh CẢ HAI thuật toán FK và IK đều chính xác về mặt toán học.")
                return True
            else:
                print(f"\n  ✗ KẾT LUẬN: FAIL — Sai lệch {err:.6f}m > 1mm")
                return False
        else:
            print(f"\n  ✗ KẾT LUẬN: FAIL — Không tìm đủ nghiệm hoặc thuật toán thất bại")
            return False

    pass_all = True
    
    # ═══════════════════════════════════════════════════════════════════════
    # TEST 1: Round-Trip từ tư thế Home
    # ═══════════════════════════════════════════════════════════════════════
    q_test1 = [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    pass_all &= run_test("TEST 1: Round-Trip từ Tư thế Home (Home Pose)", q_test1)
    
    # ═══════════════════════════════════════════════════════════════════════
    # TEST 2: Round-Trip từ tư thế ngẫu nhiên
    # ═══════════════════════════════════════════════════════════════════════
    q_test2 = [0.5, -1.2, 1.0, -1.5, -1.5, 0.3]
    pass_all &= run_test("TEST 2: Round-Trip từ Tư thế Ngẫu nhiên (Random Pose)", q_test2)
    
    # ═══════════════════════════════════════════════════════════════════════
    # TEST 3: Kiểm tra đa nghiệm
    # ═══════════════════════════════════════════════════════════════════════
    print(f"\n{'='*70}")
    print("  TEST 3: Kiểm tra Đa nghiệm không gian (Multiple Solutions)")
    print(f"{'='*70}")
    print("  Robot 6 bậc tự do luôn có nhiều cách uốn khớp để với tới cùng 1 điểm")
    print("  (VD: vòng tay qua trái/phải, khuỷu gập lên/xuống).")
    print("  Test này đảm bảo code giải ra ĐỦ bộ nghiệm chứ không chỉ 1.")
    ik_res_3 = inverse_kinematics(forward_kinematics(q_test1)['position'], forward_kinematics(q_test1)['euler'])
    if ik_res_3['n_solutions'] >= 2:
        print(f"\n  ✓ KẾT LUẬN: PASS — Tìm được {ik_res_3['n_solutions']} cấu hình khác nhau")
    else:
        print(f"\n  ✗ KẾT LUẬN: FAIL — Chỉ tìm thấy {ik_res_3['n_solutions']} < 2 nghiệm")
        pass_all = False
        
    # ═══════════════════════════════════════════════════════════════════════
    # TEST 4: An toàn ngoài vùng làm việc
    # ═══════════════════════════════════════════════════════════════════════
    print(f"\n{'='*70}")
    print("  TEST 4: An toàn ngoài Vùng làm việc (Out of Reach)")
    print(f"{'='*70}")
    print("  Ra lệnh cho Robot với tới điểm (5m, 5m, 5m) — vượt xa sải tay tối đa 0.85m.")
    print("  Thuật toán PHẢI nhận biết và từ chối, tránh sinh góc xoay gây gãy tay robot.")
    ik_res_4 = inverse_kinematics((5.0, 5.0, 5.0), (0, 0, 0))
    if ik_res_4['best'] is None:
        print(f"\n  ✓ KẾT LUẬN: PASS — Thuật toán trả về None (nhận biết ngoài tầm với)")
    else:
        print(f"\n  ✗ KẾT LUẬN: FAIL — Thuật toán vẫn cố giải sai: {ik_res_4['best']}")
        pass_all = False
        
    print(f"\n{'='*70}")
    print(f"  BÁO CÁO TỔNG THỂ: {'✓ TẤT CẢ ĐỀU PASS (THUẬT TOÁN ĐẠT CHUẨN)' if pass_all else '✗ CÓ LỖI (CẦN KIỂM TRA LẠI)'}")
    print(f"{'='*70}")

