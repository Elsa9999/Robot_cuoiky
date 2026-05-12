# THIẾT KẾ GIÁC HÚT CHÂN KHÔNG (VACUUM GRIPPER)

## 1. Phương pháp mô phỏng

Giác hút chân không được mô phỏng bằng **Ràng buộc JOINT_FIXED** trong PyBullet. Khi "hút", hệ thống tạo một mối nối cứng tàng hình giữa mũi kẹp (EE) và vật thể — vật sẽ di chuyển cùng tay máy. Khi "nhả", xóa mối nối → vật rơi tự do theo trọng lực.

Lớp `VacuumGripper` cung cấp 2 phương thức chính:
- `activate(object_id)` — Tạo Constraint gắn vật vào EE.
- `release()` — Xóa Constraint, vật rơi tự do.

## 2. Thuật toán gắn vật — Tính Offset cục bộ (Local Frame)

Khi kích hoạt giác hút, hệ thống **không** nối thẳng tâm EE với tâm vật. Thay vào đó, nó tính toán **Offset tương đối** giữa EE và vật tại thời điểm tiếp xúc, nhằm tránh hiện tượng Bùng nổ vật lý (Physics Explosion).

**Trình tự 4 bước:**

| Bước | Thao tác | Hàm PyBullet |
|------|---------|-------------|
| 1 | Đọc tọa độ tuyệt đối của EE (vị trí + quaternion) | `p.getLinkState()` |
| 2 | Đọc tọa độ tuyệt đối của vật thể | `p.getBasePositionAndOrientation()` |
| 3 | Tính nghịch đảo hệ tọa độ EE, rồi nhân với tọa độ vật → ra Offset cục bộ | `p.invertTransform()` + `p.multiplyTransforms()` |
| 4 | Tạo JOINT_FIXED sử dụng Offset đó làm `parentFramePosition` | `p.createConstraint()` |

**Về mặt toán học:** T_local = T⁻¹(EE) × T(Object), trong đó T là ma trận biến đổi thuần nhất 4×4.

**Đoạn code cốt lõi:**

```python
# Bước 3: Tính offset cục bộ
inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
obj_local_pos, obj_local_orn = p.multiplyTransforms(
    inv_ee_pos, inv_ee_orn,
    obj_pos,    obj_orn
)

# Bước 4: Tạo Constraint với Offset
self._constraint = p.createConstraint(
    parentBodyUniqueId     = self._robot_id,
    parentLinkIndex        = self._ee_link,
    childBodyUniqueId      = object_id,
    childLinkIndex         = -1,
    jointType              = p.JOINT_FIXED,
    jointAxis              = [0, 0, 0],
    parentFramePosition    = obj_local_pos,       # ← Offset đã tính
    childFramePosition     = [0, 0, 0],
    parentFrameOrientation = obj_local_orn
)
p.changeConstraint(self._constraint, maxForce=500)  # Lực giữ 500N
```

## 3. Vấn đề Physics Explosion và cách khắc phục

Nếu bỏ qua bước tính Offset và nối trực tiếp tâm EE với tâm vật (`parentFramePosition = [0,0,0]`), vật thể sẽ bị kéo giật (teleport) vào bên trong thân mũi kẹp, gây ra lồng ghép va chạm (Collision Penetration). Engine PyBullet phản ứng bằng cách đẩy vật bay văng ra xa với lực cực lớn.

Giải pháp đã áp dụng: Tính Offset cục bộ như Mục 2, đảm bảo vật thể được giữ nguyên tại vị trí tiếp xúc ban đầu mà không bị dịch chuyển.

## 4. Cơ chế nhả vật

```python
def release(self):
    p.removeConstraint(self._constraint)   # Xóa mối nối cứng
    self._constraint = None
    self._activated  = False
```

Khi gọi `p.removeConstraint()`, vật thể không còn bị ràng buộc và rơi tự do theo trọng lực (g = -9.81 m/s²).

## 5. Trực quan hóa trạng thái

Hệ thống vẽ vòng tròn 3D (8 cạnh, bán kính 3cm) tại mũi kẹp để hiển thị trạng thái:
- **Xanh lá:** Đang giữ vật (Constraint đang hoạt động).
- **Đỏ:** Rảnh / đã nhả (sẵn sàng gắp mới).

## 6. Tích hợp với 3 chế độ vận hành

| Chế độ | Ai điều khiển Gripper? | Cách gọi |
|--------|----------------------|---------|
| **Manual** | Người dùng nhấn phím | Gọi trực tiếp qua HMI |
| **Auto (FSM)** | Máy trạng thái | Tự động tại bước PICK (gắp) và RELEASE (nhả) |
| **AI (SAC)** | Hybrid Gripper | Tự động khi EE gần vật < 4.5cm hoặc gần bin < 5cm |
