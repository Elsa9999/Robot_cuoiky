"""
🤖 UR5e Pick & Place: Auto-Demo
Chạy inference (kiểm thử) mô hình Pick & Place hoàn hảo nhất (100% Success Rate).
"""
import os, sys, time, pickle
import numpy as np
import pybullet as p
from stable_baselines3 import SAC
from simulation.environment import UR5eEnvironment, CART_DELTA_MAX

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
os.chdir(ROOT)

MODEL_PATH = os.path.join(ROOT, "models_rl_17d", "seed42", "best_model.zip")

try:
    from train_17d_place import EULER_DELTA_MAX
except ImportError:
    EULER_DELTA_MAX = 0.08

def get_obs_20d(env: UR5eEnvironment) -> np.ndarray:
    ee_pos    = np.array(env.get_ee_position(),     dtype=np.float32)  
    obj_pose  = env.get_object_pose()
    obj_pos   = np.array(obj_pose[0],              dtype=np.float32)  
    obj_quat  = np.array(obj_pose[1],              dtype=np.float32)
    rel_obj   = obj_pos - ee_pos                                              
    bin_pos   = np.array(env.get_bin_center(),      dtype=np.float32)  
    rel_bin   = bin_pos - obj_pos                                             
    grip      = np.array([1.0 if env.is_gripping() else 0.0], dtype=np.float32)
    _, quat   = env.get_ee_pose()
    ee_euler  = np.array(p.getEulerFromQuaternion(quat), dtype=np.float32)
    return np.concatenate([ee_pos, obj_pos, rel_obj, rel_bin, obj_quat, grip, ee_euler])

def main():
    print("=" * 50)
    print("🚀 LOADING PERFECT PICK & PLACE MODEL (20D / 7D) 🚀")
    print("=" * 50)
    
    if not os.path.exists(MODEL_PATH):
        print(f"❌ Không tìm thấy model tại: {MODEL_PATH}")
        return

    # Khởi tạo môi trường có Bật Giao Diện (GUI=True)
    env = UR5eEnvironment(gui=True)
    
    # Load trọng số vinh quang
    # Custom_objects để tránh warning nếu gym/gymnasium version mis-match
    model = SAC.load(MODEL_PATH, device="cpu", custom_objects={"learning_rate": 0.0})
    print("✅ Model Loaded!")

    # Load VecNormalize stats
    vn_path = os.path.join(ROOT, "models_rl_17d", "seed42", "vecnormalize.pkl")
    obs_rms = None
    if os.path.exists(vn_path):
        with open(vn_path, "rb") as f:
            vec_norm = pickle.load(f)
        obs_rms = vec_norm.obs_rms
        obs_clip = vec_norm.clip_obs
        print("✅ VecNormalize Stats Loaded!")
    else:
        print("⚠️ Warning: Không tìm thấy vecnormalize.pkl!")

    for episode in range(5):
        print(f"\n--- Episode {episode + 1} ---")
        # Reset difficulty=1 để khớp với training hiện tại
        env.reset(difficulty=1)
        
        step = 0
        success = False
        
        while step < 200:
            obs = get_obs_20d(env)
            
            # Chuẩn hóa observation nếu sử dụng VecNormalize
            if obs_rms is not None:
                obs = (obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8)
                obs = np.clip(obs, -obs_clip, obs_clip)
            obs = obs.astype(np.float32)
            
            # Predict hành động (đã được train)
            action, _ = model.predict(obs, deterministic=True)
            action = np.clip(action, -1.0, 1.0)
            
            # Thực thi Action (move cartesian)
            delta_xyz   = action[:3] * CART_DELTA_MAX
            delta_euler = action[3:6] * EULER_DELTA_MAX
            env.move_ee_cartesian(delta_xyz, delta_euler)
            
            # Hybrid Gripper:
            # - Chưa gắp -> activate_gripper
            # - Đang gắp và EE gần bin center < 0.05m -> release_gripper
            is_gripping = env.is_gripping()
            if not is_gripping:
                env.activate_gripper()
            else:
                bin_center = np.array(env.get_bin_center())
                ee_pos = np.array(env.get_ee_position())
                if np.linalg.norm(ee_pos - bin_center) < 0.05:
                    env.release_gripper()
                 
            env.step(10)
            time.sleep(0.04) # Slower để nhìn rõ hơn
            step += 1
            
            # Kiểm tra xem vô lưới chưa
            if env.is_in_bin():
                print(f"🎉 THÀNH CÔNG! Trúng đích tại bước {step}")
                success = True
                time.sleep(1.0) # Dừng một giây để ngắm thành quả
                break
                
        if not success:
            print("❌ Thất bại (hết thời gian).")
        
    print("\n[DONE] Hoàn tất quá trình Test.")
    env.close()
    
if __name__ == "__main__":
    main()
