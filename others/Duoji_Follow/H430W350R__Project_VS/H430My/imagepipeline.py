from typing import Callable
import torch
import json_repair
import cv2
import numpy as np
import base64
import pyrealsense2 as rs
from PIL import Image
from segment_anything import sam_model_registry, SamPredictor
from cutie.inference.inference_core import InferenceCore
from cutie.utils.get_default_model import get_default_model
from torchvision.transforms.functional import to_tensor
from io import BytesIO

from utils import visualize, compute_mask_centers,visualize_centers
from model_adapters import QwenVLAdapter

import sys, time            # main_rs(frame_callback) 每帧回调 centers
# from imagepipeline import main_rs   # imagepipeline返回 cx, cy，
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# —————— 1. 配置区 ——————
# 串口 & 舵机 ID
PAN_PORT,  BAUD_PAN  = '/dev/ttyUSB1', 1_000_000
TILT_PORT, BAUD_TILT = '/dev/ttyUSB0', 57_600
PROTOCOL_VER         = 2.0
ID_PAN, ID_TILT      = 1, 1

# Control Table 地址 & 模式
ADDR_OP_MODE       = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POS      = 116
ADDR_PRESENT_POS   = 132
OPER_MODE_POS      = 3
TORQUE_ON, TORQUE_OFF = 1, 0

# 零点脉冲 & 初始角度
PAN_CENTER_TICK,   TILT_CENTER_TICK= 1753, 2308
pan_angle_current, tilt_angle_current = 0.0, 0.0  # “中心”对应 0°,跟踪状态

# 相机分辨率 & 视场角
IMAGE_WIDTH, IMAGE_HEIGHT = 1280, 720
H_FOV_DEG,  V_FOV_DEG     = 86.0, 57.0
# IMAGE_WIDTH, IMAGE_HEIGHT = 1280, 720
# H_FOV_DEG,  V_FOV_DEG     = 70.0, 40.0

pre_pose_x = 0
pre_pose_y = 0
# 比例增益（0 < Kp ≤ 1）
Kp_pan, Kp_tilt = 0.6, 0.6

# —————— 2. 初始化各自端口 ——————
def init_port(dev, baud):
    ph = PortHandler(dev);              #ph串口句柄
    pk = PacketHandler(PROTOCOL_VER)    #pk协议句柄
    if not ph.openPort():   
        sys.exit(f"无法打开串口 {dev}")
    if not ph.setBaudRate(baud): 
        sys.exit(f"无法设置波特 {baud} on {dev}")
    return ph, pk

ph_pan, pk_pan = None, None
ph_tilt, pk_tilt = None, None

def setup_all():
    global ph_pan, pk_pan, ph_tilt, pk_tilt
    ph_pan,  pk_pan  = init_port(PAN_PORT,  BAUD_PAN)
    ph_tilt, pk_tilt = init_port(TILT_PORT, BAUD_TILT)
    
    # 切位置模式 & 使能扭矩
    pk_pan.write1ByteTxRx(ph_pan, ID_PAN,   ADDR_OP_MODE,   OPER_MODE_POS)
    pk_pan.write1ByteTxRx(ph_tilt, ID_TILT,   ADDR_TORQUE_ENABLE, TORQUE_ON)
    pk_tilt.write1ByteTxRx(ph_pan, ID_PAN, ADDR_OP_MODE,  OPER_MODE_POS)
    pk_tilt.write1ByteTxRx(ph_tilt, ID_TILT, ADDR_TORQUE_ENABLE, TORQUE_ON)
    print("SET ALL OK!!")

def shutdown_all():
    # 关扭矩 & 关串口
    if pk_pan:  pk_pan.write1ByteTxRx(ph_pan,  ID_PAN,   ADDR_TORQUE_ENABLE, TORQUE_OFF)
    if pk_tilt: pk_tilt.write1ByteTxRx(ph_tilt, ID_TILT, ADDR_TORQUE_ENABLE, TORQUE_OFF)
    if ph_pan:  ph_pan.closePort()
    if ph_tilt: ph_tilt.closePort()

# —————— 3. 角度→脉冲映射 ——————
def deg2tick(angle_deg, center_tick):
    # angle = max(-180.0, min(180.0, angle_deg))
    print(f"diff_angle: {angle_deg}")
    return int(center_tick + angle_deg * 4096.0/360.0)

def pixel_to_error(cx, cy):
    """
    返回相对画面中心的角度误差 (pan_err, tilt_err)
    右偏 & 下偏 为正
    """
    dx = cx - IMAGE_WIDTH/2
    dy = cy - IMAGE_HEIGHT/2
    pan_err  = -dx / (IMAGE_WIDTH/2)  * (H_FOV_DEG/2)
    tilt_err = dy/ (IMAGE_HEIGHT/2) * (V_FOV_DEG/2)
    # global pre_pose_x, pre_pose_y
    # if pre_pose_x == 0:
    #     pre_pose_x = cx
    #     pre_pose_y = cy
    #     return 0, 0
    # else:
    #     err_x = cx - pre_pose_x
    #     err_y = cy - pre_pose_y
    #     pre_pose_x = cx
    #     pre_pose_y = cy
    #     return err_x, err_y
    return pan_err, tilt_err

# —————— 4. 回d调 & 控制函数 ——————
def servo_upate(centers):
    """
    这个函数会被 main_rs 在每帧调用。
    centers: list of (x,y)，我们取 centers[0]
    """
    global pan_angle_current, tilt_angle_current

    if not centers:
        return
    cx, cy = centers[0]

    # 1) 计算角度误差
    pan_err, tilt_err = pixel_to_error(cx, cy)
    print(f"x_gre_error: {pan_err}")
    print(f"y_gre_error: {tilt_err}")
    # 2) P 控制：更新目标角度
    pan_angle_current  += Kp_pan  * pan_err
    tilt_angle_current += Kp_tilt * tilt_err

    print(f"x_gre_error: {pan_angle_current}")
    print(f"y_gre_error: {tilt_angle_current}")
    
    # 3) 限幅
    pan_angle_current  = max(-180.0, min(180.0, pan_angle_current))
    tilt_angle_current = max(-180.0, min(180.0, tilt_angle_current))

    # 4) 映射 & 下发
    pan_tick  = deg2tick(pan_angle_current,  PAN_CENTER_TICK)
    tilt_tick = deg2tick(tilt_angle_current, TILT_CENTER_TICK)

    print(f"pan_tick: {pan_tick}")

    pk_pan.write4ByteTxRx(ph_pan,  ID_PAN,  ADDR_GOAL_POS, pan_tick)
    pk_tilt.write4ByteTxRx(ph_tilt, ID_TILT, ADDR_GOAL_POS, tilt_tick)

    # （可选）调试
    print(f"px({cx:.0f},{cy:.0f}) err°({pan_err:.2f},{tilt_err:.2f}) → ang°({pan_angle_current:.2f},{tilt_angle_current:.2f})")


class ImagePipeline:
    def __init__(self, sam_checkpoint: str, sam_model_type: str, cutie_model, vl_adapter):
        """
        多目标分割与跟踪管道（集成视觉语言模型）
        
        参数:
        sam_checkpoint: SAM模型权重路径
        sam_model_type: SAM模型类型 (e.g. "vit_h")
        cutie_model: 预加载的CUTIE模型实例
        vl_adapter: 视觉语言模型适配器实例（QwenVLAdapter）
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # 初始化SAM
        self.sam = sam_model_registry[sam_model_type](checkpoint=sam_checkpoint)
        self.sam.to(self.device)
        self.predictor = SamPredictor(self.sam)
        
        # 初始化CUTIE
        self.cutie = cutie_model
        self.processor = InferenceCore(self.cutie, cfg=self.cutie.cfg)
        self.processor.max_internal_size = -1
        
        # 集成视觉语言模型
        self.vl_adapter = vl_adapter
        
        # 状态跟踪
        self.current_objects = []
        self.is_initialized = False
        
    def _image_to_base64(self, image: np.ndarray) -> str:
        """将numpy图像转换为base64编码字符串"""
        pil_img = Image.fromarray(image)
        buffered = BytesIO()
        pil_img.save(buffered, format="JPEG")
        return base64.b64encode(buffered.getvalue()).decode("utf-8")

    def get_bbox_from_vl(self, frame: np.ndarray, instruction: str) -> list:
        """
        使用视觉语言模型生成目标边界框
        
        参数:
        frame: RGB格式的输入图像 [H, W, 3]
        instruction: 自然语言指令（如"the red cup on the left"）
        
        返回:
        bboxes: 边界框列表 [[x1,y1,x2,y2], ...]
        """
        # 转换图像格式
        base64_image = self._image_to_base64(frame)
        
        # 构建VL模型输入
        prompt = (
            f"Analyze the image and identify ALL objects matching: {instruction}.\n"
            "Return bboxes for ALL matching objects in this format:\n"
            "[{\"bbox_2d\": [x1,y1,x2,y2], \"label\": \"...\"}, ...]"
        )
        
        print(f"vl user prompt:\n{prompt}")
        
        # 生成响应
        input_data = self.vl_adapter.prepare_input(
            text=prompt,
            image_url=f"data:image/jpeg;base64,{base64_image}"
        )
        response, _ = self.vl_adapter.generate_response(input_data, max_tokens=512)
        
        print(f"response from vl: {response}")
        
        # 解析响应
        try:
            # 提取JSON部分
            json_str = response[response.find("["):response.rfind("]")+1]
            bbox_list = json_repair.loads(json_str)
            
            # 验证bbox格式
            valid_bboxes = []
            for item in bbox_list:
                bbox = item.get("bbox_2d", [])
                if len(bbox) == 4 and all(0 <= v <= frame.shape[1] if i%2==0 else 0 <= v <= frame.shape[0] for i,v in enumerate(bbox)):
                    valid_bboxes.append(bbox)
            return valid_bboxes
        except Exception as e:
            raise RuntimeError(f"Failed to parse VL model response: {str(e)}")

    def initialize_with_instruction(self, frame: np.ndarray, instruction: str, return_bbox: bool = False) -> tuple[np.ndarray,list|None]:
        """
        端到端初始化流程：VL生成bbox -> SAM分割 -> CUTIE初始化
        
        参数:
        frame: RGB格式的输入图像
        instruction: 自然语言指令
        
        返回:
        combined_mask: 组合后的多目标mask
        """
        # Step 1: 通过VL模型获取bbox
        bboxes = self.get_bbox_from_vl(frame, instruction)
        if not bboxes:
            raise ValueError("No valid bounding boxes detected by VL model")
        
        # Step 2: SAM生成mask
        return self.initialize_masks(frame, bboxes), None if not return_bbox else bboxes
    def initialize_masks(self, frame: np.ndarray, bboxes: list) -> np.ndarray:
        """
        初始化多目标分割
        
        参数:
        frame: RGB格式的输入图像 [H, W, 3]
        bboxes: 多个目标的边界框列表 [[x1, y1, x2, y2], ...]
        
        返回:
        combined_mask: 组合后的多目标mask，每个目标用不同整数ID表示
        """
        # 转换颜色空间并设置SAM图像
        rgb_frame = frame
        self.predictor.set_image(rgb_frame)
        
        # 生成并组合多个目标的mask
        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        object_ids = []
        for obj_idx, bbox in enumerate(bboxes):
            # SAM预测最佳mask
            masks, scores, _ = self.predictor.predict(
                box=np.array(bbox),
                multimask_output=True
            )
            best_mask = masks[np.argmax(scores)]
            
            # 分配唯一对象ID (从1开始)
            obj_id = obj_idx + 1
            combined_mask[best_mask] = obj_id
            object_ids.append(obj_id)
        
        # 初始化CUTIE处理器
        mask_tensor = torch.from_numpy(combined_mask).to(self.device)
        self.processor.clear_memory()
        self.processor.step(to_tensor(rgb_frame).to(self.device), mask_tensor, object_ids)
        
        # 更新状态
        self.current_objects = object_ids
        self.is_initialized = True
        
        return combined_mask

    def update_masks(self, frame: np.ndarray) -> tuple[np.ndarray, list]:
        """
        更新多目标跟踪结果
        
        参数:
        frame: RGB格式的新帧 [H, W, 3]
        
        返回:
        list: 每个目标的二值mask列表 [mask1, mask2, ...]
        """
        if not self.is_initialized:
            raise RuntimeError("Pipeline not initialized. Call initialize_masks first.")
        
        # 准备输入数据
        rgb_frame = frame
        image_tensor = to_tensor(rgb_frame).to(self.device)
        
        # CUTIE推理
        with torch.no_grad():
            output_prob = self.processor.step(image_tensor)
            current_mask = self.processor.output_prob_to_mask(output_prob)
            current_mask_np = current_mask.cpu().numpy().astype(np.uint8)
        
        # 分离各个目标的mask
        return current_mask_np,[(current_mask_np == obj_id) for obj_id in self.current_objects]

    def reset(self):
        """重置管道状态"""
        self.processor.clear_memory()
        self.current_objects = []
        self.is_initialized = False

    def add_object(self, frame: np.ndarray, bbox: list) -> np.ndarray:
        """
        动态添加新目标到现有跟踪
        
        参数:
        frame: RGB格式的当前帧
        bbox: 新目标的边界框 [x1, y1, x2, y2]
        
        返回:
        new_mask: 新目标的单独mask
        """
        # 生成新目标mask
        rgb_frame = frame[..., ::-1]
        self.predictor.set_image(rgb_frame)
        masks, scores, _ = self.predictor.predict(box=np.array(bbox), multimask_output=True)
        new_mask = masks[np.argmax(scores)]
        
        # 分配新ID
        new_id = max(self.current_objects) + 1 if self.current_objects else 1
        new_mask_tensor = torch.from_numpy(new_mask.astype(np.uint8) * new_id).to(self.device)
        
        # 合并到现有mask
        combined_mask = self.processor.output_prob_to_mask(self.processor.prob)
        combined_mask = torch.where(new_mask_tensor > 0, new_mask_tensor, combined_mask)
        
        # 更新处理器状态
        self.current_objects.append(new_id)
        self.processor.step(image_tensor, combined_mask, self.current_objects)
        
        return new_mask

class RealsenseCamera:
    # TODO havent been tested
    def __init__(self):
        device_id = "203522250675"
        self.pipeline = rs.pipeline()  # type: ignore
        config = rs.config()  # type: ignore
        print(f'[RealsenseCamera]: config: {config}')
        config.enable_device(device_id)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # type: ignore
        
    def __enter__(self):
        self.pipeline.start()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.pipeline.stop()
        
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())
        return frame

def main_rs(frame_callback:Callable|None):
    # 初始化所有模型
    sam_checkpoint = "/data/shiqi/ImagePipelien/sam_vit_h_4b8939.pth"
    sam_model_type = "vit_h"
    vl_adapter = QwenVLAdapter(model_path="/data/model/Qwen/Qwen2.5-VL-3B-Instruct")
    
    with torch.no_grad(), RealsenseCamera() as camera:
        cutie_model = get_default_model()
        pipeline = ImagePipeline(sam_checkpoint, sam_model_type, cutie_model, vl_adapter)
        
        # 初始化标志
        initialized = False
        
        while True:
            frame = camera.get_frame()
            rgb_frame = frame[:,:,::-1].copy()  # BGR转RGB
            
            if not initialized:
                # 使用第一帧进行初始化
                combined_mask, bboxes = pipeline.initialize_with_instruction(
                    frame=frame,
                    instruction="lemon",
                    return_bbox=True
                )
                initialized = True
                centers = compute_mask_centers(combined_mask, 'centroid')
            else:
                # 更新掩码
                updated_mask, _ = pipeline.update_masks(rgb_frame)
                centers = compute_mask_centers(updated_mask, 'centroid')
                
            # 实时可视化
            vis_frame = frame.copy()
            visualize(vis_frame, bboxes=bboxes if not initialized else None, mask=updated_mask if not initialized else None)
            visualize_centers(vis_frame, centers=centers)
            frame_callback(centers) if frame_callback else None
            # # # 按ESC退出
            # if cv2.waitKey(1) & 0xFF == 27:
            #     break

    cv2.destroyAllWindows()    
    
    
def main_rs_iter():
    print("start!")
    # 初始化所有模型
    sam_checkpoint = "/data/shiqi/ImagePipelien/sam_vit_h_4b8939.pth"
    sam_model_type = "vit_h"
    
    with torch.no_grad(), RealsenseCamera() as camera:
        frame = camera.get_frame()
        print(frame.shape)
        cutie_model = get_default_model()
        vl_adapter = QwenVLAdapter(model_path="/data/model/Qwen/Qwen2.5-VL-3B-Instruct")
        pipeline = ImagePipeline(sam_checkpoint, sam_model_type, cutie_model, vl_adapter)
        
        # 初始化标志
        initialized = False
        
        while True:
            frame = camera.get_frame()
            rgb_frame = frame[:,:,::-1].copy()  # BGR转RGB
            
            if not initialized:
                # 使用第一帧进行初始化
                combined_mask, bboxes = pipeline.initialize_with_instruction(
                    frame=frame,
                    instruction="Can",
                    return_bbox=True
                )
                initialized = True
                centers = compute_mask_centers(combined_mask, 'centroid')
                updated_mask = combined_mask
            else:
                # 更新掩码
                updated_mask, _ = pipeline.update_masks(rgb_frame)
                centers = compute_mask_centers(updated_mask, 'centroid')
            
            servo_upate(centers)
            # 实时可视化
            vis_frame = frame.copy()
            visualize(vis_frame, bboxes=bboxes if not initialized else None, mask=updated_mask)
            visualize_centers(vis_frame, centers=centers)
            # frame_callback(centers) if frame_callback else None
            # 按ESC退出
            # if cv2.waitKey(1) & 0xFF == 27:
            #     break
            yield centers
    cv2.destroyAllWindows()    
    
def main_demo():
    # 初始化所有模型
    sam_checkpoint = "/data/shiqi/ImagePipelien/sam_vit_h_4b8939.pth"
    sam_model_type = "vit_h"
    vl_adapter = QwenVLAdapter(model_path="/data/model/Qwen/Qwen2.5-VL-3B-Instruct")
    with torch.no_grad():
        cutie_model = get_default_model()
        # 创建增强版管道
        pipeline = ImagePipeline(sam_checkpoint, sam_model_type, cutie_model, vl_adapter)
        
        # 通过自然语言指令初始化
        frame = cv2.imread("erqa.png")
        rgb_frame=  frame[:,:,::-1].copy()
        combined_mask, bboxes = pipeline.initialize_with_instruction(
            frame=frame,
            instruction="lemon",  # 自然语言指令
            return_bbox=True
        )
        
        # 可视化结果
        visualize(frame,bboxes=bboxes, mask=combined_mask)
        centers = compute_mask_centers(combined_mask,'centroid')
        visualize_centers(frame, centers=centers)
        # 后续跟踪流程...
        for _ in range(10):
            new_frame = rgb_frame  # 获取新帧
            updated_mask, _ = pipeline.update_masks(new_frame)
            centers = compute_mask_centers(updated_mask,'centroid')
            visualize(new_frame, mask=updated_mask)
            visualize_centers(new_frame, centers=centers)
    
# 使用示例
if __name__ == "__main__":
    setup_all()
    try:
        for centers in main_rs_iter():
            print(centers)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_all()