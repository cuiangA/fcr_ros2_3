"""Console test node for the two-level intent classifier."""

import json
import queue
import sys
import threading
from pathlib import Path
from typing import Optional

import rclpy
from external_control_pkg.msg import VoiceCommand
from rclpy.node import Node
from std_msgs.msg import String

from voice_intent_pkg.double_layer_classifier import DoubleLayerClassifier


CONTROL_INTENTS = {
    ("移动指令", "向前移动"): "chassis_move_forward",
    ("移动指令", "向后移动"): "chassis_move_backward",
    ("移动指令", "向左移动"): "chassis_move_left",
    ("移动指令", "向右移动"): "chassis_move_right",
    ("移动指令", "调整参数"): "chassis_adjust_parameter",
    ("视野朝向指令", "向上看"): "gimbal_nudge_up",
    ("视野朝向指令", "向下看"): "gimbal_nudge_down",
    ("视野朝向指令", "向左看"): "gimbal_nudge_left",
    ("视野朝向指令", "向右看"): "gimbal_nudge_right",
    ("视野朝向指令", "调整参数"): "gimbal_adjust_parameter",
    ("视野朝向指令", "云台回中"): "gimbal_home",
    ("运镜模式指令", "切换目标"): "switch_target",
    ("运镜模式指令", "跟随模式"): "start_following",
    ("运镜模式指令", "环绕模式"): "start_orbit",
    ("运镜模式指令", "拉镜头模式"): "start_dolly",
    ("录制指令", "照相"): "camera_take_photo",
    ("录制指令", "开始录制"): "camera_start_recording",
    ("停止指令", "停止"): "stop_current_action",
    ("状态查询指令", "查询底盘状态"): "query_chassis_status",
    ("状态查询指令", "查询云台状态"): "query_gimbal_status",
    ("状态查询指令", "查询摄像机状态"): "query_camera_status",
    ("状态查询指令", "查询运镜状态"): "query_camera_motion_status",
}


class DoubleLayerConsoleVoiceNode(Node):
    def __init__(self) -> None:
        super().__init__("double_layer_console_voice_node")
        self.declare_parameter("model_root", "")
        self.declare_parameter("embedding_model_dir", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("min_coarse_confidence", 0.60)
        self.declare_parameter("max_fine_confusion", 0.05)
        self.declare_parameter("voice_command_topic", "/external/voice_command")
        self.declare_parameter("result_topic", "/external/intent_result")

        model_root = Path(str(self.get_parameter("model_root").value)).expanduser()
        embedding_dir = Path(
            str(self.get_parameter("embedding_model_dir").value)
        ).expanduser()
        self._validate_paths(model_root, embedding_dir)
        self._min_coarse_confidence = float(
            self.get_parameter("min_coarse_confidence").value
        )

        self.get_logger().info(f"正在加载双层意图模型: {model_root}")
        self._classifier = DoubleLayerClassifier(
            model_root=model_root,
            embedding_model_dir=embedding_dir,
            device_name=str(self.get_parameter("device").value),
            confusion_threshold=float(
                self.get_parameter("max_fine_confusion").value
            ),
        )
        self._classifier.load()

        self._voice_pub = self.create_publisher(
            VoiceCommand,
            str(self.get_parameter("voice_command_topic").value),
            10,
        )
        self._result_pub = self.create_publisher(
            String, str(self.get_parameter("result_topic").value), 10
        )
        self._queue: "queue.Queue[Optional[str]]" = queue.Queue()
        self._thread = threading.Thread(target=self._console_loop, daemon=True)
        self._thread.start()
        self.create_timer(0.05, self._consume_input)
        self.get_logger().info(
            "双层意图控制台已就绪。输入中文后回车，输入 :q 退出。"
        )

    @staticmethod
    def _validate_paths(model_root: Path, embedding_dir: Path) -> None:
        required = (
            model_root / "bert_intent_output" / "config.json",
            model_root / "bert_intent_output" / "model.safetensors",
            model_root / "council",
            embedding_dir / "config.json",
        )
        missing = [str(path) for path in required if not path.exists()]
        if missing:
            raise RuntimeError("模型文件不完整，缺少: " + ", ".join(missing))

    def _console_loop(self) -> None:
        while rclpy.ok():
            sys.stdout.write("INTENT> ")
            sys.stdout.flush()
            raw = sys.stdin.buffer.readline()
            if not raw:
                self._queue.put(None)
                return
            text = None
            for encoding in ("utf-8", "gb18030"):
                try:
                    text = raw.decode(encoding).strip()
                    break
                except UnicodeDecodeError:
                    continue
            if text is None:
                self.get_logger().warning("输入无法按 UTF-8 或 GB18030 解码，已忽略")
            elif text in {":q", ":quit", ":exit"}:
                self._queue.put(None)
                return
            elif text:
                self._queue.put(text)

    def _consume_input(self) -> None:
        try:
            text = self._queue.get_nowait()
        except queue.Empty:
            return
        if text is None:
            rclpy.shutdown()
            return
        try:
            self._classify_and_publish(text)
        except Exception as exc:
            self.get_logger().error(f"意图推理失败: {exc}")

    def _classify_and_publish(self, text: str) -> None:
        result = self._classifier.predict(text)
        coarse = result["coarse"]
        fine = result["fine"]
        control_intent = CONTROL_INTENTS.get((coarse["name"], fine["name"]), "")
        accepted = (
            bool(control_intent)
            and coarse["confidence"] >= self._min_coarse_confidence
            and bool(fine["passed"])
            and coarse["name"] != "干扰项"
        )

        self.get_logger().info(
            f'[MODEL] text="{text}" | coarse={coarse["name"]} '
            f'({coarse["confidence"]:.4f}) | fine={fine["name"]} '
            f'({fine["confidence"]:.4f}, confusion={fine["confusion"]:.4f}, '
            f'decision={fine["decision"]})'
        )
        self.get_logger().info(
            f"[CONTROL] intent={control_intent or 'none'} | "
            f"published={'true' if accepted else 'false'}"
        )

        if accepted:
            message = VoiceCommand()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "double_layer_console"
            message.intents = [control_intent]
            message.confidences = [
                float(min(coarse["confidence"], fine["confidence"]))
            ]
            message.raw_text = text
            message.distance = -1.0
            message.unit = ""
            message.speed = ""
            message.target_desc = ""
            message.follow = control_intent == "start_following"
            self._voice_pub.publish(message)

        diagnostic = dict(result)
        diagnostic["control_intent"] = control_intent
        diagnostic["published"] = accepted
        output = String()
        output.data = json.dumps(diagnostic, ensure_ascii=False)
        self._result_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DoubleLayerConsoleVoiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
