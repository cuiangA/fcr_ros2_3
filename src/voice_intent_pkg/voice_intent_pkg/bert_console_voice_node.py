"""Console text -> BERT result -> VoiceCommand ROS 2 test node."""

import json
import os
import queue
import sys
import threading
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from external_control_pkg.msg import VoiceCommand
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from voice_intent_pkg.intent_rules import resolve_gimbal_intent


DEFAULT_LABEL_INTENTS = [
    "start_following",
    "stop_following",
    "switch_target",
    "distance_adjust",
    "gimbal_home",
    "status_query",
    "chat",
]


class BertConsoleVoiceNode(Node):
    """Run local BERT inference for console text and publish VoiceCommand."""

    def __init__(self) -> None:
        super().__init__("bert_console_voice_node")

        self.declare_parameter(
            "model_dir",
            "",
            ParameterDescriptor(
                description="Local Hugging Face model directory; no network is used",
            ),
        )
        self.declare_parameter(
            "device",
            "cpu",
            ParameterDescriptor(description="Inference device: cpu, cuda, or auto"),
        )
        self.declare_parameter(
            "voice_command_topic",
            "/external/voice_command",
            ParameterDescriptor(description="VoiceCommand output topic"),
        )
        self.declare_parameter(
            "bert_result_topic",
            "/external/bert_result",
            ParameterDescriptor(description="JSON-formatted BERT diagnostic topic"),
        )
        self.declare_parameter(
            "min_bert_confidence",
            0.60,
            ParameterDescriptor(description="Minimum confidence for BERT intent output"),
        )
        self.declare_parameter(
            "min_bert_margin",
            0.10,
            ParameterDescriptor(description="Minimum top-1 minus top-2 confidence"),
        )
        self.declare_parameter(
            "enable_gimbal_rules",
            True,
            ParameterDescriptor(
                description=(
                    "Resolve gimbal direction commands not present in the current "
                    "seven-class BERT model"
                ),
            ),
        )
        self.declare_parameter(
            "label_intents",
            DEFAULT_LABEL_INTENTS,
            ParameterDescriptor(
                description="Class-index mapping used by the trained BERT head",
            ),
        )

        self._model_dir = self._resolve_model_dir(
            str(self.get_parameter("model_dir").value),
        )
        self._device_name = str(self.get_parameter("device").value)
        self._min_confidence = float(
            self.get_parameter("min_bert_confidence").value,
        )
        self._min_margin = float(self.get_parameter("min_bert_margin").value)
        self._enable_gimbal_rules = bool(
            self.get_parameter("enable_gimbal_rules").value,
        )
        self._label_intents = list(self.get_parameter("label_intents").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._voice_pub = self.create_publisher(
            VoiceCommand,
            str(self.get_parameter("voice_command_topic").value),
            qos,
        )
        self._result_pub = self.create_publisher(
            String,
            str(self.get_parameter("bert_result_topic").value),
            qos,
        )

        self._torch = None
        self._tokenizer = None
        self._model = None
        self._device = None
        self._load_model()

        self._input_queue: "queue.Queue[Optional[str]]" = queue.Queue()
        self._input_thread = threading.Thread(
            target=self._console_loop,
            name="bert_console_input",
            daemon=True,
        )
        self._input_thread.start()
        self._queue_timer = self.create_timer(0.05, self._process_console_input)

        output_topic = str(self.get_parameter("voice_command_topic").value)
        self.get_logger().info(
            f"BERT console ready | model={self._model_dir} | "
            f"device={self._device} | output={output_topic}",
        )
        self.get_logger().info(
            "输入自然语言后按回车；输入 :q 退出。BERT 原始结果和最终控制意图会分别输出。",
        )

    @staticmethod
    def _resolve_model_dir(parameter_value: str) -> Path:
        candidates: List[Path] = []
        if parameter_value:
            candidates.append(Path(parameter_value).expanduser())

        env_path = os.environ.get("FCR_BERT_MODEL_DIR", "")
        if env_path:
            candidates.append(Path(env_path).expanduser())

        candidates.append(Path.cwd() / "classifier" / "bert_intent_output")

        for candidate in candidates:
            resolved = candidate.resolve()
            if (resolved / "config.json").is_file() and (
                resolved / "model.safetensors"
            ).is_file():
                return resolved

        attempted = ", ".join(str(path) for path in candidates)
        raise RuntimeError(
            "BERT model directory was not found. Set model_dir or "
            f"FCR_BERT_MODEL_DIR. Attempted: {attempted}",
        )

    def _load_model(self) -> None:
        try:
            import torch
            from transformers import AutoModelForSequenceClassification
            from transformers import AutoTokenizer
        except ImportError as exc:
            raise RuntimeError(
                "Missing BERT runtime dependency. Verify that torch, transformers "
                "and safetensors are installed in the ROS 2 Python environment.",
            ) from exc

        if self._device_name not in {"cpu", "cuda", "auto"}:
            raise RuntimeError("device must be one of: cpu, cuda, auto")

        if self._device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device=cuda was requested but CUDA is unavailable")

        use_cuda = self._device_name == "cuda" or (
            self._device_name == "auto" and torch.cuda.is_available()
        )
        self._device = torch.device("cuda" if use_cuda else "cpu")
        self._torch = torch

        self.get_logger().info(f"正在加载 BERT 模型: {self._model_dir}")
        self._tokenizer = AutoTokenizer.from_pretrained(
            str(self._model_dir),
            local_files_only=True,
        )
        self._model = AutoModelForSequenceClassification.from_pretrained(
            str(self._model_dir),
            local_files_only=True,
        ).to(self._device)
        self._model.eval()

        class_count = int(self._model.config.num_labels)
        if class_count != len(self._label_intents):
            raise RuntimeError(
                "BERT class count does not match label_intents: "
                f"model={class_count}, mapping={len(self._label_intents)}",
            )

    def _console_loop(self) -> None:
        while rclpy.ok():
            try:
                sys.stdout.write("BERT> ")
                sys.stdout.flush()
                raw_input = sys.stdin.buffer.readline()
            except (EOFError, KeyboardInterrupt):
                self._input_queue.put(None)
                return

            if not raw_input:
                self._input_queue.put(None)
                return

            text = None
            decode_errors = []
            for encoding in ("utf-8", "gb18030"):
                try:
                    text = raw_input.decode(encoding).strip()
                    break
                except UnicodeDecodeError as exc:
                    decode_errors.append(f"{encoding}: {exc}")

            if text is None:
                self.get_logger().warning(
                    "终端输入无法按 UTF-8 或 GB18030 解码，已忽略本行: "
                    + "; ".join(decode_errors),
                )
                continue

            if text in {":q", ":quit", ":exit"}:
                self._input_queue.put(None)
                return
            if text:
                self._input_queue.put(text)

    def _process_console_input(self) -> None:
        try:
            text = self._input_queue.get_nowait()
        except queue.Empty:
            return

        if text is None:
            self.get_logger().info("收到退出命令")
            rclpy.shutdown()
            return

        try:
            self._classify_and_publish(text)
        except Exception as exc:  # Keep the test console alive after one bad input.
            self.get_logger().error(f"BERT 推理失败: {exc}")

    def _infer(self, text: str) -> Dict[str, object]:
        encoded = self._tokenizer(
            text,
            padding="max_length",
            truncation=True,
            max_length=48,
            return_tensors="pt",
        )
        encoded = {key: value.to(self._device) for key, value in encoded.items()}

        with self._torch.no_grad():
            logits = self._model(**encoded).logits.squeeze(0)
            probabilities = self._torch.softmax(logits, dim=-1)

        top_count = min(2, len(self._label_intents))
        top_values, top_indices = self._torch.topk(probabilities, k=top_count)
        top1_id = int(top_indices[0].item())
        top1_confidence = float(top_values[0].item())
        top2_confidence = float(top_values[1].item()) if top_count > 1 else 0.0
        return {
            "intent": self._label_intents[top1_id],
            "confidence": top1_confidence,
            "margin": top1_confidence - top2_confidence,
            "label_id": top1_id,
        }

    def _classify_and_publish(self, text: str) -> None:
        bert = self._infer(text)
        bert_intent = str(bert["intent"])
        confidence = float(bert["confidence"])
        margin = float(bert["margin"])

        self.get_logger().info(
            f'[BERT] text="{text}" | intent={bert_intent} | '
            f"confidence={confidence:.4f} | margin={margin:.4f}",
        )

        control_intent = None
        control_confidence = confidence
        control_source = "none"
        if self._enable_gimbal_rules:
            control_intent = resolve_gimbal_intent(text)
            if control_intent is not None:
                control_confidence = 1.0
                control_source = "gimbal_rule"

        if control_intent is None and bert_intent != "chat":
            if confidence >= self._min_confidence and margin >= self._min_margin:
                control_intent = bert_intent
                control_source = "bert"

        accepted = control_intent is not None
        if accepted:
            self._publish_voice_command(
                raw_text=text,
                intent=control_intent,
                confidence=control_confidence,
            )
            self.get_logger().info(
                f"[CONTROL] intent={control_intent} | "
                f"source={control_source} | published=true",
            )
        else:
            self.get_logger().warning(
                "[CONTROL] 未发布：chat、置信度不足或当前没有对应控制意图",
            )

        result = {
            "raw_text": text,
            "bert_intent": bert_intent,
            "bert_label_id": int(bert["label_id"]),
            "bert_confidence": confidence,
            "bert_margin": margin,
            "control_intent": control_intent or "",
            "control_source": control_source,
            "published": accepted,
        }
        result_msg = String()
        result_msg.data = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(result_msg)

    def _publish_voice_command(
        self,
        raw_text: str,
        intent: str,
        confidence: float,
    ) -> None:
        message = VoiceCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "bert_console"
        message.intents = [intent]
        message.confidences = [confidence]
        message.raw_text = raw_text
        message.distance = -1.0
        message.unit = ""
        message.speed = ""
        message.target_desc = ""
        message.follow = intent == "start_following"
        self._voice_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BertConsoleVoiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"bert_console_voice_node failed: {exc}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
