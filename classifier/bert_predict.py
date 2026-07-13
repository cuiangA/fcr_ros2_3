"""
bert_predict.py — BERT 意图分类交互式校验工具

加载微调好的 BERT 模型，进行交互式或单次预测。
效果对齐 predict.py（SVM 版本），同样的可视化 + 轮询。

用法：
  # 默认加载 bert_intent_output/ 下的模型
  python bert_predict.py

  # 指定模型目录
  python bert_predict.py --model-dir bert_intent_output

  # 单次预测
  python bert_predict.py --text "跟上我"

  # 显示 top-3
  python bert_predict.py --topk 3

  # 强制 CPU（服务器有 GPU 但仍可用 CPU）
  python bert_predict.py --cpu

交互模式按键：
  输入文本 → 回车 → 查看结果
  :q 或 :quit → 退出
  Ctrl+C → 退出

依赖：
  pip install transformers torch
"""

import os
import sys
import argparse
import logging
from pathlib import Path

# Windows 终端 UTF-8 兼容
if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

# 国内镜像
if not os.environ.get("HF_ENDPOINT"):
    os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"

import torch
import torch.nn.functional as F
from transformers import AutoTokenizer, AutoModelForSequenceClassification

# ── 配置 ──────────────────────────────────────────────────────────────

DEFAULT_MODEL_DIR = "bert_intent_output"
MAX_LEN = 48

INTENT_NAMES = {
    0: "开始跟随",
    1: "停止跟随",
    2: "切换目标",
    3: "距离调整",
    4: "云台回中",
    5: "状态查询",
    6: "干扰项",
}

INTENT_NAMES_EN = {
    0: "start_following",
    1: "stop_following",
    2: "switch_target",
    3: "distance_adjust",
    4: "gimbal_home",
    5: "status_query",
    6: "chat",
}

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════════
# BERTPredictor
# ══════════════════════════════════════════════════════════════════════

class BERTPredictor:
    """加载微调好的 BERT 模型，进行意图预测"""

    def __init__(self, model_dir: str = DEFAULT_MODEL_DIR, cpu: bool = False):
        self.model_dir = Path(model_dir)
        self.device = torch.device("cpu" if cpu else ("cuda" if torch.cuda.is_available() else "cpu"))
        self.tokenizer = None
        self.model = None

    def load(self):
        """加载 tokenizer 和模型"""
        if not self.model_dir.exists():
            raise FileNotFoundError(
                f"BERT 模型目录不存在: {self.model_dir}\n"
                f"请先运行 bert_finetune.py 训练模型。"
            )

        logger.info("加载 tokenizer: %s ...", self.model_dir)
        self.tokenizer = AutoTokenizer.from_pretrained(str(self.model_dir))

        logger.info("加载 BERT 模型: %s ...", self.model_dir)
        self.model = AutoModelForSequenceClassification.from_pretrained(
            str(self.model_dir)
        ).to(self.device)
        self.model.eval()

        logger.info("BERT 模型加载完成 ✓  (device=%s)", self.device)
        logger.info("类别数: %d", self.model.config.num_labels)

        # 预热：跑一次空推理，触发 CUDA kernel 编译
        if self.device.type == "cuda":
            _ = self._inference("预热")

    def _inference(self, text: str) -> torch.Tensor:
        """单条文本 → softmax 概率向量 (on CPU)"""
        enc = self.tokenizer(
            text,
            padding="max_length",
            truncation=True,
            max_length=MAX_LEN,
            return_tensors="pt",
        )
        enc = {k: v.to(self.device) for k, v in enc.items()}

        with torch.no_grad():
            logits = self.model(**enc).logits.squeeze(0)
            probs = F.softmax(logits, dim=-1)

        return probs.cpu()

    def predict(self, text: str, topk: int = 3) -> dict:
        """
        预测单条文本。

        返回字段：
          predictions: [(中文名, 英文名, 置信度), ...]  按置信度降序
          confusion:   困惑度 C = (1-top1) x (top2/top1)
          margin:      1-2 名置信度差距
          top1_conf:   第一名置信度
          top2_conf:   第二名置信度
        """
        if self.model is None:
            raise RuntimeError("模型未加载，请先调用 load()")

        probs = self._inference(text)  # shape: (n_classes,)

        # 构建 (label_id, conf) 列表并按置信度降序
        pairs = []
        for label_id, conf in enumerate(probs.tolist()):
            pairs.append((
                INTENT_NAMES.get(label_id, f"class_{label_id}"),
                INTENT_NAMES_EN.get(label_id, f"class_{label_id}"),
                conf,
            ))
        pairs.sort(key=lambda x: x[2], reverse=True)

        top1_conf = pairs[0][2]
        top2_conf = pairs[1][2] if len(pairs) > 1 else 0.0
        margin = top1_conf - top2_conf
        confusion = (1.0 - top1_conf) * (top2_conf / top1_conf) if top1_conf > 0 else 1.0

        return {
            "predictions": pairs[:topk],
            "confusion": confusion,
            "margin": margin,
            "top1_conf": top1_conf,
            "top2_conf": top2_conf,
        }


# ══════════════════════════════════════════════════════════════════════
# 输出格式化（与 predict.py 完全一致）
# ══════════════════════════════════════════════════════════════════════

GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"

def _bar(conf: float, width: int = 30) -> str:
    filled = int(conf * width)
    if conf >= 0.7:
        color = GREEN
    elif conf >= 0.4:
        color = YELLOW
    else:
        color = RED
    return f"{color}{'#' * filled}{'-' * (width - filled)}{RESET}"


def _confusion_level(confusion: float) -> str:
    if confusion >= 0.15:
        return f"{RED}高困惑{RESET}"
    elif confusion >= 0.05:
        return f"{YELLOW}中等困惑{RESET}"
    else:
        return f"{GREEN}低困惑{RESET}"


def _margin_bar(margin: float, width: int = 15) -> str:
    filled = min(int(margin * 15), width)
    if margin >= 0.3:
        color = GREEN
    elif margin >= 0.1:
        color = YELLOW
    else:
        color = RED
    return f"{color}{'|' * max(filled, 1)}{'-' * (width - max(filled, 1))}{RESET}"


def print_result(text: str, result: dict, topk: int):
    """格式化打印预测结果"""
    predictions = result["predictions"]
    confusion = result["confusion"]
    margin = result["margin"]
    top1_conf = result["top1_conf"]
    top2_conf = result["top2_conf"]

    print()
    print(f"  {BOLD}输入:{RESET} {text}")
    print(f"  {'─' * 60}")

    top = predictions[0]
    print(f"  {BOLD}Top-1:{RESET}  {GREEN}{top[0]:<10s}{RESET}  "
          f"({top[1]:<18s})  置信度 {BOLD}{top[2]:.4f}{RESET}  {_bar(top[2])}")

    if topk > 1 and len(predictions) > 1:
        print(f"  {DIM}{'─' * 60}{RESET}")
        for i, (cn, en, conf) in enumerate(predictions[1:], start=2):
            marker = ""
            if conf >= 0.1:
                marker = f" {DIM}<- 可能混淆{RESET}"
            print(f"  {DIM}Top-{i}:{RESET}  {DIM}{cn:<10s}  "
                  f"({en:<18s})  置信度 {conf:.4f}{marker}{RESET}")

    # 困惑度指标
    print(f"  {'─' * 60}")
    print(f"  {BOLD}困惑度:   {RESET} {confusion:.4f}  {_confusion_level(confusion)}")
    print(f"  {BOLD}1-2差距:  {RESET} {margin:.4f}  {_margin_bar(margin)}")
    print(f"  {BOLD}公式:     {RESET} {DIM}C = (1-{top1_conf:.2f}) x ({top2_conf:.2f}/{top1_conf:.2f}){RESET}")

    print()


def print_legend():
    """打印各类别图例"""
    print(f"  {BOLD}类别图例:{RESET}")
    for label_id in sorted(INTENT_NAMES.keys()):
        cn = INTENT_NAMES[label_id]
        en = INTENT_NAMES_EN[label_id]
        print(f"    {label_id}  {cn:<10s} ({en})")
    print()
    print(f"  {DIM}输入文本后回车查看结果，输入 :q 退出，Ctrl+C 也行{RESET}")
    print()


# ══════════════════════════════════════════════════════════════════════
# 入口
# ══════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="BERT 意图分类交互式校验工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--model-dir", default=DEFAULT_MODEL_DIR,
                        help="BERT 模型目录")
    parser.add_argument("--text", default=None,
                        help="单次预测（不进入交互模式）")
    parser.add_argument("--topk", type=int, default=3,
                        help="显示 top-k 个结果")
    parser.add_argument("--cpu", action="store_true",
                        help="强制使用 CPU 推理")
    args = parser.parse_args()

    # ── 加载模型 ───────────────────────────────────────────
    predictor = BERTPredictor(model_dir=args.model_dir, cpu=args.cpu)
    predictor.load()

    # ── 单次模式 ───────────────────────────────────────────
    if args.text:
        result = predictor.predict(args.text, topk=args.topk)
        print_result(args.text, result, args.topk)
        return

    # ── 交互模式 ───────────────────────────────────────────
    print_legend()

    while True:
        try:
            user_input = input(f"{BOLD}>>> {RESET}").strip()
        except (EOFError, KeyboardInterrupt):
            print(f"\n{DIM}退出{RESET}")
            break

        if not user_input:
            continue

        if user_input in (":q", ":quit", ":exit"):
            print(f"{DIM}退出{RESET}")
            break

        result = predictor.predict(user_input, topk=args.topk)
        print_result(user_input, result, args.topk)


if __name__ == "__main__":
    main()
