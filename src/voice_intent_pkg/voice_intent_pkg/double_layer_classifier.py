"""Local BERT coarse classifier plus SVM fine classifiers."""

from pathlib import Path
from typing import Dict, List

import joblib
import numpy as np


COARSE_LABELS = {
    0: "移动指令",
    1: "视野朝向指令",
    2: "运镜模式指令",
    3: "录制指令",
    4: "停止指令",
    5: "状态查询指令",
    6: "干扰项",
}

COARSE_TO_PREFIX = {
    0: "0_move_command",
    1: "1_vision_command",
    2: "2_camera_mode_command",
    3: "3_recording_command",
    4: "4_stop_command",
    5: "5_status_query_command",
    6: "6_distractor_command",
}

FINE_LABELS = {
    "0_move_command": ["向前移动", "向后移动", "向左移动", "向右移动", "调整参数"],
    "1_vision_command": ["向上看", "向下看", "向左看", "向右看", "调整参数", "云台回中"],
    "2_camera_mode_command": ["切换目标", "跟随模式", "环绕模式", "拉镜头模式"],
    "3_recording_command": ["照相", "开始录制"],
    "4_stop_command": ["停止"],
    "5_status_query_command": [
        "查询底盘状态",
        "查询云台状态",
        "查询摄像机状态",
        "查询运镜状态",
    ],
    "6_distractor_command": ["干扰项"],
}


class DoubleLayerClassifier:
    """Load and run the user-provided two-level classifier entirely locally."""

    def __init__(
        self,
        model_root: Path,
        embedding_model_dir: Path,
        device_name: str = "cpu",
        confusion_threshold: float = 0.05,
    ) -> None:
        self.model_root = model_root
        self.bert_dir = model_root / "bert_intent_output"
        self.council_dir = model_root / "council"
        self.embedding_model_dir = embedding_model_dir
        self.device_name = device_name
        self.confusion_threshold = confusion_threshold
        self._fine_models: Dict[str, Dict[str, object]] = {}

    def load(self) -> None:
        import torch
        from sentence_transformers import SentenceTransformer
        from transformers import AutoModelForSequenceClassification, AutoTokenizer

        if self.device_name not in {"cpu", "cuda", "auto"}:
            raise RuntimeError("device must be one of: cpu, cuda, auto")
        if self.device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device=cuda was requested but CUDA is unavailable")

        use_cuda = self.device_name == "cuda" or (
            self.device_name == "auto" and torch.cuda.is_available()
        )
        self.device = torch.device("cuda" if use_cuda else "cpu")
        self.torch = torch
        self.tokenizer = AutoTokenizer.from_pretrained(
            str(self.bert_dir), local_files_only=True
        )
        self.bert_model = AutoModelForSequenceClassification.from_pretrained(
            str(self.bert_dir), local_files_only=True
        ).to(self.device)
        self.bert_model.eval()

        self.embedder = SentenceTransformer(
            str(self.embedding_model_dir),
            device=str(self.device),
            local_files_only=True,
        )
        for prefix in COARSE_TO_PREFIX.values():
            directory = self.council_dir / prefix
            semantic_path = directory / "svm_sem.pkl"
            keyword_path = directory / "svm_kw.pkl"
            models: Dict[str, object] = {}
            if semantic_path.is_file():
                models["semantic"] = joblib.load(semantic_path)
            if keyword_path.is_file():
                vectorizer, classifier = joblib.load(keyword_path)
                models["vectorizer"] = vectorizer
                models["keyword"] = classifier
            self._fine_models[prefix] = models

    @staticmethod
    def _confusion(probabilities: np.ndarray) -> float:
        ordered = np.sort(probabilities)[::-1]
        top1 = float(ordered[0])
        top2 = float(ordered[1]) if len(ordered) > 1 else 0.0
        return 1.0 if top1 <= 0.0 else (1.0 - top1) * (top2 / top1)

    @staticmethod
    def _result(probabilities: np.ndarray, labels: List[str]) -> Dict[str, object]:
        prediction = int(np.argmax(probabilities))
        ordered = np.argsort(probabilities)[::-1]
        return {
            "id": prediction,
            "name": labels[prediction],
            "confidence": float(probabilities[prediction]),
            "confusion": DoubleLayerClassifier._confusion(probabilities),
            "top_k": [
                {
                    "id": int(index),
                    "name": labels[int(index)],
                    "confidence": float(probabilities[int(index)]),
                }
                for index in ordered[: min(3, len(ordered))]
            ],
        }

    def _predict_coarse(self, text: str) -> Dict[str, object]:
        encoded = self.tokenizer(
            text,
            padding="max_length",
            truncation=True,
            max_length=48,
            return_tensors="pt",
        )
        encoded = {key: value.to(self.device) for key, value in encoded.items()}
        with self.torch.no_grad():
            logits = self.bert_model(**encoded).logits.squeeze(0)
            probabilities = self.torch.softmax(logits, dim=-1).cpu().numpy()
        labels = [COARSE_LABELS[index] for index in range(len(COARSE_LABELS))]
        return self._result(probabilities, labels)

    def _predict_fine(self, coarse_id: int, text: str) -> Dict[str, object]:
        prefix = COARSE_TO_PREFIX[coarse_id]
        labels = FINE_LABELS[prefix]
        models = self._fine_models[prefix]
        if not models:
            return {
                "id": 0,
                "name": labels[0],
                "confidence": 1.0,
                "confusion": 0.0,
                "passed": True,
                "decision": "single_class",
                "top_k": [{"id": 0, "name": labels[0], "confidence": 1.0}],
            }

        semantic = None
        if "semantic" in models:
            embedding = self.embedder.encode(
                [text], normalize_embeddings=True, show_progress_bar=False
            )
            semantic = self._result(
                models["semantic"].predict_proba(embedding)[0], labels
            )

        keyword = None
        if "keyword" in models:
            features = models["vectorizer"].transform([text])
            keyword = self._result(
                models["keyword"].predict_proba(features)[0], labels
            )

        candidates = [
            (name, result)
            for name, result in (("semantic", semantic), ("keyword", keyword))
            if result is not None
        ]
        decision, selected = min(candidates, key=lambda item: item[1]["confusion"])
        selected = dict(selected)
        selected["passed"] = any(
            result["confusion"] <= self.confusion_threshold
            for _, result in candidates
        )
        selected["decision"] = decision
        selected["semantic"] = semantic
        selected["keyword"] = keyword
        return selected

    def predict(self, text: str) -> Dict[str, object]:
        coarse = self._predict_coarse(text.strip())
        fine = self._predict_fine(int(coarse["id"]), text.strip())
        return {"text": text.strip(), "coarse": coarse, "fine": fine}
