"""Optional speech input via faster-whisper (DESIGN.md §3.1)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal


InputMode = Literal["text", "speech"]


@dataclass
class TranscriptionResult:
    text: str
    mode: InputMode
    backend: str


def transcribe_audio(audio_path: str | Path, model_size: str = "small") -> TranscriptionResult:
    """Transcribe WAV/MP3 to text. Requires: pip install faster-whisper."""
    path = Path(audio_path)
    if not path.exists():
        raise FileNotFoundError(path)

    try:
        from faster_whisper import WhisperModel
    except ImportError as exc:
        raise RuntimeError(
            "faster-whisper 未安装。运行: pip install faster-whisper"
        ) from exc

    model = WhisperModel(model_size, device="cpu", compute_type="int8")
    segments, _info = model.transcribe(str(path), beam_size=5)
    text = "".join(seg.text for seg in segments).strip()
    return TranscriptionResult(text=text, mode="speech", backend=f"faster-whisper/{model_size}")


def resolve_user_input(
    text: str | None = None,
    audio_path: str | Path | None = None,
    mode: InputMode = "text",
    whisper_model: str = "small",
) -> TranscriptionResult:
    """Unified entry: text passthrough or speech transcription."""
    if mode == "text":
        if not text:
            raise ValueError("text 模式需要提供 text")
        return TranscriptionResult(text=text.strip(), mode="text", backend="direct")

    if mode == "speech":
        if not audio_path:
            raise ValueError("speech 模式需要提供 audio_path")
        return transcribe_audio(audio_path, model_size=whisper_model)

    raise ValueError(f"unknown mode: {mode}")
