"""Offline-ish speech-to-text for tablet voice. Runs on the Mac, which has internet."""

import io
import json
import wave
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MODEL_DIR = ROOT / "models" / "vosk-small-cn"

_model = None
_google = None


def _load_vosk():
    global _model
    if _model is not None:
        return _model
    if not (MODEL_DIR / "am").exists() and not (MODEL_DIR / "conf").exists():
        return None
    from vosk import Model

    _model = Model(str(MODEL_DIR))
    return _model


def _pcm_from_wav(blob):
    with wave.open(io.BytesIO(blob), "rb") as wav:
        channels = wav.getnchannels()
        width = wav.getsampwidth()
        rate = wav.getframerate()
        frames = wav.readframes(wav.getnframes())
    if channels != 1:
        raise ValueError("need mono")
    if width != 2:
        raise ValueError("need 16-bit")
    return frames, rate


def transcribe_wav(blob):
    if _load_vosk() is not None:
        return _transcribe_vosk(blob)
    return _transcribe_google(blob)


def _transcribe_vosk(blob):
    model = _load_vosk()
    if model is None:
        return ""
    from vosk import KaldiRecognizer

    pcm, rate = _pcm_from_wav(blob)
    rec = KaldiRecognizer(model, rate)
    rec.SetWords(True)
    rec.AcceptWaveform(pcm)
    try:
        payload = json.loads(rec.FinalResult())
    except Exception:
        return ""
    return str(payload.get("text") or "").replace(" ", "")


def _transcribe_google(blob):
    try:
        import speech_recognition as sr
    except ImportError:
        return ""
    pcm, rate = _pcm_from_wav(blob)
    audio = sr.AudioData(pcm, rate, 2)
    try:
        return str(sr.Recognizer().recognize_google(audio, language="zh-CN") or "").replace(" ", "")
    except Exception:
        return ""
