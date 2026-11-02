"""Gentle female Mandarin clips for the run-page voice helper."""

from __future__ import annotations

import asyncio
import hashlib
import re

import heacon.config as config

VOICE = "zh-CN-XiaoxiaoNeural"
VOICE_DIR = config.WEB_DIR / "voices"
_SAFE = re.compile(r"[^\u4e00-\u9fffA-Za-z0-9，。！？、：；“”‘’…,.!?;: ]+")


def clip_path(text):
    phrase = _SAFE.sub("", str(text or "")).strip()[:40]
    if not phrase:
        return None
    VOICE_DIR.mkdir(parents=True, exist_ok=True)
    name = hashlib.md5(phrase.encode("utf-8")).hexdigest()[:16] + ".mp3"
    path = VOICE_DIR / name
    if path.is_file() and path.stat().st_size > 800:
        return path
    try:
        import edge_tts
    except ImportError:
        return None
    try:
        comm = edge_tts.Communicate(phrase, VOICE, rate="-8%", pitch="+4Hz")
        asyncio.run(comm.save(str(path)))
    except Exception:
        if path.exists():
            path.unlink()
        return None
    return path if path.is_file() and path.stat().st_size > 800 else None
