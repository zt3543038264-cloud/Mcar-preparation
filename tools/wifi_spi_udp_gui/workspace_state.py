"""本地工作区存档：原子替换 JSON，并保留上一份有效文件用于恢复。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import tempfile


class WorkspaceStore:
    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.backup = self.path.with_suffix(self.path.suffix + ".bak")

    def load(self) -> tuple[dict | None, str]:
        errors = []
        for path in (self.path, self.backup):
            if not path.exists():
                continue
            try:
                if path.stat().st_size > 2_000_000:
                    raise ValueError("存档过大")
                state = json.loads(path.read_text(encoding="utf-8"))
                if not isinstance(state, dict) or state.get("version") != 1:
                    raise ValueError("存档版本不支持")
                if not isinstance(state.get("sliders"), list) or len(state["sliders"]) > 200:
                    raise ValueError("滑杆列表无效")
                buttons = state.get("buttons", [])
                if not isinstance(buttons, list) or len(buttons) > 200:
                    raise ValueError("按键列表无效")
                warning = "主存档无法读取，已恢复备份" if path == self.backup else ""
                return state, warning
            except (OSError, ValueError) as exc:
                errors.append(str(exc))
        return None, ("存档读取失败：" + "；".join(errors) if errors else "")

    def save(self, state: dict) -> None:
        data = json.dumps(state, ensure_ascii=False, indent=2, allow_nan=False)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=self.path.name + ".", suffix=".tmp", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as file:
                file.write(data)
                file.flush()
                os.fsync(file.fileno())
            # 损坏的主文件不能覆盖有效备份。
            if self.path.exists():
                try:
                    old = json.loads(self.path.read_text(encoding="utf-8"))
                    if isinstance(old, dict) and old.get("version") == 1:
                        shutil.copy2(self.path, self.backup)
                except (ValueError, OSError):
                    pass
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
