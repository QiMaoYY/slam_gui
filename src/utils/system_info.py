#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
系统信息采集（用于“系统状态”页面展示）
尽量使用本机可读文件，避免外部依赖。
"""

from __future__ import annotations

import platform
from dataclasses import dataclass
from typing import Optional


@dataclass
class SystemInfo:
    os_pretty: str
    kernel: str
    arch: str
    device_model: Optional[str]


def _read_first_line(path: str, *, binary: bool = False) -> Optional[str]:
    try:
        if binary:
            with open(path, "rb") as f:
                data = f.readline()
            return data.decode("utf-8", errors="ignore").strip().replace("\x00", "")
        with open(path, "r", encoding="utf-8") as f:
            return f.readline().strip()
    except Exception:
        return None


def read_system_info() -> SystemInfo:
    os_pretty = "Unknown"
    try:
        with open("/etc/os-release", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("PRETTY_NAME="):
                    os_pretty = line.split("=", 1)[1].strip().strip('"')
                    break
    except Exception:
        pass

    kernel = platform.release()
    arch = platform.machine()
    device_model = _read_first_line("/proc/device-tree/model", binary=True)

    return SystemInfo(
        os_pretty=os_pretty,
        kernel=kernel,
        arch=arch,
        device_model=device_model,
    )


