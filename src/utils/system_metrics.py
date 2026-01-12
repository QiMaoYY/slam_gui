#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
系统性能指标采集（无psutil依赖）
- CPU：/proc/stat 差分计算
- MEM：/proc/meminfo 计算使用率
- GPU：Jetson /sys/devices/gpu.0/load（0-1000 => 0%-100%）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SystemMetrics:
    cpu_percent: Optional[float] = None
    mem_percent: Optional[float] = None
    gpu_percent: Optional[float] = None


def _read_first_cpu_line() -> Optional[Tuple[int, int]]:
    """
    Returns:
        (total_jiffies, idle_jiffies)
    """
    try:
        with open("/proc/stat", "r", encoding="utf-8") as f:
            line = f.readline().strip()
        if not line.startswith("cpu "):
            return None
        parts = line.split()
        # cpu user nice system idle iowait irq softirq steal guest guest_nice
        nums = [int(x) for x in parts[1:]]
        if len(nums) < 4:
            return None
        idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
        total = sum(nums[:8]) if len(nums) >= 8 else sum(nums)
        return total, idle
    except Exception:
        return None


class CpuUsageSampler:
    """通过/proc/stat差分采样CPU使用率"""

    def __init__(self):
        self._prev = _read_first_cpu_line()

    def sample_percent(self) -> Optional[float]:
        cur = _read_first_cpu_line()
        if self._prev is None or cur is None:
            self._prev = cur
            return None
        prev_total, prev_idle = self._prev
        cur_total, cur_idle = cur
        self._prev = cur
        dt = cur_total - prev_total
        di = cur_idle - prev_idle
        if dt <= 0:
            return None
        usage = (dt - di) / dt * 100.0
        if usage < 0:
            usage = 0.0
        if usage > 100:
            usage = 100.0
        return usage


def read_mem_percent() -> Optional[float]:
    try:
        mem_total = None
        mem_avail = None
        with open("/proc/meminfo", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    mem_total = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    mem_avail = int(line.split()[1])
                if mem_total is not None and mem_avail is not None:
                    break
        if mem_total is None or mem_avail is None or mem_total <= 0:
            return None
        used = mem_total - mem_avail
        pct = used / mem_total * 100.0
        if pct < 0:
            pct = 0.0
        if pct > 100:
            pct = 100.0
        return pct
    except Exception:
        return None


def read_jetson_gpu_percent() -> Optional[float]:
    try:
        with open("/sys/devices/gpu.0/load", "r", encoding="utf-8") as f:
            raw = f.read().strip()
        if not raw:
            return None
        val = int(raw)
        # Jetson: 0..1000
        if val < 0:
            val = 0
        if val > 1000:
            val = 1000
        return val / 10.0
    except Exception:
        return None


