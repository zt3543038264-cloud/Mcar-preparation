"""有界历史缓存和保留峰谷的绘图抽样；所有时间使用同一会话零点。"""
from bisect import bisect_left, bisect_right
import math


class PlotHistory:
    def __init__(self, capacity=100000):
        self.capacity = capacity
        self.times = []
        self.values = []
        self.start = 0

    def append(self, sample):
        stamp, value = sample
        if not math.isfinite(stamp) or not math.isfinite(value):
            return
        if self.times:
            stamp = max(stamp, self.times[-1])
        self.times.append(stamp)
        self.values.append(value)
        self.start = max(self.start, len(self.times) - self.capacity)
        # 批量回收前缀，避免每次追加都搬移整个历史数组。
        if self.start >= max(1, self.capacity // 4):
            del self.times[:self.start]
            del self.values[:self.start]
            self.start = 0

    def __len__(self):
        return len(self.times) - self.start

    @property
    def bounds(self):
        return (self.times[self.start], self.times[-1]) if len(self) else None

    def view(self, low, high, budget, cutoff=None):
        """按视口二分定位；点多时每桶保留极小/极大值，不虚构平滑数据。"""
        if not len(self):
            return [], []
        left = max(self.start, bisect_left(self.times, low, lo=self.start) - 1)
        right = min(len(self.times), bisect_right(self.times, high, lo=self.start) + 1)
        if cutoff is not None:
            right = min(right, bisect_right(self.times, cutoff, lo=self.start))
        if left >= right:
            return [], []
        if right - left <= budget:
            return self.times[left:right], self.values[left:right]
        indices = [left]
        buckets = max(1, (budget - 2) // 2)
        count = right - left - 2
        for bucket in range(buckets):
            a = left + 1 + count * bucket // buckets
            b = left + 1 + count * (bucket + 1) // buckets
            if b <= a:
                continue
            low_i = min(range(a, b), key=self.values.__getitem__)
            high_i = max(range(a, b), key=self.values.__getitem__)
            indices.extend(sorted({low_i, high_i}))
        indices.append(right - 1)
        return [self.times[i] for i in indices], [self.values[i] for i in indices]
