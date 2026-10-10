"""历史回看和保峰抽样验证，确保提高刷新不丢真实样本。"""
import unittest
from plot_history import PlotHistory


class PlotHistoryTest(unittest.TestCase):
    def test_all_samples_and_stable_time(self):
        h = PlotHistory(1000)
        for i in range(100):
            h.append((i / 100, i))
        xs, ys = h.view(.3, .5, 1200)
        self.assertEqual(len(h), 100)
        self.assertIn(.4, xs)
        self.assertEqual(ys[xs.index(.4)], 40)

    def test_peaks_retained_with_budget(self):
        h = PlotHistory()
        for i in range(10000):
            h.append((i / 100, 999 if i == 4311 else -777 if i == 7351 else 0))
        xs, ys = h.view(0, 100, 100)
        self.assertLessEqual(len(xs), 100)
        self.assertIn(999, ys)
        self.assertIn(-777, ys)
        self.assertEqual(xs[0], 0)
        self.assertEqual(xs[-1], 99.99)
        self.assertEqual(xs, sorted(xs))

    def test_bounded_retention_and_pause_cutoff(self):
        h = PlotHistory(100)
        for i in range(1000):
            h.append((i, i))
        self.assertEqual(len(h), 100)
        self.assertEqual(h.bounds, (900, 999))
        xs, _ = h.view(900, 999, 1200, cutoff=950)
        self.assertEqual(xs[-1], 950)
        self.assertEqual(h.view(960, 999, 1200, cutoff=950), ([], []))


if __name__ == "__main__":
    unittest.main()
