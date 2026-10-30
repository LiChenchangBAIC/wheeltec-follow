import math
import unittest

from builtin_interfaces.msg import Time
from wheeltec_ultrasonic.filtered_nav_obstacles import (
    RearLaserFilter, UltrasonicSafety, points_to_pointcloud2)


class RearLaserFilterTests(unittest.TestCase):
    def setUp(self):
        self.filter = RearLaserFilter()
        self.ranges = [4.]*720
        self.angle_min = -math.pi
        self.increment = math.pi/360

    def scan(self, values):
        ranges = list(self.ranges)
        for index, value in values.items():
            ranges[index] = value
        return self.filter.process(ranges, self.angle_min, self.increment)

    def test_rear_body_is_masked_and_front_is_untouched(self):
        result = self.scan({0: .35, 360: .20})
        self.assertTrue(math.isnan(result[0]))
        self.assertEqual(result[360], .20)

    def test_near_rear_obstacle_requires_three_frames(self):
        self.assertTrue(math.isnan(self.scan({0: .42})[0]))
        self.assertTrue(math.isnan(self.scan({0: .42})[0]))
        self.assertEqual(self.scan({0: .42})[0], .42)

    def test_confirmed_obstacle_requires_three_far_frames(self):
        for _ in range(3):
            self.scan({0: .42})
        self.assertEqual(self.scan({0: 4.})[0], .42)
        self.assertEqual(self.scan({0: 4.})[0], .42)
        self.assertEqual(self.scan({0: 4.})[0], 4.)

    def test_unknown_cannot_erase_confirmed_obstacle(self):
        for _ in range(3):
            self.scan({0: .42})
        self.assertEqual(self.scan({0: float('nan')})[0], .42)

    def test_far_rear_obstacle_is_not_delayed(self):
        self.assertEqual(self.scan({0: .60})[0], .60)


class UltrasonicSafetyTests(unittest.TestCase):
    def setUp(self):
        self.safety = UltrasonicSafety()
        for index in range(5):
            now = index*.05
            for channel in 'ABCDEF':
                self.safety.update(channel, 2., now, now)

    def test_clear_after_stable_samples(self):
        for channel in 'ABCDEF':
            distance, status = self.safety.distance_or_veto(channel, .2)
            self.assertIsNone(distance)
            self.assertTrue(status['clear'])

    def test_critical_rear_obstacle_needs_three_frames(self):
        for now in (.25, .30):
            self.safety.update('E', .20, now, now)
            distance, _ = self.safety.distance_or_veto('E', now)
            self.assertIsNone(distance)
        self.safety.update('E', .20, .35, .35)
        distance, status = self.safety.distance_or_veto('E', .35)
        self.assertEqual(distance, .20)
        self.assertEqual(status['reason'], 'ultrasonic_rear_obstacle')

    def test_obstacle_release_needs_three_far_frames(self):
        for now in (.25, .30, .35):
            self.safety.update('E', .20, now, now)
        for now in (.40, .45):
            self.safety.update('E', 2., now, now)
            distance, _ = self.safety.distance_or_veto('E', now)
            self.assertIsNotNone(distance)
        self.safety.update('E', 2., .50, .50)
        distance, _ = self.safety.distance_or_veto('E', .50)
        self.assertIsNone(distance)

    def test_invalid_and_stale_data_fail_closed(self):
        self.safety.update('A', float('nan'), .25, .25)
        distance, status = self.safety.distance_or_veto('A', .25)
        self.assertEqual(distance, .02)
        self.assertFalse(status['clear'])
        distance, status = self.safety.distance_or_veto('F', 2.)
        self.assertEqual(distance, .02)
        self.assertFalse(status['clear'])

    def test_cloud_contains_xyz(self):
        cloud = points_to_pointcloud2([(1., 2., .15)], 'base_link', Time())
        self.assertEqual(cloud.width, 1)
        self.assertEqual(cloud.point_step, 12)
        self.assertEqual(len(cloud.data), 12)
        self.assertEqual(cloud.header.frame_id, 'base_link')


if __name__ == '__main__':
    unittest.main()
