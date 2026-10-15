import math

import pytest

from sci_mvsim.bridge import CommandWatchdog, ackermann_command, sensor_frames, sonar_distance


def test_sonar_minimum_ignores_missing_and_out_of_range_samples():
    assert sonar_distance([math.nan, math.inf, 0.0, -1.0, 8.0, 2.0, 0.4], 0.04, 5.0) == 0.4
    assert sonar_distance([0.0, math.nan, math.inf, 8.0], 0.04, 5.0) == math.inf
    assert sonar_distance([], 0.04, 5.0) == math.inf


@pytest.mark.parametrize('samples', [[0.02, 2.0], [-math.inf, 2.0]])
def test_too_close_reading_is_never_reported_as_free_space(samples):
    assert sonar_distance(samples, 0.04, 5.0) == -math.inf


def test_sonar_limits_are_not_silently_reversed():
    with pytest.raises(ValueError):
        sonar_distance([1.0], 5.0, 0.04)


def test_ackermann_limits_preserve_valid_commands_and_prevent_turning_in_place():
    steer = math.radians(24)
    assert ackermann_command(0.3, -0.2, 1.0, 0.3302, steer) == (0.3, -0.2)
    assert ackermann_command(0.0, 1.0, 1.0, 0.3302, steer) == (0.0, 0.0)
    speed, angular = ackermann_command(-3.0, 10.0, 1.0, 0.3302, steer)
    assert speed == -1.0
    assert angular == pytest.approx(math.tan(steer) / 0.3302)


def test_watchdog_expires_after_one_second_and_invalid_input_clears_old_command():
    watchdog = CommandWatchdog(1.0, 0.3302, math.radians(24))
    assert watchdog.output(4.0) == (0.0, 0.0)
    watchdog.accept(0.3, 0.1, 10.0)
    assert watchdog.output(10.999) == (0.3, 0.1)
    assert watchdog.output(11.0) == (0.0, 0.0)
    watchdog.accept(0.4, 0.2, 12.0)
    with pytest.raises(ValueError):
        watchdog.accept(math.nan, 0.2, 12.1)
    assert watchdog.output(12.2) == (0.0, 0.0)


def test_watchdog_does_not_replay_command_after_clock_regression():
    watchdog = CommandWatchdog(1.0, 0.3302, math.radians(24))
    watchdog.accept(0.3, 0.1, 10.0)
    assert watchdog.output(9.0) == (0.0, 0.0)


def sensor_config():
    return {
        'robot': {'name': 'r0'},
        'lidar': {'x': 0.27, 'y': 0.0, 'z': 0.15},
        'sonar': {'z': 0.10, 'poses': [
            {'index': index, 'x': 0.1, 'y': 0.0, 'yaw_deg': index * 45.0}
            for index in reversed(range(8))]},
    }


def test_sensor_configuration_assigns_unique_sorted_frames_and_radians():
    robot, frames = sensor_frames(sensor_config())
    assert robot == 'r0'
    assert [frame.frame_id for frame in frames] == [
        'r0/laser', *(f'r0/sonar{index}' for index in range(8))]
    assert frames[3].yaw == pytest.approx(math.pi / 2)
    assert frames[3].z == 0.10


def test_sensor_configuration_rejects_duplicate_sonars_and_invalid_namespace():
    config = sensor_config()
    config['sonar']['poses'][0]['index'] = 0
    with pytest.raises(ValueError, match='indices 0 through 7'):
        sensor_frames(config)
    config = sensor_config()
    config['robot']['name'] = '/r0/bad'
    with pytest.raises(ValueError, match='namespace'):
        sensor_frames(config)


def test_sensor_configuration_rejects_nonfinite_pose():
    config = sensor_config()
    config['sonar']['poses'][0]['yaw_deg'] = math.nan
    with pytest.raises(ValueError, match='Non-finite'):
        sensor_frames(config)
