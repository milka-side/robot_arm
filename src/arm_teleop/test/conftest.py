"""Shared rclpy lifecycle for this package's tests.

Session-scoped so multiple test files don't race each other's teardown.
"""
import rclpy
import pytest


@pytest.fixture(scope='session', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
