"""Cross-host mutual exclusion for anything that submits a goal to
``robot_arm_controller`` (the JTC).

A plain ``threading.Lock``/``flock`` only protects within one process or
host; this uses a ROS service (``arm_motion_lock_server.py``) so two
clients on different hosts can't race a goal onto the same controller.
"""

from __future__ import annotations

import contextlib
import threading
import time

from arm_interfaces.srv import AcquireArmMotionLock, ReleaseArmMotionLock

# How long to wait for the lock server to answer a single acquire/release
# call — separate from lease_sec and timeout_sec below.
_SERVICE_WAIT_SEC = 2.0
_CALL_TIMEOUT_SEC = 3.0


class ArmMotionBusy(Exception):
    """Raised when the lock can't be acquired: held by someone else, or
    the lock server is unreachable (fails closed).
    """


@contextlib.contextmanager
def arm_motion_lock(acquire_client, release_client, holder_id: str,
                     lease_sec: float, timeout_sec: float = 0.0):
    """Acquire the cross-host arm-motion lock for the ``with`` block.

    ``lease_sec`` auto-expires the lock server-side if release() never
    arrives. ``timeout_sec`` 0.0 (default) tries once; raises
    ArmMotionBusy if still unavailable when it gives up.
    """
    deadline = time.monotonic() + timeout_sec
    while True:
        granted, message = _call_acquire(acquire_client, holder_id, lease_sec)
        if granted:
            break
        if time.monotonic() >= deadline:
            raise ArmMotionBusy(message)
        time.sleep(0.05)
    try:
        yield
    finally:
        _call_release(release_client, holder_id)


def _call_acquire(client, holder_id: str, lease_sec: float) -> tuple[bool, str]:
    if not client.wait_for_service(timeout_sec=_SERVICE_WAIT_SEC):
        return False, 'arm_motion_lock_server not reachable — refusing to move without it'

    done = threading.Event()
    result = {}

    def _cb(fut):
        result['r'] = fut.result()
        done.set()

    req = AcquireArmMotionLock.Request(holder_id=holder_id, lease_sec=lease_sec)
    client.call_async(req).add_done_callback(_cb)
    if not done.wait(timeout=_CALL_TIMEOUT_SEC):
        return False, 'arm_motion_lock_server did not respond in time'

    r = result.get('r')
    if r is None:
        return False, 'arm_motion_lock_server call failed'
    return bool(r.granted), r.message


def _call_release(client, holder_id: str) -> None:
    if not client.wait_for_service(timeout_sec=_SERVICE_WAIT_SEC):
        return  # best-effort — the lease expires on its own regardless
    done = threading.Event()
    req = ReleaseArmMotionLock.Request(holder_id=holder_id)
    client.call_async(req).add_done_callback(lambda _fut: done.set())
    done.wait(timeout=_CALL_TIMEOUT_SEC)
