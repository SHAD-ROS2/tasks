#!/usr/bin/env python3
"""Run one course launch; close its RViz windows before forwarding Ctrl+C.

The child has its own session, so terminal SIGINT cannot race with launch's
forwarded SIGINT. GUI close uses WM_DELETE_WINDOW, like the window's X button.
No ROS node, launch file or saved RViz configuration is changed.
"""
import ctypes as ct
import fcntl
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path


def invocation_paths(token):
    if not re.fullmatch(r'[0-9]+-[0-9]+-[0-9]+', token):
        raise ValueError('Invalid course invocation token')
    return Path(f'/tmp/course-l04-{token}.json'), Path(f'/tmp/course-l04-{token}.cancel')


def process_start(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def stop_invocation(token):
    """Relay a host signal even when Docker exec has no terminal attached."""
    registration, cancelled = invocation_paths(token)
    cancelled.touch()  # Also prevents a delayed exec from launching after Ctrl+C.
    end = time.monotonic() + 2
    while not registration.exists() and time.monotonic() < end:
        time.sleep(0.05)
    if not registration.exists():
        return 0
    state = json.loads(registration.read_text())
    pid = state['pid']
    if process_start(pid) != state['start']:
        return 0
    os.kill(pid, signal.SIGINT)
    end = time.monotonic() + 22
    while process_start(pid) == state['start'] and time.monotonic() < end:
        time.sleep(0.1)
    if process_start(pid) == state['start']:
        from rover_sim_lab.processes import cleanup_session
        cleanup_session(state['session'])
        if process_start(pid) == state['start']:
            os.kill(pid, signal.SIGKILL)
    registration.unlink(missing_ok=True)
    return 0


def rviz_windows(session):
    """Select only RViz windows whose process belongs to this launch session."""
    if not os.environ.get('DISPLAY'):
        return []
    try:
        listing = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'],
                                          text=True, stderr=subprocess.DEVNULL, timeout=1)
        found = []
        for window in re.findall(r'0x[0-9a-fA-F]+', listing):
            info = subprocess.check_output(['xprop', '-id', window, '_NET_WM_PID', 'WM_CLASS'],
                                           text=True, stderr=subprocess.DEVNULL, timeout=1)
            pid = re.search(r'_NET_WM_PID\(CARDINAL\) = (\d+)', info)
            if pid and '"rviz2"' in info and os.getsid(int(pid.group(1))) == session:
                found.append((int(window, 16), int(pid.group(1))))
        return found
    except (OSError, subprocess.SubprocessError):
        return []


def request_close(windows):
    if not windows:
        return
    class Data(ct.Union):
        _fields_ = [('bytes', ct.c_char*20), ('shorts', ct.c_short*10), ('longs', ct.c_long*5)]
    class Client(ct.Structure):
        _fields_ = [('type', ct.c_int), ('serial', ct.c_ulong), ('send_event', ct.c_int),
                    ('display', ct.c_void_p), ('window', ct.c_ulong),
                    ('message_type', ct.c_ulong), ('format', ct.c_int), ('data', Data)]
    class Event(ct.Union):
        _fields_ = [('client', Client), ('padding', ct.c_long*24)]
    xlib = ct.CDLL('libX11.so.6')
    xlib.XOpenDisplay.argtypes = [ct.c_char_p]
    xlib.XOpenDisplay.restype = ct.c_void_p
    xlib.XInternAtom.argtypes = [ct.c_void_p, ct.c_char_p, ct.c_int]
    xlib.XInternAtom.restype = ct.c_ulong
    xlib.XSendEvent.argtypes = [ct.c_void_p, ct.c_ulong, ct.c_int, ct.c_long, ct.c_void_p]
    xlib.XFlush.argtypes = [ct.c_void_p]
    xlib.XCloseDisplay.argtypes = [ct.c_void_p]
    display = xlib.XOpenDisplay(None)
    if not display:
        return
    try:
        protocols = xlib.XInternAtom(display, b'WM_PROTOCOLS', False)
        delete = xlib.XInternAtom(display, b'WM_DELETE_WINDOW', False)
        for window, _pid in windows:
            event = Event()
            event.client.type, event.client.window, event.client.display = 33, window, display
            event.client.message_type, event.client.format = protocols, 32
            event.client.data.longs[0] = delete
            xlib.XSendEvent(display, window, False, 0, ct.byref(event))
        xlib.XFlush(display)
    finally:
        xlib.XCloseDisplay(display)


def finish(process):
    windows = rviz_windows(process.pid)
    try:
        request_close(windows)
    except (OSError, ValueError):
        pass  # Headless/no X11: normal ROS launch shutdown still applies.
    if windows:
        print('[course] Closing RViz before stopping the ROS launch...', file=sys.stderr, flush=True)
        deadline = time.monotonic()+3.0
        # The window may disappear while Qt/Ogre still joins its render threads.
        # Wait for the process, not only the window, before signalling launch.
        while time.monotonic() < deadline and any(os.path.exists(f'/proc/{pid}') for _, pid in windows):
            time.sleep(0.1)
    if process.poll() is None:
        # Signal ONLY launch; launch forwards once to its own processes.
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        print('[course] Launch did not stop in 5 s; terminating its process group.', file=sys.stderr)
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return process.wait(timeout=2)
        try:
            return process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            return process.wait(timeout=2)


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '--stop':
        return stop_invocation(sys.argv[2])
    if len(sys.argv) < 3:
        print('Usage: run_launch.py PACKAGE LAUNCH_FILE [launch arguments]', file=sys.stderr)
        return 2
    lock = open('/tmp/l04-session.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('An L04 launch is already running; stop it first.', file=sys.stderr)
        return 2
    token = os.environ.get('COURSE_INVOCATION')
    registration, cancelled = invocation_paths(token) if token else (None, None)
    if cancelled and cancelled.exists():
        cancelled.unlink(missing_ok=True)
        return 130
    stopping = False
    def interrupt(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    # Without -n, launch in a terminal assumes its children already received
    # the terminal's group SIGINT. This child session intentionally did not.
    process = subprocess.Popen(['ros2', 'launch', '--noninteractive', *sys.argv[1:]],
                               start_new_session=True)
    if registration:
        state = dict(pid=os.getpid(), start=process_start(os.getpid()), session=process.pid)
        temporary = registration.with_suffix('.tmp')
        temporary.write_text(json.dumps(state))
        temporary.replace(registration)
        if cancelled.exists():
            stopping = True
    while process.poll() is None and not stopping:
        time.sleep(0.1)
    code = finish(process) if stopping else process.returncode
    from rover_sim_lab.processes import cleanup_session
    cleanup_session(process.pid)
    if registration:
        registration.unlink(missing_ok=True)
        cancelled.unlink(missing_ok=True)
    return 128-code if code < 0 else code


if __name__ == '__main__':
    sys.exit(main())
