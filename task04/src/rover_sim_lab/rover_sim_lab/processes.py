"""Linux container cleanup restricted to the session created by this runner."""
import os
from pathlib import Path
import signal
import sys
import time


def session_members(session):
    members=[]
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            try:
                pid=int(path.name)
                if os.getsid(pid)==session and pid!=os.getpid():
                    # Zombies cannot be signalled and are reaped by docker-init.
                    if path.joinpath('stat').read_text().split(') ',1)[1][0]!='Z':
                        members.append(pid)
            except (ProcessLookupError,FileNotFoundError,PermissionError):
                pass
    return members


def cleanup_session(session):
    """A launch process can exit before a renderer's child has finished."""
    for sig,timeout in [(signal.SIGTERM,2.0),(signal.SIGKILL,2.0)]:
        members=session_members(session)
        if not members:
            return
        print(f'[course] Completing cleanup of owned session {session}: {members} ({sig.name})',file=sys.stderr)
        for pid in members:
            try:
                if os.getsid(pid)==session:
                    os.kill(pid,sig)
            except ProcessLookupError:
                pass
        end=time.monotonic()+timeout
        while time.monotonic()<end and session_members(session):
            time.sleep(0.05)
    if session_members(session):
        raise RuntimeError(f'Owned session {session} did not stop; use ./course down')
