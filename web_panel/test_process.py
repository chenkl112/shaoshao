"""Verify Linux launch process-group cleanup on processes created by this test."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from server import stop_process


@unittest.skipUnless(os.name == 'posix', '实际进程组清理需要 Linux/POSIX')
class ProcessTests(unittest.TestCase):
    def test_launch_and_child_are_stopped(self):
        with tempfile.TemporaryDirectory() as folder:
            pid_file = Path(folder) / 'child.pid'
            source = (
                'import pathlib, subprocess, sys, time\n'
                'child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])\n'
                'pathlib.Path(sys.argv[1]).write_text(str(child.pid))\n'
                'try:\n    time.sleep(60)\n'
                'except KeyboardInterrupt:\n    child.send_signal(2)\n'
                'finally:\n    child.wait(timeout=2)\n')
            process = subprocess.Popen([sys.executable, '-c', source, str(pid_file)],
                start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.monotonic()+3
                while not pid_file.exists() and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(pid_file.exists())
                child_pid = int(pid_file.read_text())
                stop_process(process)
                self.assertIsNotNone(process.poll())
                with self.assertRaises(ProcessLookupError):
                    os.kill(child_pid, 0)
            finally:
                try:
                    os.killpg(process.pid, 9)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)

    def test_already_exited_launch_is_handled(self):
        process = subprocess.Popen([sys.executable, '-c', 'pass'], start_new_session=True)
        process.wait(timeout=3)
        stop_process(process)


if __name__ == '__main__':
    unittest.main()
