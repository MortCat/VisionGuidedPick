"""Stop closes only the App browser owned by this checkout; App close does not recurse."""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
LINUX = sys.platform == 'linux'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fake_browser(folder, exit_code):
    """An executable standing in for Chrome: exits with exit_code on SIGTERM.

    Like Chrome, it rewrites its argv into one space-joined string in /proc/<pid>/cmdline.
    """
    path = Path(folder) / 'browser'
    path.write_text('#!/usr/bin/env python3\nimport signal,sys,time\n'
                    f'signal.signal(signal.SIGTERM,lambda *_: sys.exit({exit_code}))\n'
                    # arg_start/arg_end are /proc/<pid>/stat fields 48 and 49.
                    'start,end=map(int,open("/proc/self/stat").read().rsplit(")",1)[1].split()[45:47])\n'
                    'line=" ".join(sys.argv).encode()[:end-start-1].ljust(end-start,b"\\0")\n'
                    'with open("/proc/self/mem","r+b") as mem: mem.seek(start); mem.write(line)\n'
                    f'open({str(Path(folder) / "browser-ready")!r},"w").close()\n'
                    'time.sleep(60)\n')
    path.chmod(0o755)
    return str(path)


def wait_for(predicate, seconds=10):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.05)
    return False


@unittest.skipUnless(LINUX, 'Linux /proc process identity')
class AppWindowLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = Path(self.folder.name)
        (self.root / 'logs/teach_target').mkdir(parents=True)
        self.processes = load('project_processes_under_test', ROOT / 'deployment/linux/project_processes.py')
        self.processes.ROOT = self.root
        self.children = []

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:
                child.kill()
                child.wait()
        self.folder.cleanup()

    def spawn(self, argv):
        child = subprocess.Popen(argv)
        self.children.append(child)
        return child

    def watcher(self, browser, web_instance):
        """Run the real lifecycle watcher, as start.sh does, with stop.sh replaced by a marker."""
        stop_marker = self.root / 'stop-called'
        stop_script = self.root / 'stop.sh'
        stop_script.write_text(f'#!/usr/bin/env bash\necho "$@" > {stop_marker}\n')
        code = ('import sys,importlib.util;'
                f'sys.path.insert(0,{str(ROOT / "tools/teach_target")!r});'
                f'spec=importlib.util.spec_from_file_location("lifecycle",{str(ROOT / "tools/teach_target/lifecycle.py")!r});'
                'm=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);'
                f'm.ROOT=__import__("pathlib").Path({str(self.root)!r});'
                f'm.watch_app_window(8088,{browser!r},{web_instance!r})')
        return self.spawn([sys.executable, '-c', code]), stop_marker

    def record(self):
        path = self.root / 'logs/teach_target/app-window.json'
        self.assertTrue(wait_for(path.exists), 'watcher records the browser it owns')
        self.assertTrue(wait_for((self.root / 'browser-ready').exists), 'fake browser installed its close handler')
        return json.loads(path.read_text())

    def test_stop_closes_owned_app_without_second_stop(self):
        # Chrome may exit 0 on SIGTERM; the stopped Web instance must still prevent recursion.
        web = self.spawn([sys.executable, '-c', 'import time; time.sleep(60)'])
        web_instance = f'{web.pid}:{self.processes.processes()[web.pid]["start"]}'
        watcher, stop_marker = self.watcher(fake_browser(self.root, 0), web_instance)
        row = self.record()
        self.assertEqual(self.processes.app_window(self.processes.processes())['pid'], row['pid'])
        web.kill(); web.wait()
        self.assertEqual(self.processes.close_app_window(), 0)
        self.assertIsNotNone(watcher.poll(), 'watcher exits with its App')
        self.assertFalse(stop_marker.exists(), 'Stop does not re-enter through the App-closed path')
        self.assertFalse((self.root / 'logs/teach_target/app-window.json').exists())

    def test_user_close_still_stops_live_services(self):
        web = self.spawn([sys.executable, '-c', 'import time; time.sleep(60)'])
        web_instance = f'{web.pid}:{self.processes.processes()[web.pid]["start"]}'
        watcher, stop_marker = self.watcher(fake_browser(self.root, 0), web_instance)
        row = self.record()
        os.kill(row['pid'], 15)  # The operator closes the window.
        self.assertTrue(wait_for(lambda: watcher.poll() is not None))
        self.assertEqual(stop_marker.read_text().split(), ['--app-closed', '--web-instance', web_instance])

    def test_unrelated_browser_is_not_signalled(self):
        # Same recorded identity but without this checkout's profile argument.
        other = self.spawn([fake_browser(self.root, 0), '--user-data-dir=/tmp/someone-else'])
        start = self.processes.processes()[other.pid]['start']
        (self.root / 'logs/teach_target/app-window.json').write_text(json.dumps({
            'pid': other.pid, 'start': start, 'watcher_pid': other.pid, 'watcher_start': start,
            'profile': str(self.root / 'logs/teach_target/app-profile'), 'web_instance': '1:1'}))
        self.assertIsNone(self.processes.app_window(self.processes.processes()))
        self.assertEqual(self.processes.close_app_window(), 0)
        time.sleep(.3)
        self.assertIsNone(other.poll(), 'a browser outside the dedicated profile keeps running')

    def test_stale_record_after_pid_reuse_is_ignored(self):
        other = self.spawn([fake_browser(self.root, 0),
                            f'--user-data-dir={self.root / "logs/teach_target/app-profile"}'])
        (self.root / 'logs/teach_target/app-window.json').write_text(json.dumps({
            'pid': other.pid, 'start': 'older', 'watcher_pid': other.pid, 'watcher_start': 'older',
            'profile': str(self.root / 'logs/teach_target/app-profile'), 'web_instance': '1:1'}))
        with mock.patch.object(self.processes.os, 'kill') as kill:
            self.assertEqual(self.processes.close_app_window(), 0)
        kill.assert_not_called()


if __name__ == '__main__':
    unittest.main()
