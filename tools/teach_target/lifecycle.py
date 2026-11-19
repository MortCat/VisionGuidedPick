"""Native Linux/WSL product entry; controls the product Web service."""
import argparse,fcntl,json,os,shutil,signal,socket,subprocess,sys,time
from http.client import HTTPConnection, HTTPException
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from teaching_config import config_path as default_config_path, load_config

ROOT=Path(__file__).resolve().parents[2]
def process_identity(pid):
    try:
        p=Path('/proc')/str(pid);parts=(p/'stat').read_text().rsplit(')',1)[1].split()
        if parts[0]=='Z':return None
        return parts[19]
    except (OSError,IndexError):return None
def alive(row):return bool(row and row.get('start') is not None and process_identity(row['pid'])==row['start'])
def web_ready(port):
    connection = HTTPConnection('127.0.0.1', port, timeout=1)
    try:
        connection.request('GET', '/')
        return connection.getresponse().status == 200
    except (OSError, HTTPException):
        return False
    finally:
        connection.close()


def wait_for_web(row, port, directory):
    deadline = time.monotonic() + 15
    while alive(row):
        if web_ready(port):
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Web UI did not respond on 127.0.0.1:{port}; inspect {directory / "server.log"}')
        time.sleep(.2)
    raise RuntimeError(f'Web service process exited; inspect {directory / "server.log"}')

def watch_app_window(port, browser, web_instance):
    directory = ROOT / 'logs/teach_target'
    with (directory / 'app-window.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('VisionGuidedPick app window is already running.')
            return
        profile = directory / 'app-profile'
        process = subprocess.Popen([
            browser, f'--user-data-dir={profile}', '--no-first-run',
            '--disable-background-mode', '--window-size=1520,850', f'--app=http://127.0.0.1:{port}/',
        ])
        # Stop closes only this exact browser, identified by PID, start time and profile.
        record = directory / 'app-window.json'
        record.write_text(json.dumps({'pid': process.pid, 'start': process_identity(process.pid),
                                      'watcher_pid': os.getpid(), 'watcher_start': process_identity(os.getpid()),
                                      'profile': str(profile), 'web_instance': web_instance}))
        try:
            closed = process.wait() == 0
        finally:
            record.unlink(missing_ok=True)
        # After Stop has already ended this Web instance, there is nothing left to stop.
        pid, _, start = web_instance.partition(':')
        if closed and process_identity(pid) == start:
            subprocess.run(['bash', str(ROOT / 'stop.sh'), '--app-closed',
                            '--web-instance', web_instance], check=True)


def open_app_window(public_url, web_instance):
    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        for name in ('google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser',
                     'microsoft-edge', 'microsoft-edge-stable'):
            browser = shutil.which(name)
            if browser is None:
                continue
            try:
                with (ROOT / 'logs/teach_target/app-window.log').open('a') as log:
                    subprocess.Popen(
                        [sys.executable, '-I', __file__, '--watch-app', browser,
                         '--web-instance', web_instance],
                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                return
            except OSError as error:
                print(f'Could not open the app window with {name}: {error}', file=sys.stderr)
    print(f'Open the product UI in a browser at {public_url}')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stop',action='store_true');parser.add_argument('--status',action='store_true');parser.add_argument('--dry-run',action='store_true');parser.add_argument('--no-browser',action='store_true');parser.add_argument('--open-app',action='store_true');parser.add_argument('--watch-app',help=argparse.SUPPRESS);parser.add_argument('--web-instance',help=argparse.SUPPRESS);args=parser.parse_args()
    cfg=load_config(ROOT);host=cfg['server']['host'];port=cfg['server']['port'];url=f'http://{host}:{port}'
    if host=='0.0.0.0':raise RuntimeError('Configure an explicit host address; do not widen the product binding')
    if args.watch_app:
        watch_app_window(port, args.watch_app, args.web_instance)
        return
    if args.dry_run:
        print(json.dumps({'entry':'Teach Target','url':url,'starts_arm_backend':False,'gpu_training':False}));return
    directory=ROOT/'logs/teach_target';directory.mkdir(exist_ok=True,parents=True)
    with (directory/'lifecycle.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        record=directory/'process.json';row=json.loads(record.read_text()) if record.exists() else None
        if args.status:
            print(json.dumps({'running':alive(row),'url':url,'listening':web_ready(port) if alive(row) else False}));return
        if args.stop:
            if args.web_instance and (not row or args.web_instance != f"{row['pid']}:{row['start']}"):
                return
            if alive(row):
                os.kill(row['pid'],signal.SIGTERM)
                for _ in range(100):
                    if not alive(row):break
                    time.sleep(.1)
                if alive(row):raise RuntimeError('API is still stopping; no duplicate process will be started')
            print('Teach Target stopped. Arm backend unchanged.');return
        if args.open_app:
            wait_for_web(row, port, directory)
            open_app_window(url, f"{row['pid']}:{row['start']}")
            return
        if not alive(row):
            # Report a port conflict before starting another Web service.
            try:
                with socket.create_connection((host,port),timeout=.5):occupied=True
            except OSError:occupied=False
            if occupied:
                raise RuntimeError('Product Web port is occupied; stop the existing service first')
            env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('PYTHONHOME',None)
            with (directory/'server.log').open('a') as log:
                proc=subprocess.Popen([sys.executable,'-I',str(ROOT/'tools/teach_target/service.py')],cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            row={'pid':proc.pid,'start':process_identity(proc.pid),'url':url};record.write_text(json.dumps(row))
        wait_for_web(row, port, directory)
        print('Teach Target: '+url)
        print('Stop ROS and Web: bash stop.sh')
        if not args.no_browser:
            open_app_window(url, f"{row['pid']}:{row['start']}")
if __name__=='__main__':main()
