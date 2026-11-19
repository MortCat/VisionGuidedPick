"""Status and graceful shutdown for this project's lock-owning process trees."""
import argparse, base64, fcntl, json, os, pathlib, platform, signal, socket, subprocess, sys, time

ROOT=pathlib.Path(__file__).resolve().parents[2]
COMPONENTS=['arm_01','arm_02','motion','bridge','devices','monitor']
def processes():
    rows={}
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            stat=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
            if stat[0]=='Z': continue
            rows[int(p.name)]={'parent':int(stat[1]),'start':stat[19]}
        except (OSError,ValueError,IndexError): continue
    return rows

def owners(rows):
    found={name:[] for name in COMPONENTS}
    for name in COMPONENTS:
        path=ROOT/'logs/factory_ros/locks'/f'{name}.lock'
        if not path.exists(): continue
        with path.open('r') as lock:
            try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB); continue
            except BlockingIOError: pass
        inode=path.stat()
        for pid in rows:
            try:
                held=(pathlib.Path('/proc')/str(pid)/'fd/9').stat()
                if (held.st_dev,held.st_ino)==(inode.st_dev,inode.st_ino): found[name].append(pid)
            except OSError: continue
    return found

def tree(roots,rows):
    result=set(roots)
    while True:
        children={pid for pid,value in rows.items() if value['parent'] in result}
        if children<=result: break
        result.update(children)
    return {pid:rows[pid]['start'] for pid in result if pid in rows}

def alive(targets):
    rows=processes()
    return {pid:start for pid,start in targets.items() if rows.get(pid,{}).get('start')==start}

def send(targets,sig):
    for pid in alive(targets):
        try: os.kill(pid,sig)
        except ProcessLookupError: pass

def wait(targets,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        remaining=alive(targets)
        if not remaining:return {}
        time.sleep(.2)
    return alive(targets)

def isaac_request(code):
    request=json.dumps({'code':code,'context':'project_shutdown_check','timeout':3})
    if 'microsoft' in platform.release().lower():
        encoded = base64.b64encode(request.encode()).decode()
        script = r"""
$client = [Net.Sockets.TcpClient]::new()
try {
    $client.Connect('127.0.0.1', 8226)
    $stream = $client.GetStream()
    $stream.ReadTimeout = 4000
    $data = [Convert]::FromBase64String('REQUEST')
    $stream.Write($data, 0, $data.Length)
    $client.Client.Shutdown([Net.Sockets.SocketShutdown]::Send)
    $reader = [IO.StreamReader]::new($stream)
    [Console]::Write($reader.ReadToEnd())
} finally { $client.Dispose() }
"""
        return json.loads(subprocess.check_output(
            ['powershell.exe', '-NoProfile', '-NonInteractive', '-Command',
             script.replace('REQUEST', encoded)], timeout=7))

    with socket.create_connection(('127.0.0.1',8226),3) as connection:
        connection.settimeout(4);connection.sendall(request.encode());connection.shutdown(socket.SHUT_WR)
        data=b''
        while True:
            chunk=connection.recv(65536)
            if not chunk:break
            data+=chunk
    return json.loads(data)


def simulation_stopped(pause=False):
    if pause:
        reply = isaac_request('import omni.timeline; omni.timeline.get_timeline_interface().pause()')
        if reply.get('status') != 'ok':
            return False
    for _ in range(8 if pause else 1):
        reply = isaac_request('import json,omni.timeline; print(json.dumps({"playing":omni.timeline.get_timeline_interface().is_playing()}))')
        if reply.get('status') != 'ok':
            return False
        if json.loads(reply['output'])['playing'] is False:
            return True
        if pause:
            time.sleep(.2)
    return False


def web_instance(rows):
    try:
        row = json.loads((ROOT / 'logs/teach_target/process.json').read_text())
        pid = row['pid']
        if rows.get(pid, {}).get('start') == row['start']:
            return f"{pid}:{row['start']}"
    except (OSError, KeyError, ValueError):
        pass
    return None

def app_window(rows):
    """The App browser started by this checkout's watcher, verified by identity and profile."""
    try:
        row = json.loads((ROOT / 'logs/teach_target/app-window.json').read_text())
        pid, profile = int(row['pid']), ROOT / 'logs/teach_target/app-profile'
        if rows.get(pid, {}).get('start') != row['start'] or pathlib.Path(row['profile']) != profile:
            return None
        # Chrome rewrites its argv into one space-joined string, so match within the raw cmdline.
        arguments = pathlib.Path('/proc', str(pid), 'cmdline').read_bytes().replace(b'\0', b' ') + b' '
        if f'--user-data-dir={profile} '.encode() not in arguments:
            return None
        return row
    except (OSError, KeyError, TypeError, ValueError):
        return None

def close_app_window():
    """Close the owned App after Stop; its watcher then exits without a second Stop."""
    row = app_window(processes())
    if row is None:
        return 0
    send({row['pid']: row['start']}, signal.SIGTERM)
    remaining = wait({row['pid']: row['start'], row['watcher_pid']: row['watcher_start']}, 10)
    if remaining:
        print('App window still running: ' + ', '.join(map(str, remaining)), file=sys.stderr)
        return 1
    print('App window closed.')
    return 0

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--status',action='store_true')
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--simulation-stopped',action='store_true',help='Operator confirms Isaac Stop when Python Server is unavailable')
    parser.add_argument('--app-closed',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--web-instance',help=argparse.SUPPRESS)
    args=parser.parse_args()
    rows=processes();found=owners(rows)
    for name,pids in found.items():print(f'{name}: '+(', '.join(map(str,pids)) if pids else 'stopped'))
    print(f'web: {web_instance(rows) or "stopped"}')
    app = app_window(rows)
    print(f'app: {app["pid"] if app else "stopped"}')
    if args.status or args.dry_run:return 0
    result = stop(args)
    # Outside the lifecycle lock: the watcher's own --app-closed call may need it.
    if result == 0 and not args.app_closed:
        result = close_app_window()
    return result

def stop(args):
    lock_path=ROOT/'logs/factory_ros/project-start.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a') as lock:
        mode = fcntl.LOCK_EX if args.app_closed else fcntl.LOCK_EX | fcntl.LOCK_NB
        try:
            fcntl.flock(lock, mode)
        except BlockingIOError:
            print('Startup or shutdown already in progress.', file=sys.stderr)
            return 3
        # Refresh under the shared lifecycle lock. Never signal a whole session/group:
        # these launches can share a session with Isaac Sim.
        rows=processes();found=owners(rows)
        roots=[pid for pids in found.values() for pid in pids]
        current_web = web_instance(rows)
        if args.web_instance and args.web_instance != current_web:
            return 0
        if roots and not args.simulation_stopped:
            try:
                stopped = simulation_stopped(pause=args.app_closed)
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError):
                stopped = False
            if not stopped:
                print('Cannot confirm Isaac is paused. Pause it, then run bash stop.sh --simulation-stopped.', file=sys.stderr)
                return 2
        targets=tree(roots,rows)
        send({pid:targets[pid] for pid in roots},signal.SIGINT)
        remaining=wait(targets,8)
        if remaining:
            send(remaining,signal.SIGTERM);remaining=wait(remaining,4)
        if remaining:
            print('Still running: '+', '.join(map(str,remaining)),file=sys.stderr)
        web_result = 0
        if current_web:
            web_result = subprocess.call(['bash', str(ROOT / 'deployment/linux/web_service.sh'), '--stop',
                                          '--web-instance', current_web])
        if not remaining and web_result == 0:
            print('ROS backend and Web service stopped.')
        return 1 if remaining or web_result else 0
if __name__=='__main__':sys.exit(main())
