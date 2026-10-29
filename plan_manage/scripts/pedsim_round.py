#!/usr/bin/env python3
"""Own one private master; never attach navigation/recording to an unowned master."""
import argparse
from export_pedsim_office import load_scene
from export_pedsim_blank import export_world, load_blank_scene
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import uuid
import xmlrpc.client


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .5
        return connection


def rpc(port):
    return xmlrpc.client.ServerProxy('http://127.0.0.1:%d' % port, transport=TimeoutTransport())


def lock_path(port):
    directory = Path.home()/'.ros'/'cane-pedsim-rounds'
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    return directory/('%d.lock' % port)


def acquire(port):
    # File existence is not ownership. Never delete lock files (inode race).
    handle = lock_path(port).open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('This port already has an owned round')
    return handle


def identity(port):
    with lock_path(port).open('r') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            pass
        else:
            raise RuntimeError('No running owned round on this port')
        record = json.load(handle)
    master = rpc(port)
    if master.getPid('/cane_round_check')[2] != record['master_pid']:
        raise RuntimeError('Master PID is not the owned round')
    if master.getParam('/cane_round_check', '/cane_round_token')[2] != record['token']:
        raise RuntimeError('Master identity token mismatch')
    return record


def stop_owned(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def run_owned(port, prepare, launch_command, clock_command=None):
    """prepare runs under the lock; launch_command is called only after PID proof."""
    handle = acquire(port)
    master = launch = rosout = clock = None
    old_handlers = {}
    def interrupted(signum, _frame):
        raise KeyboardInterrupt()
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, interrupted)
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('127.0.0.1', port))
        prepared = prepare()
        # Direct rosmaster never intentionally attaches like roslaunch/roscore.
        master = subprocess.Popen(['rosmaster', '--core', '-p', str(port)], start_new_session=True)
        server = rpc(port)
        deadline = time.monotonic()+10
        while time.monotonic()<deadline:
            if master.poll() is not None:
                raise RuntimeError('Owned master exited; port may have been taken')
            try:
                pid = server.getPid('/cane_round_owner')[2]
                if pid != master.pid:
                    raise RuntimeError('External master took port; refusing attachment')
                break
            except (OSError, xmlrpc.client.Error):
                time.sleep(.05)
        else:
            raise RuntimeError('Owned master did not become ready')
        # A bare rosmaster does not initialize /run_id or launch rosout.
        # roslaunch --wait requires /run_id before it can even create its parent.
        run_id = str(uuid.uuid4())
        if server.setParam('/cane_round_owner', '/run_id', run_id)[0] != 1:
            raise RuntimeError('Could not initialize owned master run_id')
        if server.getParam('/cane_round_owner', '/run_id')[2] != run_id:
            raise RuntimeError('Owned master run_id verification failed')
        token = uuid.uuid4().hex
        server.setParam('/cane_round_owner', '/cane_round_token', token)
        handle.seek(0); handle.truncate()
        json.dump({'master_pid': master.pid, 'token': token}, handle)
        handle.flush(); os.fsync(handle.fileno())
        identity(port)
        if clock_command is not None:
            if server.setParam('/cane_round_owner', '/use_sim_time', True)[0] != 1:
                raise RuntimeError('Could not select owned simulation time')
            clock = subprocess.Popen(clock_command, start_new_session=True)
            deadline = time.monotonic()+10
            while time.monotonic()<deadline:
                identity(port)
                if clock.poll() is not None:
                    raise RuntimeError('Owned clock exited before startup')
                ready = server.getParam('/cane_round_owner', '/pedsim_monotonic_clock/ready')
                if ready[0] == 1 and ready[2] is True:
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('Owned clock did not become ready')
            print('Simulation clock: monotonic /clock (1x), /use_sim_time=true', flush=True)
        rosout = subprocess.Popen(['rosrun', 'rosout', 'rosout', '__name:=rosout'], start_new_session=True)
        deadline = time.monotonic()+10
        while time.monotonic()<deadline:
            identity(port)
            if rosout.poll() is not None:
                raise RuntimeError('Owned rosout exited before startup')
            if server.lookupNode('/cane_round_owner', '/rosout')[0] == 1:
                break
            time.sleep(.05)
        else:
            raise RuntimeError('Owned rosout did not become ready')
        launch = subprocess.Popen(launch_command(prepared), start_new_session=True)
        while launch.poll() is None:
            if clock is not None and clock.poll() is not None:
                raise RuntimeError('Owned clock lost; ending only this round')
            if rosout.poll() is not None:
                raise RuntimeError('Owned rosout lost; ending only this round')
            if master.poll() is not None:
                raise RuntimeError('Owned master lost; ending only this round')
            identity(port)
            time.sleep(.2)
        return launch.returncode
    except KeyboardInterrupt:
        return 130
    finally:
        # Ignore repeated terminal signals during owned child cleanup.
        for sig in old_handlers:
            signal.signal(sig, signal.SIG_IGN)
        try:
            stop_owned(launch)
            stop_owned(rosout)
            stop_owned(clock)
            stop_owned(master)
        finally:
            handle.close()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs='?', default='start', choices=['start', 'record'])
    parser.add_argument('--scenario',default='headon', help='YAML scene name; blank also supports crowd_2 and crowd_5')
    parser.add_argument('--world', choices=['blank', 'office'], default='blank')
    parser.add_argument('--pedestrian-mode', choices=['interactive', 'independent'], default='interactive',
                        help='interactive: avoid robot within 220-deg FOV; independent: ignore robot, retain walls/peers')
    parser.add_argument('--no-rviz', action='store_true', help='Headless navigation; no goals are sent automatically')
    parser.add_argument('--fresh-scene', action='store_true', help='Use the YAML preset without reading or writing remembered edits')
    parser.add_argument('--editor-state-file', help='Explicit scene memory file (blank arena only)')
    parser.add_argument('--port', type=int, default=11551)
    parser.add_argument('--label', default='pedsim_round')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('port must be 1024..65535')
    if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.label):
        parser.error('label must contain only letters, digits, underscore or hyphen')
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % args.port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ.pop('ROS_IP', None)
    package = Path(subprocess.check_output(['rospack', 'find', 'plan_manage'], text=True).strip())
    if args.command == 'record':
        identity(args.port)
        recorder = None
        old_handlers = {}
        def interrupted(signum, _frame):
            raise KeyboardInterrupt()
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, interrupted)
        try:
            recorder = subprocess.Popen(['/usr/bin/python3', str(package/'scripts/record_lightweight_mpc_eval.sh'), args.label], start_new_session=True)
            while recorder.poll() is None:
                identity(args.port)
                time.sleep(.2)
            return recorder.returncode
        except KeyboardInterrupt:
            return 130
        finally:
            for sig in old_handlers:
                signal.signal(sig, signal.SIG_IGN)
            try:
                stop_owned(recorder)
            finally:
                for sig, handler in old_handlers.items():
                    signal.signal(sig, handler)
    scene_config=load_blank_scene(args.scenario) if args.world == 'blank' else load_scene(args.scenario)
    speeds = [p['desired_max_speed'] for p in scene_config['pedestrians']] if args.world == 'blank' else [scene_config['desired_max_speed']]
    def prepare():
        dependency = Path.home()/'cane-pedsim-ws'/'upstream'
        manifest = json.loads((package/'config/pedsim/upstream-manifest.json').read_text())
        for name, entry in manifest['files'].items():
            if hashlib.sha256((dependency/name).read_bytes()).hexdigest() != entry['patched_sha256']:
                raise RuntimeError('Pinned dependency mismatch: '+name)
        directory = Path(tempfile.mkdtemp(prefix='cane-pedsim-round.'))
        if args.world == 'blank':
            metadata = export_world(dependency/'pedsim_simulator/scenarios/blank_world.xml', directory, args.scenario)
            map_pcd = directory/'world.pcd'
            print(json.dumps(metadata, indent=2), flush=True)
            print('RViz robot start: %s; goal: %s (not sent automatically)' %
                  (scene_config['robot_start'], scene_config['robot_goal']), flush=True)
        else:
            map_package = Path(subprocess.check_output(['rospack', 'find', 'map_generator'], text=True).strip())
            map_pcd = map_package/'resource/office2_modify.pcd'
            subprocess.run(['/usr/bin/python3', str(package/'scripts/export_pedsim_office.py'), str(map_pcd), str(directory/'world.xml'),'--scenario',args.scenario], check=True)
        print('Round geometry: %s' % directory, flush=True)
        print('Pedestrian mode: %s (%s)' % (args.pedestrian_mode,
              'people ignore the robot; walls/peers remain active' if args.pedestrian_mode == 'independent'
              else 'people react to the robot only within heading +/-110 deg'), flush=True)
        return directory/'world.xml', map_pcd
    return run_owned(args.port, prepare, lambda world: ['roslaunch', '--wait', 'plan_manage', 'pedsim_navigation.launch',
                     'scene_file:='+str(world[0]), 'map_pcd:='+str(world[1]),
                     'enable_scene_editor:='+str(args.world == 'blank').lower(),
                     'pedestrians_avoid_robot:='+str(args.pedestrian_mode == 'interactive').lower(),
                     'scene_description:='+str(world[0].with_suffix('.json')),
                     'remember_last:='+str(not args.fresh_scene).lower(),
                     'editor_state_file:='+(str(Path(args.editor_state_file).expanduser().resolve()) if args.editor_state_file else ''),
                     'use_static_global_map:='+str(args.world == 'blank').lower(),
                     'visualize:='+str(not args.no_rviz).lower(),
                     'pedestrian_speeds:='+json.dumps(speeds)],
                     clock_command=['/usr/bin/python3',str(package/'scripts/pedsim_monotonic_clock.py')])


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, xmlrpc.client.Error) as error:
        raise SystemExit(str(error))
