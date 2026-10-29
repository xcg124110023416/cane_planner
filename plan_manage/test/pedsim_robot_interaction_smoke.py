#!/usr/bin/env python3
"""Verify real pedsim's interactive/independent modes on owned private masters.

Requires the pinned pedsim overlay. Checks startup and replaced scenes, retained
peer/goal/wall forces, actual motion and the unchanged robot-input loss latch.
"""
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from export_pedsim_blank import export_world
from pedsim_round import run_owned, stop_owned


def child(directory, mode):
    import rospy
    from geometry_msgs.msg import Point, Pose2D, PoseStamped
    from nav_msgs.msg import Odometry
    from pedsim_msgs.msg import AgentStates
    from pedsim_srvs.srv import EditScene, EditSceneRequest
    from std_msgs.msg import Bool, String

    process = subprocess.Popen(['roslaunch', '--wait', 'plan_manage', 'pedsim_single_source.launch',
        'scene_file:='+str(directory/'world.xml'), 'pedestrian_speeds:=[0.8]',
        'enable_scene_editor:=true', 'auto_start:=true',
        'pedestrians_avoid_robot:='+str(mode == 'interactive').lower()], start_new_session=True)
    try:
        rospy.init_node('robot_interaction_smoke', disable_signals=True)
        batches = []; statuses = []; guards = []
        subscribers = [
            rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, batches.append, queue_size=100),
            rospy.Subscriber('/pedsim_single_bridge/status', String, lambda m: statuses.append(m.data), queue_size=20),
            rospy.Subscriber('/pedsim_simulator/guard_status', String, lambda m: guards.append(m.data), queue_size=20)]
        odom_pub = rospy.Publisher('/sim_odom', Odometry, queue_size=1)
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        ready_pub = rospy.Publisher('/pedsim_scene/ready', Bool, queue_size=1, latch=True)

        def odom(_):
            msg = Odometry(); msg.header.frame_id = 'world'; msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = 2.; msg.pose.pose.position.y = .25
            msg.pose.pose.orientation.w = 1.
            odom_pub.publish(msg)

        timer = rospy.Timer(rospy.Duration(.05), odom)

        def wait(predicate, description, timeout=10.):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                if predicate(): return
                assert process.poll() is None, 'pedsim launch exited'
                time.sleep(.025)
            raise AssertionError(description+'; guards='+str(guards[-2:]))

        wait(lambda: goal_pub.get_num_connections() and any('state=PAUSED_READY ' in g for g in guards), 'guard readiness')
        ready_pub.publish(Bool(True))  # This harness owns the startup/editor configuration.
        goal = PoseStamped(); goal.header.frame_id = 'world'; goal.pose.orientation.w = 1.
        goal_pub.publish(goal)
        wait(lambda: len(batches) >= 8 and 'RUNNING' in statuses, 'startup motion')

        def people(frames):
            return [a for b in frames for a in b.agent_states if a.type != 2]

        def magnitude(v): return math.hypot(v.x, v.y)

        def check_robot_response(frames):
            agents = people(frames)
            assert agents and all(len(b.agent_states) == 1 for b in frames)
            force = max(magnitude(a.forces.social_force) for a in agents)
            if mode == 'independent':
                assert force < 1e-12, 'independent pedestrian still reacts to robot'
            else:
                assert force > .01, 'interactive pedestrian lost robot response'
            assert max(magnitude(a.twist.linear) for a in agents) > .1, 'goal-driven motion lost'
            return force

        startup_force = check_robot_response(batches[:8])
        service = rospy.ServiceProxy('/pedsim_simulator/edit_scene', EditScene)
        revision = 0

        def command(name, **kwargs):
            nonlocal revision
            response = service(EditSceneRequest(command=name, expected_revision=revision, **kwargs))
            assert response.success, response.message
            revision = response.revision

        def sample(starts, goals, headings=None, speeds=None):
            headings = headings or [0.]*len(starts)
            command('replace', starts=[Pose2D(x=x, y=y, theta=h) for (x, y), h in zip(starts, headings)],
                    goals=[Point(x=x, y=y) for x, y in goals],
                    desired_speeds=speeds if speeds is not None else [.8]*len(starts), end_behaviors=['stop']*len(starts))
            command('start')
            cutoff = rospy.Time.now()+rospy.Duration(.1)
            wait(lambda: sum(b.header.stamp > cutoff for b in batches) >= 8, 'edited-scene motion')
            frames = [b for b in batches if b.header.stamp > cutoff]
            assert all(len(b.agent_states) == len(starts) for b in frames)
            return frames

        edited_force = check_robot_response(sample([(0., 0.)], [(4., 0.)]))
        peers = people(sample([(0., 0.), (1.2, .2)], [(4., 0.), (-3., .2)]))
        peer_force = max(magnitude(a.forces.social_force) for a in peers)
        assert peer_force > .01, 'peer interaction was disabled with robot response'
        wall_agents = people(sample([(13.8, 0.)], [(13.8, 4.)]))
        wall_force = max(magnitude(a.forces.obstacle_force) for a in wall_agents)
        desired_force = max(magnitude(a.forces.desired_force) for a in wall_agents)
        assert wall_force > .01 and desired_force > .01, 'wall or goal force was disabled'

        rear_agents = people(sample([(4., 0.)], [(8., 0.)]))
        rear_force = max(magnitude(a.forces.social_force) for a in rear_agents)
        assert rear_force < 1e-12, 'pedestrian reacts to the robot behind its field of view'
        assert max(abs(a.pose.position.y) for a in rear_agents) < 1e-12, 'rear robot deflected pedestrian'
        # At rest, a deliberately different goal must not override the placed
        # heading. Both signed boundaries belong to the 220-degree view sector.
        fov_cases = []
        for angle in (0., 109.9, 110., 110.1, -109.9, -110., -110.1, 180.):
            yaw = math.radians(angle)
            # Keep the robot fixed and move the stationary pedestrian so its
            # relative bearing is exactly the requested angle at every sample.
            start = (2.-1.5*math.cos(yaw), .25-1.5*math.sin(yaw))
            frames = sample([start], [(start[0]+4., start[1])], speeds=[0.])
            force = max(magnitude(a.forces.social_force) for a in people(frames))
            visible = mode == 'interactive' and abs(angle) <= 110.
            assert force > .01 if visible else force < 1e-12, ('FOV boundary', mode, angle, force)
            fov_cases.append(dict(bearing_degrees=angle, social_force=force, visible=visible))
        placed = people(sample([(0., 0.)], [(4., 0.)], headings=[math.pi], speeds=[0.]))
        assert max(magnitude(a.forces.social_force) for a in placed) < 1e-12, 'stationary heading ignored'
        for a in placed:
            assert abs(abs(2.*math.atan2(a.pose.orientation.z, a.pose.orientation.w))-math.pi) < 1e-9
        assert 'FAULT_RESTART_ROUND' not in statuses, 'unexpected input fault'

        # Ignoring robot forces must not bypass the real observation/TF guard.
        timer.shutdown()
        wait(lambda: 'FAULT_RESTART_ROUND' in statuses, 'real input loss did not latch')
        response = service(EditSceneRequest(command='start', expected_revision=revision))
        assert not response.success, 'scene start cleared a real input fault'
        report = dict(mode=mode, passed=True, startup_robot_social_force=startup_force,
                      edited_robot_social_force=edited_force, peer_social_force=peer_force,
                      rear_robot_social_force=rear_force, fov_cases=fov_cases,
                      wall_force=wall_force, desired_force=desired_force, agent_frames=len(batches),
                      input_loss_latched=True)
        (directory/'results.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        return 0
    finally:
        stop_owned(process)
        if 'rospy' in locals(): rospy.signal_shutdown('interaction smoke finished')


def main():
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    if len(sys.argv) == 4 and sys.argv[1] == '--child':
        return child(Path(sys.argv[2]), sys.argv[3])
    for mode in ('interactive', 'independent'):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'; os.environ.pop('ROS_IP', None)
        directory = Path(tempfile.mkdtemp(prefix='cane-pedsim-%s-test.' % mode))

        def prepare():
            export_world(Path.home()/'cane-pedsim-ws/upstream/pedsim_simulator/scenarios/blank_world.xml',
                         directory, 'headon')

        print('MODE', mode, 'PRIVATE_MASTER', port, 'EVIDENCE', directory, flush=True)
        result = run_owned(port, prepare, lambda _: [
            '/usr/bin/python3', str(Path(__file__).resolve()), '--child', str(directory), mode])
        if result: return result
    return 0


if __name__ == '__main__': raise SystemExit(main())
