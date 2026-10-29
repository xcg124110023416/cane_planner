#!/usr/bin/env python3
"""Optional real pedsim/navigation smoke check; run after sourcing both ROS workspaces.

python3 plan_manage/test/pedsim_navigation_smoke.py crowd_5
Add --repeat-goals to verify arrival, pose reset, idle heartbeats and a second goal.
Uses a free private master, sends the configured robot poses, checks all people
and force/prediction markers, writes /tmp evidence, and stops only its own round.
Not registered in catkin: the pinned pedsim overlay is optional.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('scenario', nargs='?', default='crowd_2')
parser.add_argument('--edit-scene', action='store_true')
parser.add_argument('--repeat-goals', action='store_true')
parser.add_argument('--pedestrian-mode', choices=['interactive', 'independent'], default='interactive')
args = parser.parse_args()
scenario = args.scenario
edit_scene = args.edit_scene
repeat_goals = args.repeat_goals
evidence_tag = 'repeat-goals' if repeat_goals else ('stage4-edited' if edit_scene else 'stage3')
if args.pedestrian_mode == 'independent': evidence_tag += '-independent'
package=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(package/'scripts'))
from export_pedsim_blank import load_blank_scene
scene=load_blank_scene(scenario)
expected_people=len(scene['pedestrians'])
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
os.environ.pop('ROS_IP', None)
logpath = Path('/tmp/cane-pedsim-%s-%s.log' % (evidence_tag, scenario))
log = logpath.open('w')
process = subprocess.Popen(['bash', str(package/'scripts/start_pedsim_navigation.sh'),
                            'start', '--world', 'blank', '--scenario', scenario, '--fresh-scene', '--no-rviz',
                            '--pedestrian-mode', args.pedestrian_mode, '--port', str(port)],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    from pedsim_round import TimeoutTransport
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'], transport=TimeoutTransport())
    deadline = time.monotonic()+25
    while time.monotonic() < deadline:
        if process.poll() is not None: raise RuntimeError('round exited; inspect '+str(logpath))
        try:
            if master.lookupNode('/stage3_check', '/kin_replan_node')[0] == 1: break
        except (OSError, xmlrpc.client.Error): pass
        time.sleep(.1)
    else: raise RuntimeError('round startup timed out')
    import rospy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, Path as NavPath
    from onboard_detector.msg import DynamicObstacles
    from pedsim_msgs.msg import AgentStates
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String,Float64MultiArray
    from visualization_msgs.msg import MarkerArray
    from rosgraph_msgs.msg import Log
    rospy.init_node('cane_stage3_check', anonymous=True, disable_signals=True)
    odoms = []; agents = []; observations = []; statuses = []; paths = []; clouds = []; force_frames = []; timed_frames=[]; execution=[]; transactions=[]; guards=[]; arrivals=[]
    subscribers = [
        rospy.Subscriber('/rosout_agg', Log, lambda m: arrivals.append(m.msg) if '[MPC] Reached final goal' in m.msg else None, queue_size=20),
        rospy.Subscriber('/mpc/timed_collision',MarkerArray,timed_frames.append,queue_size=100),
        rospy.Subscriber('/mpc/execution_diagnostics',Float64MultiArray,lambda m: execution.append(list(m.data)),queue_size=100),
        rospy.Subscriber('/mpc/transaction_diagnostics',Float64MultiArray,lambda m: transactions.append(list(m.data)),queue_size=100),
        rospy.Subscriber('/mpc/convex_corridor', MarkerArray, force_frames.append, queue_size=30),
        rospy.Subscriber('/sim_odom', Odometry, odoms.append, queue_size=100),
        rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, agents.append, queue_size=100),
        rospy.Subscriber('/onboard_detector/dynamic_obstacles_info', DynamicObstacles, observations.append, queue_size=100),
        rospy.Subscriber('/pedsim_single_bridge/status', String, lambda msg: statuses.append(msg.data), queue_size=10),
        rospy.Subscriber('/pedsim_simulator/guard_status', String, lambda msg: guards.append(msg.data), queue_size=10),
        rospy.Subscriber('/mpc/path', NavPath, lambda msg: paths.append(len(msg.poses)), queue_size=10),
        rospy.Subscriber('/simulation_generator/global_cloud', PointCloud2,
                         lambda msg: clouds.append((msg.width*msg.height, msg.header.frame_id)), queue_size=1)]
    startpub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    goalpub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    deadline = time.monotonic()+15
    while time.monotonic() < deadline:
        if startpub.get_num_connections() >= 2 and goalpub.get_num_connections() >= 2 and clouds: break
        time.sleep(.1)
    else: raise RuntimeError('planner subscribers or shared map not ready')
    start = PoseWithCovarianceStamped(); start.header.frame_id = 'world'; start.header.stamp = rospy.Time.now()
    start.pose.pose.position.x, start.pose.pose.position.y = scene['robot_start']; start.pose.pose.orientation.w = 1.
    startpub.publish(start)
    time.sleep(1.)
    if edit_scene:
        from cane_pedestrian import social_scene
        from plan_manage.msg import SocialSceneState
        from plan_manage.srv import ControlSocialScene, ControlSocialSceneRequest
        rospy.wait_for_service('/pedsim_scene/control', timeout=10.)
        state = rospy.wait_for_message('/pedsim_scene/state', SocialSceneState, timeout=5.)
        assert state.ready
        robot_before = odoms[-1].pose.pose.position
        edited = social_scene.from_startup(scene)
        for config, expected in zip(edited['pedestrians'], scene['pedestrians']):
            config['desired_speed'] *= .85
            config['start'][1] += .2
            config['goal'][1] += .2
            expected['desired_max_speed'] = config['desired_speed']
        response = rospy.ServiceProxy('/pedsim_scene/control', ControlSocialScene)(
            ControlSocialSceneRequest(command='load',expected_revision=state.revision,
                                     scenario_yaml=social_scene.dumps(edited)))
        assert response.success, response.message
        time.sleep(.25)
        robot_after = odoms[-1].pose.pose.position
        assert math.hypot(robot_after.x-robot_before.x, robot_after.y-robot_before.y) < 1e-9, 'scene editing reset robot'
    goal = PoseStamped(); goal.header.frame_id = 'world'; goal.header.stamp = rospy.Time.now()
    goal.pose.position.x, goal.pose.position.y = scene['robot_goal']; goal.pose.orientation.w = 1.
    goalpub.publish(goal)
    began = time.monotonic(); deadline = began+45.; reached = False
    while time.monotonic() < deadline:
        if process.poll() is not None: raise RuntimeError('navigation round exited')
        if 'FAULT_RESTART_ROUND' in statuses:
            def max_gap(messages):
                stamps=[m.header.stamp.to_sec() for m in messages]
                return max((b-a for a,b in zip(stamps,stamps[1:])),default=0.)
            print(json.dumps(dict(fault=True,port=port,guards=guards,odom_max_gap=max_gap(odoms),
                                  agents_max_gap=max_gap(agents),observation_max_gap=max_gap(observations))),flush=True)
            raise RuntimeError('pedsim bridge fault')
        if odoms:
            p = odoms[-1].pose.pose.position
            reached = math.hypot(p.x-goal.pose.position.x, p.y-goal.pose.position.y) < .2
            if reached and time.monotonic()-began >= 9. and (not repeat_goals or arrivals): break
        time.sleep(.1)
    if repeat_goals:
        assert reached and arrivals, 'first navigation did not complete arrival'
        # Re-place the robot, then take two seconds to choose the next goal
        # while pedsim remains running, as an operator does in RViz.
        start.header.stamp = rospy.Time.now()
        start.pose.pose.position.y += .5
        startpub.publish(start)
        reset_at = rospy.Time.now()
        time.sleep(2.)
        assert 'FAULT_RESTART_ROUND' not in statuses, 'robot pose reset interrupted the live pedsim input heartbeat'
        held = [m for m in odoms if m.header.stamp > reset_at+rospy.Duration(.3)]
        assert len(held) >= 10, 'no stationary odometry while selecting the second goal'
        assert all(math.hypot(m.pose.pose.position.x-start.pose.pose.position.x,
                              m.pose.pose.position.y-start.pose.pose.position.y)<1e-8 for m in held), 'robot moved before the second goal'
        assert all(abs(m.twist.twist.linear.x)+abs(m.twist.twist.linear.y)<1e-9 for m in held)
        goal.header.stamp = rospy.Time.now();goal.pose.position.y += .5
        goalpub.publish(goal)
        deadline = time.monotonic()+45.;reached = False
        while time.monotonic()<deadline:
            assert process.poll() is None, 'second navigation round exited'
            assert 'FAULT_RESTART_ROUND' not in statuses, 'pedsim fault during second navigation'
            p = odoms[-1].pose.pose.position
            reached = math.hypot(p.x-goal.pose.position.x,p.y-goal.pose.position.y)<.2
            if reached and len(arrivals)>=2:break
            time.sleep(.1)
        assert reached and len(arrivals)>=2, 'second navigation did not complete arrival'
    people = [a for batch in agents for a in batch.agent_states if a.type != 2]
    observed_max_speeds=[0.]*expected_people
    for batch in agents:
        ordered=sorted((a for a in batch.agent_states if a.type!=2),key=lambda a:a.id)
        assert len(ordered)==expected_people, 'wrong simulator pedestrian count'
        for i,person in enumerate(ordered):
            speed=math.hypot(person.twist.linear.x,person.twist.linear.y)
            assert speed<=scene['pedestrians'][i]['desired_max_speed']+1e-6, 'per-person speed not applied'
            observed_max_speeds[i]=max(observed_max_speeds[i],speed)
    force_max = max((math.hypot(a.forces.social_force.x, a.forces.social_force.y) for a in people), default=0.)
    checked = 0
    namespaces = set()
    for frame in force_frames:
        arrows = {m.ns: m for m in frame.markers if m.action == 0 and m.type == 0 and m.id==expected_people-1}
        namespaces.update(m.ns for m in frame.markers if m.action == 0)
        if 'mpc_pedestrian_force' not in arrows: continue
        def vector(name):
            if name not in arrows: return (0., 0.)
            marker = arrows[name]
            assert len(marker.points) == 2 and marker.header.frame_id == 'world'
            assert 0. < marker.lifetime.to_sec() <= .3
            a, b = marker.points
            return (b.x-a.x, b.y-a.y)
        total=vector('mpc_pedestrian_force'); wall=vector('mpc_pedestrian_map_force'); robot=vector('mpc_pedestrian_robot_force');peer=vector('mpc_pedestrian_peer_force')
        assert math.hypot(total[0]-wall[0]-robot[0]-peer[0],total[1]-wall[1]-robot[1]-peer[1]) < 1e-9
        assert 'mpc_pedestrian_force_label' in {m.ns for m in frame.markers}
        checked += 1
    assert checked > 5, 'force component markers not exercised'
    assert 'mpc_pedestrian_velocity' in namespaces, 'observed velocity not displayed'
    all_hull_frames=sum({m.id for m in batch.markers if m.action==0 and m.ns=='mpc_pedestrian_used_hull'}==set(range(expected_people)) for batch in force_frames)
    all_timed_frames=sum({m.id%32 for m in batch.markers if m.action==0 and m.ns=='timed_pedestrian'}==set(range(expected_people)) for batch in timed_frames)
    assert all_hull_frames>5 and all_timed_frames>5,'not all pedestrians visualized'
    assert 'mpc_pedestrian_peer_force' in namespaces or expected_people==1,'peer force missing'
    def profile(values):
        values=sorted(values)
        return dict(count=len(values),p50_ms=round(values[len(values)//2]*1000,3),p95_ms=round(values[int(.95*(len(values)-1))]*1000,3),max_ms=round(max(values)*1000,3)) if values else {}
    p = odoms[-1].pose.pose.position if odoms else None
    report = dict(pedestrian_mode=args.pedestrian_mode,edited_scene=edit_scene,repeat_goals=repeat_goals,completed_goals=len(arrivals),expected_people=expected_people,observed_max_speeds=observed_max_speeds,all_hull_frames=all_hull_frames,all_timed_frames=all_timed_frames,execution_seconds=profile([v[7] for v in execution]),transaction_seconds=profile([v[18] for v in transactions if len(v)>18]),mppi_seconds=profile([v[16] for v in transactions if len(v)>16 and math.isfinite(v[16])]),force_frames_checked=checked, marker_namespaces=sorted(namespaces),scenario=scenario, port=port, log=str(logpath), reached=reached,
                  elapsed=round(time.monotonic()-began, 3), robot_final=[p.x, p.y] if p else None,
                  agent_frames=len(agents), observation_frames=len(observations), statuses=statuses,
                  path_max_points=max(paths, default=0), cloud_counts=sorted(set(clouds)),
                  planner_map=rospy.get_param('/kin_replan_node/Collision/static_map_pcd'),
                  max_social_force=force_max,
                  pedestrian_first=[people[0].pose.position.x, people[0].pose.position.y] if people else None,
                  pedestrian_last=[people[-1].pose.position.x, people[-1].pose.position.y] if people else None)
    reportpath = Path('/tmp/cane-pedsim-%s-%s.json' % (evidence_tag, scenario))
    reportpath.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    assert reached, 'robot did not reach the requested goal'
    assert len(observations) >= 20 and people and all(m.num == expected_people for m in observations), 'multi-pedestrian data missing'
    assert 'RUNNING' in statuses, 'social simulation did not run'
    if expected_people > 1 or args.pedestrian_mode == 'interactive':
        assert force_max > 0., 'social interaction missing'
    assert clouds and all(count == 162120 and frame == 'world' for count, frame in clouds), 'wrong simulation map'
    assert report['planner_map'].endswith('/world.pcd'), 'wrong planner map'
    assert max(paths, default=0) > 10, 'local trajectory was not published'
finally:
    if 'rospy' in locals(): rospy.signal_shutdown('stage3 test finished')
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=10)
    log.close()
