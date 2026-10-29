#!/usr/bin/env python3
"""Acknowledged RViz scene editing for the guarded pedsim blank arena."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import threading

import rospy
from geometry_msgs.msg import Pose2D, Point, PointStamped, PoseStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from pedsim_srvs.srv import EditScene, EditSceneRequest
from plan_manage.msg import SocialPedestrian, SocialSceneState
from plan_manage.srv import ControlSocialScene, ControlSocialSceneResponse
from cane_pedestrian import social_scene
from cane_pedestrian.scene_io import write_scene


def message(person):
    return SocialPedestrian(person['id'], Pose2D(*person['start']), Point(*person['goal'], 0.),
                            person['desired_speed'], person['end_behavior'])


def config(person):
    return dict(id=person.id, start=[person.start.x, person.start.y, person.start.theta],
                goal=[person.goal.x, person.goal.y], desired_speed=person.desired_speed,
                end_behavior=person.end_behavior)


class Editor:
    def __init__(self):
        self.lock = threading.RLock()
        self.revision = 0
        self.selected = 0
        self.snapshot = None
        self.initialized = False
        self.healthy = False
        self.desynchronized = False
        self.fault = ''
        self.error = ''
        self.persistence_error = ''
        self.remember = rospy.get_param('~remember_last', True)
        metadata = json.loads(Path(rospy.get_param('~scene_description')).read_text())
        if metadata['world'] != 'blank':
            raise ValueError('The interactive pedsim editor currently requires the blank arena')
        self.scene = social_scene.from_startup(metadata['scene'])
        context = 'blank:'+metadata['scenario']+':'+rospy.get_param('~scene_context')
        key = hashlib.sha256(context.encode()).hexdigest()[:20]
        default = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))/'cane_planner/social_editor'/(key+'.yaml')
        self.cache = Path(rospy.get_param('~editor_state_file', str(default))).expanduser()
        if self.remember and self.cache.exists():
            try:
                self.scene = social_scene.loads(self.cache.read_text())
                rospy.loginfo('Restored pedsim scene: %s', self.cache)
            except (OSError, ValueError) as exc:
                rospy.logwarn('Ignoring invalid remembered pedsim scene: %s', exc)
        self.state_pub = rospy.Publisher('~state', SocialSceneState, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher('~ready', Bool, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('~visualization', MarkerArray, queue_size=1)
        self.guard_sub = rospy.Subscriber('/pedsim_simulator/guard_status', String, self.guard, queue_size=1)
        self.bridge_sub = rospy.Subscriber('/pedsim_single_bridge/status', String, self.guard, queue_size=1)
        rospy.wait_for_service('/pedsim_simulator/edit_scene', timeout=15.)
        self.upstream = rospy.ServiceProxy('/pedsim_simulator/edit_scene', EditScene)
        self.call('state')
        self.replace(self.scene, remember=False)
        self.initialized = True
        self.control_service = rospy.Service('~control', ControlSocialScene, self.control)
        self.start_sub = rospy.Subscriber('~start_pose', PoseStamped, self.place_start, queue_size=1)
        self.goal_sub = rospy.Subscriber('~goal_pose', PoseStamped, self.place_goal, queue_size=1)
        self.select_sub = rospy.Subscriber('~select_point', PointStamped, self.pick, queue_size=1)
        self.nav_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navigation_goal, queue_size=1)
        self.publish()
        self.timer = rospy.Timer(rospy.Duration(.2), self.poll)

    def call(self, command, candidate=None):
        if self.desynchronized:
            raise ValueError('Simulator scene changed outside this editor; restart the round')
        request = EditSceneRequest(command=command, expected_revision=self.revision)
        if candidate is not None:
            for p in candidate['pedestrians']:
                request.starts.append(Pose2D(*p['start']))
                request.goals.append(Point(*p['goal'], 0.))
                request.desired_speeds.append(p['desired_speed'])
                request.end_behaviors.append(p['end_behavior'])
        response = self.upstream(request)
        if not response.success:
            raise ValueError(response.message)
        if command == 'state' and self.initialized and response.revision != self.revision:
            self.desynchronized = True
            raise ValueError('Simulator scene changed outside this editor; restart the round')
        expected_count = len((candidate or self.scene)['pedestrians'])
        if command != 'state' or self.initialized:
            if (len(response.agent_ids) != expected_count or len(response.poses) != expected_count or
                    len(response.phases) != expected_count or len(set(response.agent_ids)) != expected_count or
                    not all(math.isfinite(v) for p in response.poses for v in (p.x, p.y, p.theta))):
                self.desynchronized = True
                raise ValueError('Invalid simulator scene acknowledgement; restart the round')
        self.revision = response.revision
        self.snapshot = response
        self.healthy = True
        return response

    def guard(self, msg):
        with self.lock:
            if 'LATCHED_' in msg.data or 'FAULT_RESTART_ROUND' in msg.data:
                self.fault = 'Robot input/bridge fault: restart this round'
                self.publish()

    def replace(self, scene, remember=True):
        candidate = social_scene.validate(scene)
        self.call('replace', candidate)
        self.scene = candidate
        identities = [p['id'] for p in candidate['pedestrians']]
        if self.selected not in identities:
            self.selected = identities[0] if identities else 0
        if remember:
            self.persist()

    def persist(self):
        if self.remember:
            try:
                write_scene(self.cache, social_scene.dumps(self.scene))
                self.persistence_error = ''
            except OSError as exc:
                self.persistence_error = str(exc)
                rospy.logwarn('Pedsim scene memory write failed: %s', exc)

    def selected_person(self, scene):
        for p in scene['pedestrians']:
            if p['id'] == self.selected:
                return p
        raise ValueError('Select a pedestrian first')

    def control(self, request):
        with self.lock:
            try:
                if request.command == 'export':
                    return ControlSocialSceneResponse(True, 'OK', social_scene.dumps(self.scene), self.revision)
                if self.fault or not self.healthy or self.desynchronized:
                    raise ValueError(self.fault or 'Simulator unavailable; wait or restart the round')
                if request.expected_revision != self.revision:
                    raise ValueError('Scene changed; wait for the panel to refresh')
                if request.command == 'select':
                    if request.selected_id == 0 and len(self.scene['pedestrians']) >= social_scene.MAX_PEOPLE:
                        raise ValueError('At most five pedestrians')
                    if request.selected_id and not any(p['id'] == request.selected_id for p in self.scene['pedestrians']):
                        raise ValueError('Unknown pedestrian')
                    self.selected = request.selected_id
                elif request.command in ('start', 'pause'):
                    self.call(request.command)
                else:
                    scene = copy.deepcopy(self.scene)
                    if request.command == 'apply':
                        if self.selected:
                            p = self.selected_person(scene)
                            candidate = config(request.pedestrian)
                            if candidate['id'] != p['id']:
                                raise ValueError('Selected pedestrian changed')
                            p.update(candidate)
                        elif request.pedestrian.id:
                            raise ValueError('Selected pedestrian changed')
                        scene['start_on_goal'] = request.start_on_goal
                    elif request.command == 'delete':
                        self.selected_person(scene)
                        scene['pedestrians'] = [p for p in scene['pedestrians'] if p['id'] != self.selected]
                    elif request.command == 'clear':
                        scene['pedestrians'] = []
                    elif request.command == 'load':
                        scene = social_scene.loads(request.scenario_yaml)
                    elif request.command != 'reset':
                        raise ValueError('Unknown editor command')
                    self.replace(scene)
                self.error = ''
                self.publish()
                return ControlSocialSceneResponse(True, 'OK', '', self.revision)
            except (ValueError, rospy.ServiceException) as exc:
                self.error = str(exc)
                self.publish()
                return ControlSocialSceneResponse(False, str(exc), '', self.revision)

    def world_pose(self, msg):
        q = msg.pose.orientation
        if (msg.header.frame_id != 'world' or
                not all(math.isfinite(v) for v in (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, q.x, q.y, q.z, q.w)) or
                abs(msg.pose.position.z) > 1e-3 or abs(q.x) > 1e-3 or abs(q.y) > 1e-3 or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-3):
            raise ValueError('Use RViz Fixed Frame world and a finite planar pose')
        return [msg.pose.position.x, msg.pose.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

    def place(self, msg, goal=False):
        with self.lock:
            try:
                if self.fault or not self.healthy:
                    raise ValueError(self.fault or 'Simulator unavailable')
                pose = self.world_pose(msg)
                scene = copy.deepcopy(self.scene)
                if not self.selected and not goal:
                    identity = max((p['id'] for p in scene['pedestrians']), default=0)+1
                    scene['pedestrians'].append(dict(id=identity, start=pose,
                        goal=[pose[0]+3*math.cos(pose[2]), pose[1]+3*math.sin(pose[2])],
                        desired_speed=.5, end_behavior='stop'))
                    selected = identity
                else:
                    p = self.selected_person(scene)
                    if goal:
                        p['goal'] = pose[:2]
                    else:
                        # Move start and heading together; preserve route length.
                        distance = math.hypot(p['goal'][0]-p['start'][0],p['goal'][1]-p['start'][1])
                        p['start'] = pose
                        p['goal'] = [pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2])]
                    selected = self.selected
                self.replace(scene)
                self.selected = selected
                self.error = ''
            except (ValueError, rospy.ServiceException) as exc:
                self.error = str(exc)
            self.publish()

    def place_start(self, msg):
        self.place(msg)

    def place_goal(self, msg):
        self.place(msg, goal=True)

    def pick(self, msg):
        with self.lock:
            if msg.header.frame_id != 'world' or not all(math.isfinite(v) for v in (msg.point.x,msg.point.y)):
                self.error = 'Select in world frame'
            elif self.scene['pedestrians']:
                poses = self.snapshot.poses if self.snapshot and len(self.snapshot.poses)==len(self.scene['pedestrians']) else []
                def distance(i):
                    p=poses[i] if poses else Pose2D(*self.scene['pedestrians'][i]['start'])
                    return math.hypot(p.x-msg.point.x,p.y-msg.point.y)
                i=min(range(len(self.scene['pedestrians'])),key=distance)
                if distance(i) <= 1.:
                    self.selected=self.scene['pedestrians'][i]['id'];self.error=''
                else:
                    self.error='Click within 1m of a pedestrian'
            self.publish()

    def navigation_goal(self, _msg):
        with self.lock:
            if self.scene['start_on_goal'] and self.healthy and not self.fault:
                try:
                    self.replace(self.scene, remember=False)
                    self.call('start')
                    self.error=''
                except (ValueError,rospy.ServiceException) as exc:
                    self.error=str(exc)
                self.publish()

    def poll(self, _event):
        with self.lock:
            try:
                was_healthy = self.healthy
                self.call('state')
                if not was_healthy:
                    self.error = ''
            except (ValueError, rospy.ServiceException) as exc:
                self.error=str(exc)
                self.healthy=False
            self.publish()

    def publish(self):
        state=SocialSceneState()
        state.header.stamp=rospy.Time.now();state.header.frame_id='world'
        state.revision=self.revision;state.pedestrians=[message(p) for p in self.scene['pedestrians']]
        state.selected_id=self.selected;state.start_on_goal=self.scene['start_on_goal']
        state.ready=self.healthy and not self.desynchronized and not self.fault
        state.remember_last=self.remember;state.message=self.fault or self.error;state.persistence_error=self.persistence_error
        if self.snapshot:
            state.agent_ids=self.snapshot.agent_ids;state.poses=self.snapshot.poses
            state.phases=self.snapshot.phases;state.running=self.snapshot.running
        self.state_pub.publish(state);self.ready_pub.publish(Bool(state.ready))
        markers=MarkerArray()
        clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
        for i,p in enumerate(self.scene['pedestrians']):
            selected=p['id']==self.selected
            def marker(namespace,kind):
                m=Marker();m.header=state.header;m.ns=namespace;m.id=p['id'];m.action=Marker.ADD;m.type=kind
                m.pose.orientation.w=1.;m.color.r=1. if selected else .3;m.color.g=.8;m.color.b=.2
                m.color.a=.9;m.lifetime=rospy.Duration(.5)
                return m
            route=marker('social_route',Marker.LINE_STRIP);route.scale.x=.04 if selected else .02
            route.points=[Point(*p['start'][:2],.12),Point(*p['goal'],.12)];markers.markers.append(route)
            goal=marker('social_goal',Marker.SPHERE);goal.pose.position=Point(*p['goal'],.15)
            goal.scale.x=goal.scale.y=.2;goal.scale.z=.05;markers.markers.append(goal)
            current=state.poses[i] if i<len(state.poses) else Pose2D(*p['start'])
            body=marker('social_body',Marker.CYLINDER);body.pose.position=Point(current.x,current.y,.85)
            body.scale.x=body.scale.y=.5;body.scale.z=1.7;body.color.a=.35;markers.markers.append(body)
            arrow=marker('social_heading',Marker.ARROW);arrow.scale.x=.04;arrow.scale.y=.12;arrow.scale.z=.15
            arrow.points=[Point(*p['start'][:2],.25),Point(p['start'][0]+.65*math.cos(p['start'][2]),p['start'][1]+.65*math.sin(p['start'][2]),.25)]
            markers.markers.append(arrow)
            label=marker('social_label',Marker.TEXT_VIEW_FACING);label.scale.z=.22
            label.pose.position=Point(current.x,current.y,1.85)
            phase=state.phases[i] if i<len(state.phases) else 'waiting'
            label.text='#%d | %.2f m/s | %s'%(p['id'],p['desired_speed'],phase)
            markers.markers.append(label)
        self.marker_pub.publish(markers)


if __name__=='__main__':
    rospy.init_node('pedsim_scene')
    Editor()
    rospy.spin()
