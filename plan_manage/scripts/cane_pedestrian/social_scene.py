"""Validated, portable pedsim editor configuration. No transient simulation state."""
import math
import yaml

MAX_PEOPLE = 5


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Expected a finite number')
    return float(value)


def person(config):
    try:
        identity = config['id']
        if isinstance(identity, bool) or not isinstance(identity, int) or not 1 <= identity <= 1000000:
            raise ValueError('Invalid pedestrian id')
        start = [number(v) for v in config['start']]
        goal = [number(v) for v in config['goal']]
        speed = number(config['desired_speed'])
        behavior = config['end_behavior']
        if (len(start) != 3 or len(goal) != 2 or not 0 <= speed <= 3 or
                any(abs(v) >= 14.5 for v in start[:2]+goal) or
                (speed > 0 and math.hypot(start[0]-goal[0], start[1]-goal[1]) <= .4) or
                behavior not in ('stop', 'return', 'repeat')):
            raise ValueError('Invalid start/goal, desired speed [0,3], or end behavior')
        start[2] = math.atan2(math.sin(start[2]), math.cos(start[2]))
        return dict(id=identity, start=start, goal=goal, desired_speed=speed, end_behavior=behavior)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('Incomplete pedestrian configuration') from exc


def validate(config):
    if (not isinstance(config, dict) or config.get('format') != 'cane_pedsim_scene_v1' or
            config.get('frame_id') != 'world' or config.get('world') != 'blank' or
            not isinstance(config.get('start_on_goal'), bool)):
        raise ValueError('Expected a world-frame blank-arena pedsim scene (v1)')
    people = config.get('pedestrians')
    if not isinstance(people, list) or len(people) > MAX_PEOPLE:
        raise ValueError('A scene supports zero through five pedestrians')
    people = [person(p) for p in people]
    if len({p['id'] for p in people}) != len(people):
        raise ValueError('Duplicate pedestrian id')
    for i, p in enumerate(people):
        for other in people[:i]:
            if math.hypot(p['start'][0]-other['start'][0], p['start'][1]-other['start'][1]) < .55:
                raise ValueError('Pedestrian starts must be at least 0.55 m apart')
    return dict(format='cane_pedsim_scene_v1', frame_id='world', world='blank',
                start_on_goal=config['start_on_goal'], pedestrians=people)


def loads(text):
    if len(text.encode('utf-8')) > 65536:
        raise ValueError('Scene file exceeds 64 KiB')
    try:
        return validate(yaml.safe_load(text))
    except yaml.YAMLError as exc:
        raise ValueError('Invalid scene YAML') from exc


def dumps(config):
    return yaml.safe_dump(validate(config), sort_keys=False, allow_unicode=True)


def from_startup(scene):
    people=[]
    for i,p in enumerate(scene['pedestrians']):
        dx=p['goal'][0]-p['start'][0];dy=p['goal'][1]-p['start'][1]
        people.append(dict(id=i+1, start=p['start']+[math.atan2(dy,dx)], goal=p['goal'],
                           desired_speed=p['desired_max_speed'], end_behavior='stop'))
    return validate(dict(format='cane_pedsim_scene_v1', frame_id='world', world='blank',
                         start_on_goal=True, pedestrians=people))
