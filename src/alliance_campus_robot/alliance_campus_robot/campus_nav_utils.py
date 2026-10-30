"""Pure-Python helpers (no ROS): location lookup and goal snapping."""
import os

import numpy as np
import yaml


def load_locations(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)['locations']


def find_location(text, locations):
    """Longest matching alias wins ('student activity' beats 'store')."""
    text = text.lower().strip()
    best, best_len = None, 0
    for key, v in locations.items():
        names = list(v.get('aliases', [])) + [key.replace('_', ' '), v['display_name'].lower()]
        for alias in names:
            if alias in text and len(alias) > best_len:
                best, best_len = (key, v), len(alias)
    return best


def load_pgm_map(yaml_path):
    """Return (free[H,W] bool with row 0 = lowest y, resolution, origin_x, origin_y)."""
    meta = yaml.safe_load(open(yaml_path))
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(img_path, 'rb') as f:
        data = f.read()
    parts = data.split(None, 4)           # P5 W H MAX <binary>
    assert parts[0] == b'P5', 'only binary PGM supported'
    w, h = int(parts[1]), int(parts[2])
    pix = np.frombuffer(parts[4][-w * h:], dtype=np.uint8).reshape(h, w)
    free = np.flipud(pix) > 250
    return free, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def snap_to_free(free, res, ox, oy, x, y, clearance=0.8, search=6.0):
    """Nearest point to (x, y) that is free with `clearance` metres around it."""
    r = int(round(clearance / res))
    s = int(round(search / res))
    cx, cy = int((x - ox) / res), int((y - oy) / res)
    x0, x1 = max(cx - s - r, 0), min(cx + s + r + 1, free.shape[1])
    y0, y1 = max(cy - s - r, 0), min(cy + s + r + 1, free.shape[0])
    win = free[y0:y1, x0:x1]
    ok = win.copy()
    h, w = win.shape
    pad = np.pad(win, r, constant_values=False)
    for dy in range(-r, r + 1, 2):
        for dx in range(-r, r + 1, 2):
            ok &= pad[r + dy:r + dy + h, r + dx:r + dx + w]
    ys, xs = np.nonzero(ok)
    if len(xs) == 0:
        return None
    d = np.hypot(xs + x0 - cx, ys + y0 - cy) * res
    j = int(d.argmin())
    return (float(ox + (xs[j] + x0 + 0.5) * res), float(oy + (ys[j] + y0 + 0.5) * res), float(d[j]))


def resolve_goal(text, locations, free, res, ox, oy):
    """Turn a place name into a drivable map point.

    Returns (display_name, x, y, moved_metres) or (None, message) on failure.
    """
    found = find_location(text, locations)
    if not found:
        return None, f'Unknown place: {text!r}'
    _, loc = found
    snapped = snap_to_free(free, res, ox, oy, loc['x'], loc['y'])
    if snapped is None:
        return None, f'No drivable spot near {loc["display_name"]}'
    x, y, moved = snapped
    return loc['display_name'], x, y, moved


def tune_nav2_params(p, obstacle_range=2.5):
    """Adapt Nav2's default params to this robot and make it steer AROUND obstacles.

    obstacle_range: obstacles are only noticed within this many metres of the robot.
    Pure function (dict in, same dict out) so it can be tested without ROS.
    """
    def rp(name):
        return p.setdefault(name, {}).setdefault('ros__parameters', {})

    # --- controller (DWB): speed limits + weigh obstacles more, path-hugging less
    ctrl = rp('controller_server')
    fp = ctrl.setdefault('FollowPath', {})
    fp.update({'max_vel_x': 0.4, 'max_speed_xy': 0.4, 'max_vel_theta': 0.8,
               'acc_lim_x': 1.0, 'decel_lim_x': -1.0, 'acc_lim_theta': 1.5,
               'decel_lim_theta': -1.5})
    fp.update({'BaseObstacle.scale': 0.5, 'PathDist.scale': 16.0, 'PathAlign.scale': 8.0})
    ctrl.setdefault('general_goal_checker', {}).update(
        {'xy_goal_tolerance': 0.4, 'yaw_goal_tolerance': 3.14})

    obs_src = {'marking': True, 'clearing': True, 'data_type': 'LaserScan', 'topic': '/scan',
               'obstacle_min_range': 0.0, 'raytrace_min_range': 0.0,
               'obstacle_max_range': float(obstacle_range),
               'raytrace_max_range': float(obstacle_range) + 0.5,
               'min_obstacle_height': 0.0, 'max_obstacle_height': 2.0}

    for cm in ('local_costmap', 'global_costmap'):
        r = p.setdefault(cm, {}).setdefault(cm, {}).setdefault('ros__parameters', {})
        # footprint of the real robot is ~0.9 x 0.88 m -> circle of 0.6 m
        r['robot_radius'] = 0.6
        r.pop('footprint', None)
        r.setdefault('inflation_layer', {}).update(
            {'inflation_radius': 0.9, 'cost_scaling_factor': 3.0})

        plugins = list(r.get('plugins', []))
        if cm == 'global_costmap':
            # make sure the global planner also sees new obstacles and plans around them
            if 'obstacle_layer' not in plugins and 'voxel_layer' not in plugins:
                i = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)
                plugins.insert(i, 'obstacle_layer')
                r['obstacle_layer'] = {'plugin': 'nav2_costmap_2d::ObstacleLayer',
                                       'enabled': True}
            r['plugins'] = plugins
            r['update_frequency'] = 5.0
            r['publish_frequency'] = 2.0
        else:
            r['update_frequency'] = 10.0
            r['publish_frequency'] = 5.0
            # the local window must reach at least as far as we want to see
            side = max(6.0, 2.0 * (float(obstacle_range) + 1.0))
            r['width'], r['height'] = int(round(side)), int(round(side))
            r['rolling_window'] = True

        for layer_name in plugins:
            if layer_name in ('obstacle_layer', 'voxel_layer'):
                layer = r.setdefault(layer_name, {})
                layer['observation_sources'] = 'scan'
                layer.setdefault('scan', {}).update(obs_src)
                layer['enabled'] = True

    rp('velocity_smoother').update(
        {'max_velocity': [0.5, 0.0, 1.0], 'min_velocity': [-0.5, 0.0, -1.0]})
    return p
