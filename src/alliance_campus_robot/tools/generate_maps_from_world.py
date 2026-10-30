#!/usr/bin/env python3
"""Generate exact occupancy maps from the Gazebo world file (no ROS needed).

Outputs (in the output dir):
  campus_map.pgm/.yaml      true obstacles only (what a perfect LiDAR map shows)
  campus_nav_map.pgm/.yaml  road-restricted map for navigation: only roads,
                            spurs, parking and grounds are free; all else blocked
Usage: python3 generate_maps_from_world.py WORLD.sdf OUTDIR [locations.yaml]
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

RES = 0.05
X0, Y0 = -50.0, -45.0           # map origin (lower-left), metres
W, H = 2000, 1800               # 100 m x 90 m
LIDAR_Z = (0.2, 0.9)            # heights the robot's LiDAR can see


def floats(s):
    return [float(v) for v in (s or '0 0 0 0 0 0').split()]


def grid_xy():
    xs = X0 + (np.arange(W) + 0.5) * RES
    ys = Y0 + (np.arange(H) + 0.5) * RES
    return np.meshgrid(xs, ys)          # arrays shaped (H, W)


def shape_mask(gx, gy, shape, dims, cx, cy, yaw, grow=0.0):
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = c * (gx - cx) - s * (gy - cy)
    ly = s * (gx - cx) + c * (gy - cy)
    if shape == 'box':
        return (np.abs(lx) <= dims[0] / 2 + grow) & (np.abs(ly) <= dims[1] / 2 + grow)
    return lx * lx + ly * ly <= (dims[0] + grow) ** 2      # cylinder: dims=[radius]


def parse(world_path):
    root = ET.parse(world_path).getroot().find('world')
    solids, roads, trees, extras = [], [], [], []
    for m in root.findall('model'):
        name = m.get('name')
        mp = floats(m.findtext('pose'))
        for link in m.findall('link'):
            lp = floats(link.findtext('pose'))
            for kind in ('collision', 'visual'):
                for el in link.findall(kind):
                    g = el.find('geometry')
                    if g is None:
                        continue
                    sh = g.find('box')
                    if sh is not None:
                        dims = floats(sh.findtext('size'))[:3]
                        shape = 'box'
                    else:
                        sh = g.find('cylinder')
                        if sh is None:
                            continue
                        r, l = float(sh.findtext('radius')), float(sh.findtext('length'))
                        dims, shape = [r, l], 'cyl'
                    ep = floats(el.findtext('pose'))
                    cx, cy = mp[0] + lp[0] + ep[0], mp[1] + lp[1] + ep[1]
                    cz = mp[2] + lp[2] + ep[2]
                    yaw = mp[5] + lp[5] + ep[5]
                    height = dims[2] if shape == 'box' else dims[1]
                    zlo, zhi = cz - height / 2, cz + height / 2
                    item = (name, shape, dims, cx, cy, yaw)
                    if kind == 'collision' and zhi > LIDAR_Z[0] and zlo < LIDAR_Z[1]:
                        solids.append(item)
                    elif kind == 'visual' and el.get('name') == 'visual':
                        if name.endswith('road') or '_road' in name or name.startswith('road_to'):
                            roads.append(item)
                        elif name in ('parking_area', 'cricket_ground', 'sports_ground'):
                            extras.append(item)
                    if name.startswith('tree_') and el.get('name') == 'trunk':
                        trees.append((name, 'cyl', [0.5, 1], cx, cy, 0.0))
    return solids, roads, extras, trees


def write(outdir, base, img):
    img = np.flipud(img)                # PGM row 0 is the top
    with open(os.path.join(outdir, base + '.pgm'), 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        f.write(img.astype(np.uint8).tobytes())
    with open(os.path.join(outdir, base + '.yaml'), 'w') as f:
        f.write(f'image: {base}.pgm\nmode: trinary\nresolution: {RES}\n'
                f'origin: [{X0}, {Y0}, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n')


def main():
    world, outdir = sys.argv[1], sys.argv[2]
    os.makedirs(outdir, exist_ok=True)
    gx, gy = grid_xy()
    solids, roads, extras, trees = parse(world)

    occ = np.zeros((H, W), bool)
    for _, sh, d, cx, cy, yaw in solids:
        occ |= shape_mask(gx, gy, sh, d, cx, cy, yaw)

    # ---- true map: free inside the boundary walls, unknown outside
    inside = (np.abs(gx) < 47.75) & (gy > -37.75) & (gy < 37.75)
    inside |= (np.abs(gx) < 8) & (gy <= -37.75) & (gy > -40)       # gate opening
    img = np.full((H, W), 205, np.uint8)
    img[inside] = 254
    img[occ] = 0
    write(outdir, 'campus_map', img)

    # ---- navigation map: only drivable areas are free
    drive = np.zeros((H, W), bool)
    for _, sh, d, cx, cy, yaw in roads + extras:
        drive |= shape_mask(gx, gy, sh, d, cx, cy, yaw, grow=0.6)   # bridges <=1.2 m gaps
    block = occ.copy()
    for _, sh, d, cx, cy, yaw in trees:
        block |= shape_mask(gx, gy, sh, d, cx, cy, yaw)
    nav = np.zeros((H, W), np.uint8)          # default: blocked
    nav[drive & ~block] = 254
    write(outdir, 'campus_nav_map', nav)

    print(f'solids={len(solids)} road/extra pieces={len(roads) + len(extras)} trees={len(trees)}')

    if len(sys.argv) > 3:
        import yaml
        locs = yaml.safe_load(open(sys.argv[3]))['locations']
        print('\nDestination check on navigation map:')
        for k, v in locs.items():
            ix = int((v['x'] - X0) / RES)
            iy = int((v['y'] - Y0) / RES)
            ok = nav[iy, ix] == 254
            # distance to nearest free cell
            msg = 'OK (free)'
            if not ok:
                ys, xs = np.nonzero(nav == 254)
                d = np.hypot(xs - ix, ys - iy) * RES
                j = int(d.argmin())
                fx, fy = X0 + (xs[j] + 0.5) * RES, Y0 + (ys[j] + 0.5) * RES
                msg = f'BLOCKED - nearest free point ({fx:.1f}, {fy:.1f}), {d[j]:.1f} m away'
            print(f'  {k:24s} ({v["x"]:6.1f},{v["y"]:6.1f})  {msg}')


if __name__ == '__main__':
    main()
