import numpy as np
import matplotlib.pyplot as plt
from road.course_library import course_library
from road.construct_course import construct_course

def select_course(course: str = "", resolution: float = 10.0, preview: bool = True):
    """Port of SelectCourse.m (non-interactive by default).
    Returns (x_course, z_course, course_data, course_info).
    """
    courses = course_library()
    if not courses:
        raise RuntimeError("No courses found in course library")
    if course:
        if course not in courses:
            raise ValueError(f"Course '{course}' not found. Available: {', '.join(courses.keys())}")
        key = course
    else:
        key = "extreme"
    course_data = courses[key]
    print(f"Selected: {course_data['name']}")
    print(f"Description: {course_data['description']}")
    x_course, z_course, course_info = construct_course(course_data['segments'], resolution=resolution)
    course_data = dict(course_data)
    course_data['total_distance'] = course_info['total_distance']
    course_data['segment_boundaries'] = course_info['segment_boundaries']
    print('Course generated successfully!')
    print(f"Total distance: {course_info['total_distance']:.1f} m")
    print(f"Number of points: {len(x_course)}")
    print(f"Resolution: {resolution:.1f} points/m")
    if preview:
        _show_course_preview(x_course, z_course, course_data, course_info)
    return x_course, z_course, course_data, course_info

def _show_course_preview(x_course, z_course, course_data, course_info):
    fig = plt.figure(figsize=(12,6))
    fig.suptitle(f"Course Preview: {course_data['name']}")
    ax = fig.add_subplot(2,1,1)
    ax.plot(x_course, z_course, linewidth=2)
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Elevation (m)')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    for xb in course_info['segment_boundaries']:
        zb = float(np.interp(xb, x_course, z_course))
        ax.plot([xb], [zb], 'ro', markersize=6)
    ax.plot([x_course[0]], [z_course[0]], 'go', markersize=8)
    ax.plot([x_course[-1]], [z_course[-1]], 'ro', markersize=8)
    segs = course_data['segments']
    bounds = course_info['segment_boundaries']
    for i in range(len(segs)):
        if i == 0:
            x_start = x_course[0]
            x_end = bounds[0]
        else:
            x_start = bounds[i-1]
            x_end = bounds[i]
        xm = 0.5*(x_start + x_end)
        zm = float(np.interp(xm, x_course, z_course))
        ax.text(xm, zm + 2.0, f"S{i+1}", ha='center', va='bottom', fontsize=10, fontweight='bold')
    ax.text(x_course[0], z_course[0] + 0.2, 'START', ha='center', fontweight='bold')
    ax.text(x_course[-1], z_course[-1] + 0.2, 'END', ha='center', fontweight='bold')
    ax.set_title(f"{course_data['name']} - Course Overview")
    ax2 = fig.add_subplot(2,1,2)
    ax2.axis('off')
    header = ['Segment','Type','Distance (m)','Gradient','Features']
    lines = [header]
    for i, seg in enumerate(segs, start=1):
        typ = seg['type'].lower()
        features = ''
        if typ == 'bumpy' and 'height' in seg and 'frequency' in seg:
            features = f"Height: {seg['height']:.2f} m, Freq: {seg['frequency']:.1f}/m"
        elif typ in ('jump','drop') and 'height' in seg:
            features = f"Height: {seg['height']:.2f} m"
        elif typ == 'staircase' and all(k in seg for k in ('height','num_steps')):
            features = f"Height: {seg.get('height',0):.2f} m, Steps: {seg['num_steps']}"
        elif typ == 'trapezium' and 'flat_length' in seg:
            features = f"Height: {seg.get('height',0):.2f} m, Flat: {seg['flat_length']:.1f} m"
        elif typ == 'gap_jump' and 'gap_width' in seg:
            features = f"Height: {seg.get('height',0):.2f} m, Gap: {seg['gap_width']:.1f} m"
        elif typ == 'rock_garden' and 'rock_density' in seg:
            features = f"Height: {seg.get('height',0):.2f} m, Rocks/m: {seg['rock_density']:.1f}"
        elif typ == 'root_section' and 'root_density' in seg:
            features = f"Height: {seg.get('height',0):.2f} m, Roots/m: {seg['root_density']:.1f}"
        elif typ == 'curve' and 'R_curve' in seg:
            features = f"R={seg['R_curve']:.1f} m, {seg['gradient']:.2f}→{seg['final_gradient']:.2f}"
        lines.append([str(i), seg['type'], f"{seg['distance']:.1f}", f"{seg['gradient']:.3f}", features])
    col_widths = [8, 12, 13, 10, 40]
    def fmt_row(cols):
        pads = [str(cols[i]).ljust(col_widths[i]) for i in range(len(cols))]
        return ' '.join(pads)
    text = '\n'.join(fmt_row(r) for r in lines)
    ax2.text(0.5, 0.5, text, ha='center', va='center', family='monospace')
    fig.tight_layout()
    plt.show()
