import math
import matplotlib.pyplot as plt
import os

# --- Geometry utilities ---

def point_in_polygon(point, poly):
    """
    Ray-casting algorithm: returns True if point (x, y) is inside polygon 'poly'
    (poly is a list of (x, y) vertices).
    """
    x, y = point
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi):
            inside = not inside
        j = i
    return inside

def line_intersection(p, q, a, b):
    """
    Computes the intersection of line segment p→q with segment a→b.
    Returns (t, u) such that: p + t*(q-p) == a + u*(b-a).
    Only returns a valid result if t and u are in [0, 1]. Otherwise, returns None.
    """
    (x1, y1), (x2, y2) = p, q
    (x3, y3), (x4, y4) = a, b
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom
    return t, u

def polygon_line_intersections(p, q, poly):
    """
    For a line segment p→q and a polygon (list of vertices),
    return a sorted list of parameter t values (from 0 to 1) for which p+t*(q-p) lies on an edge.
    """
    ts = []
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        result = line_intersection(p, q, a, b)
        if result:
            t, u = result
            if 0 <= t <= 1 and 0 <= u <= 1:
                ts.append(t)
    return sorted(ts)

def interpolate(p, t, q):
    """Linearly interpolates between p and q using parameter t."""
    return (p[0]*(1-t) + q[0]*t, p[1]*(1-t) + q[1]*t)

def perpendicular_direction(p, q):
    """
    Returns a unit vector perpendicular to the vector from p to q.
    (One of the two possibilities.)
    """
    dx, dy = q[0]-p[0], q[1]-p[1]
    mag = math.hypot(dx, dy)
    if mag == 0:
        return (0, 0)
    return (-dy/mag, dx/mag)

def adjust_point_out_of_polygon(mid, p, q, poly, step=0.2, max_iter=20):
    """
    Given a candidate midpoint 'mid' (which lies inside poly), this function uses
    perpendicular raycasts from mid to determine which side of the obstacle requires the
    smallest displacement to move out of poly.
    """
    perp = perpendicular_direction(p, q)
    candidates = []
    for sign in [1, -1]:
        candidate = mid
        for i in range(max_iter):
            if not point_in_polygon(candidate, poly):
                candidates.append((candidate, i * step))
                break
            candidate = (candidate[0] + sign * step * perp[0],
                         candidate[1] + sign * step * perp[1])
    if not candidates:
        return mid
    # Choose the candidate with minimal displacement
    candidates.sort(key=lambda x: x[1])
    return candidates[0][0]

# --- Global variables for visualization ---
frame_counter = 0   # for saving frames
global_start = None # will be set in main
global_goal = None

def save_frame(p, q, iteration, obstacles, start, goal):
    """
    Plots the obstacles, start and goal points, and only the current segment (p->q)
    with the title indicating the current iteration. Saves the figure as a PNG.
    """
    global frame_counter
    plt.figure(figsize=(6,6))
    
    # Plot each obstacle
    for poly in obstacles:
        polygon_patch = plt.Polygon(poly, closed=True, color='gray', alpha=0.4)
        plt.gca().add_patch(polygon_patch)
    
    # Plot start and goal
    plt.plot(start[0], start[1], 'go', markersize=10, label="Start")
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label="Goal")
    
    # Plot only the current segment (p, q)
    plt.plot([p[0], q[0]], [p[1], q[1]], 'b-', linewidth=3)
    
    plt.title(f"Iteration {iteration}")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.xlim(-1, 11)
    plt.ylim(-1, 12)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    
    filename = f"frame_{frame_counter:03d}.png"
    plt.savefig(filename)
    plt.close()
    frame_counter += 1

# --- Refinement function with iterative frame saving ---
def refine_path(p, q, obstacles, tolerance=0.1, iteration=0):
    """
    Recursively refines the path from p to q. If the segment p->q intersects any polygon,
    finds the intersection interval, computes its midpoint, adjusts it out of collision,
    saves a frame showing the current segment, and recurses on the two subsegments.
    """
    # Save the current segment as a frame (only the current iteration line)
    save_frame(p, q, iteration, obstacles, global_start, global_goal)
    
    # If the segment is collision-free, return it.
    if is_collision_free_poly(p, q, obstacles):
        return [p, q]
    
    for poly in obstacles:
        ts = polygon_line_intersections(p, q, poly)
        if len(ts) >= 2:
            t_in, t_out = ts[0], ts[-1]
            t_mid = (t_in + t_out) / 2
            mid_candidate = interpolate(p, t_mid, q)
            mid_adjusted = adjust_point_out_of_polygon(mid_candidate, p, q, poly)
            left_path = refine_path(p, mid_adjusted, obstacles, tolerance, iteration + 1)
            right_path = refine_path(mid_adjusted, q, obstacles, tolerance, iteration + 1)
            return left_path[:-1] + right_path
    # Fallback: if no obstacle provided a clear intersection interval, use simple midpoint.
    mid = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
    for poly in obstacles:
        if point_in_polygon(mid, poly):
            mid = adjust_point_out_of_polygon(mid, p, q, poly)
    if math.hypot(p[0]-mid[0], p[1]-mid[1]) < tolerance:
        return [p, mid, q]
    left_path = refine_path(p, mid, obstacles, tolerance, iteration + 1)
    right_path = refine_path(mid, q, obstacles, tolerance, iteration + 1)
    return left_path[:-1] + right_path

def is_collision_free_poly(p, q, obstacles, steps=20):
    """
    Checks if the line segment from p to q is collision-free for all polygon obstacles.
    """
    for i in range(steps + 1):
        t = i / steps
        point = interpolate(p, t, q)
        for poly in obstacles:
            if point_in_polygon(point, poly):
                return False
    return True

# --- Main function ---
if __name__ == '__main__':
    # Define start and goal points
    global_start = (0, 0)
    global_goal = (10, 10)
    
    # Define obstacles as polygons (list of vertices)
    obstacle1 = [(4, 4), (7, 4), (7, 7), (4, 7)]   # A square/rectangle obstacle
    obstacle2 = [(2, 8), (3, 11), (1, 10)]           # A triangle obstacle
    obstacle3 = [(2,2), (3,6), (4,2)]
    obstacle4 = [(6.5,8), (6.5,9.5), (8,9.5), (8, 8)]
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]

    # Clear frame counter
    frame_counter = 0
    
    path = refine_path(global_start, global_goal, obstacles, tolerance=0.1, iteration=0)
    print("Computed path:", path)
    
    # For overall visualization, plot the final refined path:
    plt.figure(figsize=(6,6))
    for poly in obstacles:
        polygon_patch = plt.Polygon(poly, closed=True, color='gray', alpha=0.4)
        plt.gca().add_patch(polygon_patch)
    xs, ys = zip(*path)
    plt.plot(xs, ys, 'k-', linewidth=3, label="Final Path")
    plt.plot(global_start[0], global_start[1], 'go', markersize=10, label="Start")
    plt.plot(global_goal[0], global_goal[1], 'ro', markersize=10, label="Goal")
    plt.title("Final Refined Path")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.xlim(-1, 11)
    plt.ylim(-1, 12)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.savefig("final_path.png")
    plt.show()

