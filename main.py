import math
import matplotlib.pyplot as plt
import numpy as np

# --- Geometry utilities ---

def point_in_polygon(point, poly):
    """
    Determines if a point is inside a polygon using the ray-casting algorithm.
    poly: list of (x, y) tuples.
    Returns True if point is inside poly.
    """
    x, y = point
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < ((xj - xi) * (y - yi) / (yj - yi + 1e-10)) + xi):
            inside = not inside
        j = i
    return inside

def line_intersection(p, q, a, b):
    """
    Compute the intersection of line segment p->q with segment a->b.
    Returns (t, u) if they intersect, where:
      p + t*(q-p) == a + u*(b-a),
      t, u in [0, 1] for intersection on both segments.
    Returns None if no intersection.
    """
    (x1, y1), (x2, y2) = p, q
    (x3, y3), (x4, y4) = a, b
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None  # Parallel
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom
    return t, u

def polygon_line_intersections(p, q, poly):
    """
    For a given line segment p->q and a polygon (list of vertices),
    compute the parameter t values along the segment where intersections occur.
    Returns a sorted list of t values (if within [0,1]).
    """
    ts = []
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i+1) % n]
        result = line_intersection(p, q, a, b)
        if result:
            t, u = result
            if 0 <= t <= 1 and 0 <= u <= 1:
                ts.append(t)
    return sorted(ts)

def interpolate(p, t, q):
    """Linear interpolation between p and q with parameter t."""
    return (p[0]*(1-t) + q[0]*t, p[1]*(1-t) + q[1]*t)

def perpendicular_direction(p, q):
    """
    Returns a unit vector perpendicular to the vector from p to q.
    There are two possible directions; this returns one.
    """
    dx, dy = q[0]-p[0], q[1]-p[1]
    # One perpendicular: (-dy, dx)
    mag = math.hypot(dx, dy)
    if mag == 0:
        return (0,0)
    return (-dy/mag, dx/mag)

def adjust_point_out_of_polygon(mid, p, q, poly, step=0.2, max_iter=20):
    """
    From candidate midpoint 'mid' (which lies inside poly), 
    sample along the perpendicular to p->q in both directions until a point is found that is not inside poly.
    Returns the adjusted point with minimal displacement.
    """
    perp = perpendicular_direction(p, q)
    # Try both directions: +perp and -perp
    candidates = []
    for sign in [1, -1]:
        candidate = mid
        for i in range(max_iter):
            if not point_in_polygon(candidate, poly):
                candidates.append((candidate, i*step))
                break
            # Move candidate further in the chosen direction
            candidate = (candidate[0] + sign * step * perp[0], candidate[1] + sign * step * perp[1])
    # If none found, return original (should not happen)
    if not candidates:
        return mid
    # Choose candidate with minimal displacement
    candidates.sort(key=lambda x: x[1])
    return candidates[0][0]

# --- Refinement function with visualization support ---

segments = []  # To record segments for plotting; each is (p, q, depth)

def refine_path(p, q, obstacles, tolerance=0.1, depth=0):
    """
    Recursively refine the path from p to q.
    For each obstacle (polygon) in obstacles, if the segment p->q intersects the obstacle,
    we compute the t values of intersection and take the midpoint of the collision interval.
    Then, using perpendicular raycasts, we adjust that midpoint out of the polygon.
    We then recursively refine p->mid and mid->q.
    """
    segments.append((p, q, depth))
    
    if is_collision_free_poly(p, q, obstacles):
        return [p, q]
    
    # For each polygon obstacle:
    for poly in obstacles:
        ts = polygon_line_intersections(p, q, poly)
        if len(ts) >= 2:
            # The segment from t1 to t2 lies within the obstacle.
            t_in, t_out = ts[0], ts[-1]
            t_mid = (t_in + t_out) / 2
            mid_candidate = interpolate(p, t_mid, q)
            # Adjust this candidate out of the polygon using perpendicular sampling:
            mid_adjusted = adjust_point_out_of_polygon(mid_candidate, p, q, poly)
            # Recursively refine the two segments:
            left_path = refine_path(p, mid_adjusted, obstacles, tolerance, depth+1)
            right_path = refine_path(mid_adjusted, q, obstacles, tolerance, depth+1)
            return left_path[:-1] + right_path
    # Fallback: if no polygon provided a clear intersection interval,
    # use simple midpoint and adjust it out if needed.
    mid = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
    for poly in obstacles:
        if point_in_polygon(mid, poly):
            mid = adjust_point_out_of_polygon(mid, p, q, poly)
    if math.hypot(p[0]-mid[0], p[1]-mid[1]) < tolerance:
        return [p, mid, q]
    left_path = refine_path(p, mid, obstacles, tolerance, depth+1)
    right_path = refine_path(mid, q, obstacles, tolerance, depth+1)
    return left_path[:-1] + right_path

def is_collision_free_poly(p, q, obstacles, steps=20):
    """
    Check if the line segment from p to q is collision-free with respect to a list of polygon obstacles.
    We sample along the segment.
    """
    for i in range(steps + 1):
        t = i / steps
        point = interpolate(p, t, q)
        for poly in obstacles:
            if point_in_polygon(point, poly):
                return False
    return True

# --- Example usage and visualization ---

if __name__ == '__main__':
    # Define start and goal points
    start = (0, 0)
    goal = (10, 10)
    
    # Define obstacles as polygons.
    # Example: one rectangle and one triangle.
    obstacle1 = [(4, 4), (7, 4), (7, 7), (4, 7)]   # A square/rectangle obstacle
    obstacle2 = [(2, 8), (3, 11), (1, 10)]           # A triangle obstacle
    obstacle3 = [(2,2), (3,6), (4,2)]
    obstacle4 = [(6.5,8), (6.5,9.5), (8,9.5), (8, 8)]
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]
    
    # Clear segments for plotting:
    segments.clear()
    
    path = refine_path(start, goal, obstacles, tolerance=0.1)
    print("Computed path:", path)
    
    # --- Plotting ---
    plt.figure(figsize=(6,6))
    
    # Plot obstacles as filled polygons:
    for poly in obstacles:
        polygon_patch = plt.Polygon(poly, closed=True, color='gray', alpha=0.4)
        plt.gca().add_patch(polygon_patch)
    
    # Create a colormap for different recursion depths:
    cmap = plt.get_cmap('viridis')
    max_depth = max(seg[2] for seg in segments) if segments else 1
    
    # Plot each recorded segment with a color based on its depth.
    for (p_seg, q_seg, depth) in segments:
        color = cmap(depth / (max_depth + 1))
        plt.plot([p_seg[0], q_seg[0]], [p_seg[1], q_seg[1]], color=color, linewidth=1, linestyle="dashed")

    plt.plot((0,0), (0,0), linewidth=3, label="Iterative Paths", linestyle="dashed")
    
    # Plot the final refined path over top in black:
    xs, ys = zip(*path)
    plt.plot(xs, ys, 'k-', linewidth=3, label="Final Path")
    #plt.plot((0,10),(0,10), 'k-', linewidth=3, label="Final Path")
    
    # Plot start and goal points:
    plt.plot(start[0], start[1], 'go', markersize=10, label="Start")
    plt.plot(goal[0], goal[1], 'ro', markersize=10, label="Goal")
    
    plt.legend()
    plt.title("Iterative In-Obstacle Midpoint Refinement")# Iterative In-Obstacle Midpoint Refinement
    plt.xlim(-1, 11)
    plt.ylim(-1, 12)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()

