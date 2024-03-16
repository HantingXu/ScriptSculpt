from svg.path import parse_path, Line, CubicBezier, QuadraticBezier
import matplotlib.pyplot as plt
import numpy as np
import math

# Your SVG path data
path_data = d="M60 830h182v-382h2c42 56 93 82 164 82c130 0 193 -86 193 -216v-314h-182v263c0 60 -9 122 -82 122c-72 0 -95 -62 -95 -122v-263h-182v830z"
# Parse the SVG path
path = parse_path(path_data)

# Function to interpolate between two points
def interpolate_point(start, end, t):
    return start + (end - start) * t

# Process each segment and extract or calculate control points
def process_segment(segment):
    if isinstance(segment, CubicBezier):
        # For Cubic Bezier, return the control points directly
        return [segment.start, segment.control1, segment.control2, segment.end]
    elif isinstance(segment, QuadraticBezier):
        # For Quadratic Bezier, convert to cubic by calculating equivalent cubic control points
        control1 = segment.start + 2/3 * (segment.control - segment.start)
        control2 = segment.end + 2/3 * (segment.control - segment.end)
        return [segment.start, control1, control2, segment.end]
    elif isinstance(segment, Line):
        # For Line, calculate control points at 1/3 and 2/3 of the distance
        control1 = interpolate_point(segment.start, segment.end, 1/3)
        control2 = interpolate_point(segment.start, segment.end, 2/3)
        return [segment.start, control1, control2, segment.end]
    else:
        # If there are any other segment types, handle them as needed
        print("something weird")
        return []

# Process segments
segments_control_points = [process_segment(segment) for segment in path]

# Convert control points to a friendly format and print them
for segment_points in segments_control_points:
    formatted_points = [(p.real, p.imag) for p in segment_points]
    print(formatted_points, end=",")
    print()

def bezier_curve(control_points, num_points=100):
    """Calculate Bezier curve coordinates for a set of control points."""
    n = len(control_points) - 1
    t = np.linspace(0, 1, num_points)
    curve_points = np.zeros((num_points, 2))
    for i in range(n + 1):
        binomial_coeff = math.factorial(n) / (math.factorial(i) * math.factorial(n - i))
        for j, (x, y) in enumerate(control_points):
            curve_points[:, 0] += binomial_coeff * (1 - t) ** (n - j) * t ** j * x if i == j else 0
            curve_points[:, 1] += binomial_coeff * (1 - t) ** (n - j) * t ** j * y if i == j else 0
    return curve_points

# Example control points for segments
segments_control_points = [
[(60.0, 830.0), (120.66666666666666, 830.0), (181.33333333333331, 830.0), (242.0, 830.0)],
[(242.0, 830.0), (242.0, 702.6666666666666), (242.0, 575.3333333333334), (242.0, 448.0)],
[(242.0, 448.0), (242.66666666666666, 448.0), (243.33333333333334, 448.0), (244.0, 448.0)],
[(244.0, 448.0), (286.0, 504.0), (337.0, 530.0), (408.0, 530.0)],
[(408.0, 530.0), (538.0, 530.0), (601.0, 444.0), (601.0, 314.0)],
[(601.0, 314.0), (601.0, 209.33333333333334), (601.0, 104.66666666666669), (601.0, 0.0)],
[(601.0, 0.0), (540.3333333333334, 0.0), (479.6666666666667, 0.0), (419.0, 0.0)],
[(419.0, 0.0), (419.0, 87.66666666666666), (419.0, 175.33333333333331), (419.0, 263.0)],
[(419.0, 263.0), (419.0, 323.0), (410.0, 385.0), (337.0, 385.0)],
[(337.0, 385.0), (265.0, 385.0), (242.0, 323.0), (242.0, 263.0)],
[(242.0, 263.0), (242.0, 175.33333333333334), (242.0, 87.66666666666669), (242.0, 0.0)],
[(242.0, 0.0), (181.33333333333334, 0.0), (120.66666666666667, 0.0), (60.0, 0.0)],
[(60.0, 0.0), (60.0, 276.66666666666663), (60.0, 553.3333333333333), (60.0, 830.0)]
]

# Plotting each Bezier curve segment
plt.figure(figsize=(10, 10))
for segment_points in segments_control_points:
    curve = bezier_curve(segment_points, 100)
    plt.plot(curve[:, 0], curve[:, 1])

# Optionally, plot control points
for segment_points in segments_control_points:
    control_points = np.array(segment_points)
    plt.scatter(control_points[:, 0], control_points[:, 1], s=10, color='red')

formatted_segments = ""
for segment in segments_control_points:
    formatted_segments += " ".join(f"vec2({x}, {y})," for x, y in segment) + "\n "

print(formatted_segments)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Bezier Curve Segments')
plt.legend()
plt.axis('equal')
#plt.savefig('z.png')
plt.show()
plt.close()