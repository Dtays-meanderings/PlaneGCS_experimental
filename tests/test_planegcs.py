import sys
import os
import math

# Append build/Release to path so we can import the built pyd
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'build', 'Release'))

import planegcs

def test_distance_solving():
    print("Testing Distance Solving in Python...")
    sys_gcs = planegcs.System()

    # Parameters
    x1 = planegcs.Parameter(0.0)
    y1 = planegcs.Parameter(0.0)
    x2 = planegcs.Parameter(3.0)
    y2 = planegcs.Parameter(4.0)

    # Points
    p1 = planegcs.Point(x1, y1)
    p2 = planegcs.Point(x2, y2)

    # Distance target
    target_dist = planegcs.Parameter(10.0)

    # Add distance constraint
    sys_gcs.addConstraintP2PDistance(p1, p2, target_dist)

    # Declare unknowns
    sys_gcs.declareUnknowns([x2, y2])

    sys_gcs.initSolution()
    status = sys_gcs.solve()
    sys_gcs.applySolution()

    print(f"Solver Status: {status}")
    print(f"p1: ({x1.value}, {y1.value})")
    print(f"p2: ({x2.value}, {y2.value})")

    # The distance should be 10.0
    dist = math.sqrt((x2.value - x1.value)**2 + (y2.value - y1.value)**2)
    print(f"Computed Distance: {dist}")
    
    assert status == planegcs.SolveStatus.Success
    assert math.isclose(dist, 10.0, abs_tol=1e-5)
    print("Python test passed successfully!")

if __name__ == '__main__':
    test_distance_solving()
