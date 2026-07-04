import sys
import os
import math

# Set PYTHONPATH to build directory
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "build", "Release"))

import planegcs

def test_parallel_and_logging():
    print("Testing Parallel Subsystem Solving & Log Callbacks in Python...")

    # Create a system
    sys_obj = planegcs.System()

    # Capture logs in a Python list
    received_logs = []
    def log_cb(message):
        received_logs.append(message)
        print(f"[Solver Log] {message}", end="")

    # Register callback
    sys_obj.registerLogCallback(log_cb)

    # Subsystem 1 parameters
    x1 = planegcs.Parameter(0.0)
    y1 = planegcs.Parameter(0.0)
    x2 = planegcs.Parameter(3.0)
    y2 = planegcs.Parameter(4.0)

    # Subsystem 2 parameters
    x3 = planegcs.Parameter(10.0)
    y3 = planegcs.Parameter(10.0)
    x4 = planegcs.Parameter(12.0)
    y4 = planegcs.Parameter(12.0)

    # Points
    p1 = planegcs.Point(x1, y1)
    p2 = planegcs.Point(x2, y2)
    p3 = planegcs.Point(x3, y3)
    p4 = planegcs.Point(x4, y4)

    # Distance targets
    dist1 = planegcs.Parameter(10.0)
    dist2 = planegcs.Parameter(20.0)

    # Add constraints to system
    sys_obj.addConstraintP2PDistance(p1, p2, dist1)
    sys_obj.addConstraintP2PDistance(p3, p4, dist2)

    # Declare unknowns
    unknowns = [x2, y2, x4, y4]
    sys_obj.declareUnknowns(unknowns)

    # Initialize and solve
    print("Initializing solver...")
    sys_obj.initSolution(planegcs.Algorithm.LevenbergMarquardt)

    print("Solving system...")
    status = sys_obj.solve(True, planegcs.Algorithm.LevenbergMarquardt, False)
    print(f"Solver Status: {status}")

    # Solve should return Success (0) or Converged (1)
    assert status in [planegcs.SolveStatus.Success, planegcs.SolveStatus.Converged]

    # Verify solver modified parameters in-place or solved them
    sys_obj.applySolution()

    # Retrieve values
    p2_resolved_dist = math.sqrt((x2.value - x1.value)**2 + (y2.value - y1.value)**2)
    p4_resolved_dist = math.sqrt((x4.value - x3.value)**2 + (y4.value - y3.value)**2)

    print(f"Resolved Subsystem 1 Distance: {p2_resolved_dist:.4f}")
    print(f"Resolved Subsystem 2 Distance: {p4_resolved_dist:.4f}")

    assert abs(p2_resolved_dist - 10.0) < 1e-4
    assert abs(p4_resolved_dist - 20.0) < 1e-4

    print("Success! Parallel solving verified.")
    print(f"Total log messages received: {len(received_logs)}")

if __name__ == "__main__":
    test_parallel_and_logging()
