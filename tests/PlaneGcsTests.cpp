#include <gtest/gtest.h>
#include "GCS.h"
#include "Geo.h"
#include <cmath>

TEST(PlaneGcsTests, TestDistanceSolving) {
    GCS::System system;

    // Point A (fixed at 0, 0)
    double ax = 0.0;
    double ay = 0.0;

    // Point B (free, initially at 3, 0)
    double bx = 3.0;
    double by = 0.0;

    double target_distance = 5.0;

    GCS::Point pA(&ax, &ay);
    GCS::Point pB(&bx, &by);

    system.addConstraintP2PDistance(pA, pB, &target_distance, 0, true);

    GCS::VEC_pD unknowns;
    unknowns.push_back(&bx);
    unknowns.push_back(&by);
    system.declareUnknowns(unknowns);

    system.initSolution();
    int status = system.solve();
    system.applySolution();

    EXPECT_EQ(status, GCS::SolveStatus::Success);

    double dx = bx - ax;
    double dy = by - ay;
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 5.0, 1e-6);
}

TEST(PlaneGcsTests, TestHorizontalVertical) {
    GCS::System system;

    // Point A (fixed at 1, 1)
    double ax = 1.0;
    double ay = 1.0;

    // Point B (free, initially at 2, 3)
    double bx = 2.0;
    double by = 3.0;

    GCS::Point pA(&ax, &ay);
    GCS::Point pB(&bx, &by);

    // Vertical constraint between A and B (aligns their X coordinates)
    system.addConstraintVertical(pA, pB, 0, true);

    GCS::VEC_pD unknowns;
    unknowns.push_back(&bx);
    unknowns.push_back(&by);
    system.declareUnknowns(unknowns);

    system.initSolution();
    int status = system.solve();
    system.applySolution();

    EXPECT_EQ(status, GCS::SolveStatus::Success);
    EXPECT_NEAR(bx, 1.0, 1e-6);
}

TEST(PlaneGcsTests, TestRectangleSolving) {
    GCS::System system;

    // Line 0 parameters (starts fixed at 10.0, 20.0)
    double x0a = 10.0;
    double y0a = 20.0;
    double x0b = 140.0;
    double y0b = 25.0;

    // Line 1 parameters
    double x1a = 141.0;
    double y1a = 24.0;
    double x1b = 165.0;
    double y1b = 95.0;

    // Line 2 parameters
    double x2a = 164.0;
    double y2a = 96.0;
    double x2b = 15.0;
    double y2b = 105.0;

    // Line 3 parameters
    double x3a = 16.0;
    double y3a = 104.0;
    double x3b = 11.0;
    double y3b = 21.0;

    GCS::Point p0a(&x0a, &y0a);
    GCS::Point p0b(&x0b, &y0b);
    GCS::Point p1a(&x1a, &y1a);
    GCS::Point p1b(&x1b, &y1b);
    GCS::Point p2a(&x2a, &y2a);
    GCS::Point p2b(&x2b, &y2b);
    GCS::Point p3a(&x3a, &y3a);
    GCS::Point p3b(&x3b, &y3b);

    // Corner coincident constraints (join the endpoints of the lines)
    system.addConstraintP2PCoincident(p0b, p1a, 0, true);
    system.addConstraintP2PCoincident(p1b, p2a, 0, true);
    system.addConstraintP2PCoincident(p2b, p3a, 0, true);
    system.addConstraintP2PCoincident(p3b, p0a, 0, true);

    // Horizontal constraints on Line 0 and Line 2
    system.addConstraintHorizontal(p0a, p0b, 0, true);
    system.addConstraintHorizontal(p2a, p2b, 0, true);

    // Vertical constraints on Line 1 and Line 3
    system.addConstraintVertical(p1a, p1b, 0, true);
    system.addConstraintVertical(p3a, p3b, 0, true);

    // Distance constraints: Width = 150.0, Height = 80.0
    double target_width = 150.0;
    double target_height = 80.0;
    system.addConstraintP2PDistance(p0a, p0b, &target_width, 0, true);
    system.addConstraintP2PDistance(p1a, p1b, &target_height, 0, true);

    // Free parameters (excluding x0a and y0a which are fixed at the origin)
    GCS::VEC_pD unknowns;
    unknowns.push_back(&x0b); unknowns.push_back(&y0b);
    unknowns.push_back(&x1a); unknowns.push_back(&y1a);
    unknowns.push_back(&x1b); unknowns.push_back(&y1b);
    unknowns.push_back(&x2a); unknowns.push_back(&y2a);
    unknowns.push_back(&x2b); unknowns.push_back(&y2b);
    unknowns.push_back(&x3a); unknowns.push_back(&y3a);
    unknowns.push_back(&x3b); unknowns.push_back(&y3b);
    system.declareUnknowns(unknowns);

    system.initSolution();
    int status = system.solve();
    system.applySolution();

    EXPECT_EQ(status, GCS::SolveStatus::Success);

    // Verify origin remains unchanged
    EXPECT_NEAR(x0a, 10.0, 1e-5);
    EXPECT_NEAR(y0a, 20.0, 1e-5);

    // Verify correct solved dimensions
    EXPECT_NEAR(x0b, 160.0, 1e-5);
    EXPECT_NEAR(y0b, 20.0, 1e-5);
    EXPECT_NEAR(x1b, 160.0, 1e-5);
    EXPECT_NEAR(y1b, 100.0, 1e-5);
    EXPECT_NEAR(x2b, 10.0, 1e-5);
    EXPECT_NEAR(y2b, 100.0, 1e-5);
}

TEST(PlaneGcsTests, TestFilletTangencySolving) {
    GCS::System system;

    // Line 0 (Horizontal, start point fixed at 0.0, 0.0)
    double x0a = 0.0;
    double y0a = 0.0;
    double x0b = 35.0;
    double y0b = 0.5; // perturbed

    // Line 1 (Vertical, end point fixed at 50.0, 50.0)
    double x1a = 49.5; // perturbed
    double y1a = 15.0;
    double x1b = 50.0;
    double y1b = 50.0;

    // Fillet circle center parameters (Radius = 10.0, expected center at 40.0, 10.0)
    double xc = 45.0; // perturbed
    double yc = 15.0; // perturbed
    double radius = 10.0;

    GCS::Point p0a(&x0a, &y0a);
    GCS::Point p0b(&x0b, &y0b);
    GCS::Point p1a(&x1a, &y1a);
    GCS::Point p1b(&x1b, &y1b);
    GCS::Point pc(&xc, &yc);

    GCS::Line l0; l0.p1 = p0a; l0.p2 = p0b;
    GCS::Line l1; l1.p1 = p1a; l1.p2 = p1b;
    GCS::Circle c; c.center = pc; c.rad = &radius;

    // Horizontal on Line 0 and Vertical on Line 1
    system.addConstraintHorizontal(p0a, p0b, 0, true);
    system.addConstraintVertical(p1a, p1b, 0, true);

    // Tangent constraints to circle
    system.addConstraintTangent(l0, c, 0, true);
    system.addConstraintTangent(l1, c, 0, true);

    // Align the coordinate points (tangent contacts)
    system.addConstraintVertical(p0b, pc, 0, true);
    system.addConstraintHorizontal(p1a, pc, 0, true);

    GCS::VEC_pD unknowns;
    unknowns.push_back(&x0b); unknowns.push_back(&y0b);
    unknowns.push_back(&x1a); unknowns.push_back(&y1a);
    unknowns.push_back(&xc); unknowns.push_back(&yc);
    system.declareUnknowns(unknowns);

    system.initSolution();
    int status = system.solve();
    system.applySolution();

    EXPECT_EQ(status, GCS::SolveStatus::Success);

    // Center should converge to (40.0, 10.0)
    EXPECT_NEAR(xc, 40.0, 1e-5);
    EXPECT_NEAR(yc, 10.0, 1e-5);

    // Trimmed endpoints should align perfectly
    EXPECT_NEAR(x0b, 40.0, 1e-5);
    EXPECT_NEAR(y0b, 0.0, 1e-5);
    EXPECT_NEAR(x1a, 50.0, 1e-5);
    EXPECT_NEAR(y1a, 10.0, 1e-5);
}

TEST(PlaneGcsTests, TestParallelPerpendicular) {
    GCS::System system;

    // Line 0: fixed from (0,0) to (10,0) - Horizontal
    double x0a = 0.0, y0a = 0.0;
    double x0b = 10.0, y0b = 0.0;

    // Line 1: free, starts near (0,5) to (5,12)
    double x1a = 0.0, y1a = 5.0;
    double x1b = 5.0, y1b = 12.0;

    // Line 2: free, starts near (5,5) to (12,5)
    double x2a = 5.0, y2a = 5.0;
    double x2b = 12.0, y2b = 5.0;

    GCS::Point p0a(&x0a, &y0a), p0b(&x0b, &y0b);
    GCS::Point p1a(&x1a, &y1a), p1b(&x1b, &y1b);
    GCS::Point p2a(&x2a, &y2a), p2b(&x2b, &y2b);

    GCS::Line l0; l0.p1 = p0a; l0.p2 = p0b;
    GCS::Line l1; l1.p1 = p1a; l1.p2 = p1b;
    GCS::Line l2; l2.p1 = p2a; l2.p2 = p2b;

    // Constrain l1 perpendicular to l0
    system.addConstraintPerpendicular(l0, l1, 0, true);

    // Constrain l2 parallel to l0
    system.addConstraintParallel(l0, l2, 0, true);

    // Pin start points of l1 and l2 to keep them in place
    // (meaning only the end points are free to move, but let's declare all free coordinates as unknowns)
    GCS::VEC_pD unknowns;
    unknowns.push_back(&x1b); unknowns.push_back(&y1b);
    unknowns.push_back(&x2b); unknowns.push_back(&y2b);
    system.declareUnknowns(unknowns);

    system.initSolution();
    int status = system.solve();
    system.applySolution();

    EXPECT_EQ(status, GCS::SolveStatus::Success);

    // l1 perpendicular to l0 (l0 is horizontal, so l1 must be vertical)
    EXPECT_NEAR(x1b, x1a, 1e-5);

    // l2 parallel to l0 (l0 is horizontal, so l2 must be horizontal)
    EXPECT_NEAR(y2b, y2a, 1e-5);
}

TEST(PlaneGcsTests, TestPointOnCircleAndEqualRadius) {
    GCS::System system;

    // Circle 1: center fixed at (0,0), radius parameter free
    double xc1 = 0.0, yc1 = 0.0;
    double rad1 = 2.0;

    // Circle 2: center fixed at (10,0), radius parameter free
    double xc2 = 10.0, yc2 = 0.0;
    double rad2 = 4.0;

    // Point P: free, initially at (0,3)
    double px = 0.0, py = 3.0;

    GCS::Point pc1(&xc1, &yc1);
    GCS::Point pc2(&xc2, &yc2);
    GCS::Point pP(&px, &py);

    GCS::Circle c1; c1.center = pc1; c1.rad = &rad1;
    GCS::Circle c2; c2.center = pc2; c2.rad = &rad2;

    // P must lie on Circle 1
    system.addConstraintPointOnCircle(pP, c1, 0, true);

    // Radius of Circle 1 must equal Circle 2
    system.addConstraintEqualRadius(c1, c2, 0, true);

    // We fix P's coordinates and solve for the radii (Circle 2's radius should change to 3.0, and Circle 1's radius should also change to 3.0)
    GCS::VEC_pD unknowns;
    unknowns.push_back(&rad1);
    unknowns.push_back(&rad2);
    system.declareUnknowns(unknowns);

    system.initSolution();
    int status = system.solve();
    system.applySolution();

    EXPECT_EQ(status, GCS::SolveStatus::Success);
    EXPECT_NEAR(rad1, 3.0, 1e-5);
    EXPECT_NEAR(rad2, 3.0, 1e-5);
}

TEST(PlaneGcsTests, TestAngleSolving) {
    GCS::System system;

    // Line 0: fixed along X axis (from 0,0 to 10,0)
    double x0a = 0.0, y0a = 0.0;
    double x0b = 10.0, y0b = 0.0;

    // Line 1: pivots around (0,0), ends at (5,5) - initially 45 degrees
    double x1a = 0.0, y1a = 0.0;
    double x1b = 5.0, y1b = 5.0;

    GCS::Point p0a(&x0a, &y0a), p0b(&x0b, &y0b);
    GCS::Point p1a(&x1a, &y1a), p1b(&x1b, &y1b);

    GCS::Line l0; l0.p1 = p0a; l0.p2 = p0b;
    GCS::Line l1; l1.p1 = p1a; l1.p2 = p1b;

    // We want the angle between l0 and l1 to be 60 degrees (pi / 3 radians)
    double target_angle = M_PI / 3.0; // 60 degrees in radians
    system.addConstraintL2LAngle(l0, l1, &target_angle, 0, true);

    // We also constrain the length of l1 to be 10.0
    double l1_len = 10.0;
    system.addConstraintP2PDistance(p1a, p1b, &l1_len, 0, true);

    GCS::VEC_pD unknowns;
    unknowns.push_back(&x1b);
    unknowns.push_back(&y1b);
    system.declareUnknowns(unknowns);

    system.initSolution();
    int status = system.solve();
    system.applySolution();

    EXPECT_EQ(status, GCS::SolveStatus::Success);

    // x1b should be 10 * cos(60) = 5.0
    // y1b should be 10 * sin(60) = 8.660254
    EXPECT_NEAR(x1b, 5.0, 1e-5);
    EXPECT_NEAR(y1b, 5.0 * std::sqrt(3.0), 1e-5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
