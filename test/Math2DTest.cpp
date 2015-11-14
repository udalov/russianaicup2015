#include <cmath>
#include "gtest/gtest.h"
#include "math2d.h"

using namespace std;

class Math2DTest : public ::testing::Test {
};

TEST_F(Math2DTest, VecLength) {
    EXPECT_EQ(5.0, Vec(3.0, 4.0).length());
}

TEST_F(Math2DTest, VecArithmetic) {
    auto a = Vec { 1.0, 2.0 };
    auto b = Vec { -5.0, 3.0 };
    EXPECT_EQ((Vec { -4.0, 5.0 }), a + b);
    EXPECT_EQ((Vec { 6.0, -1.0 }), a - b);
    EXPECT_EQ((Vec { 2.5, 5.0 }), a * 2.5);
    EXPECT_EQ((Vec { 5.0, -3.0 }), -b);
    EXPECT_DOUBLE_EQ(1.0, a * b);
    EXPECT_DOUBLE_EQ(13.0, a ^ b);
}

TEST_F(Math2DTest, VecAngle) {
    EXPECT_DOUBLE_EQ(M_PI / 4, (Vec { 1.0, 1.0 }).angle());
    EXPECT_DOUBLE_EQ(3 * M_PI / 4, (Vec { -1.0, 1.0 }).angle());
    EXPECT_DOUBLE_EQ(-3 * M_PI / 4, (Vec { -1.0, -1.0 }).angle());
    EXPECT_DOUBLE_EQ(-M_PI / 4, (Vec { 1.0, -1.0 }).angle());
}

TEST_F(Math2DTest, VecNormalize) {
    EXPECT_EQ((Vec { 0.6, 0.8 }), (Vec { 3.0, 4.0 }).normalize());
    EXPECT_EQ((Vec { 0.6, -0.8 }), (Vec { 3.0, -4.0 }).normalize());
}

TEST_F(Math2DTest, VecProject) {
    auto r = Vec { 7.0, 0.0 };
    auto u = Vec { 0.0, 1.0 };
    auto l = Vec { -1.0, 0.0 };
    auto b = Vec { 0.0, -1.0 };
    auto ur = Vec { 1.0, 1.0 };

    EXPECT_EQ(r.normalize(), ur.project(r));
    EXPECT_EQ(r.normalize(), ur.project(l));
    EXPECT_EQ(u, ur.project(u));
    EXPECT_EQ(u, ur.project(b));
}

TEST_F(Math2DTest, VecAngleTo) {
    auto r = Vec { 1.0, 0.0 };
    auto u = Vec { 0.0, 1.0 };
    auto l = Vec { -1.0, 0.0 };
    auto b = Vec { 0.0, -1.0 };
    auto ur = Vec { 1.0, 1.0 };
    auto ul = Vec { -1.0, 1.0 };
    auto bl = Vec { -1.0, -1.0 };
    auto br = Vec { 1.0, -1.0 };

    EXPECT_DOUBLE_EQ(M_PI / 4, ur.angleTo(r));
    EXPECT_DOUBLE_EQ(-M_PI / 4, ur.angleTo(u));
    EXPECT_DOUBLE_EQ(-3 * M_PI / 4, ur.angleTo(l));
    EXPECT_DOUBLE_EQ(3 * M_PI / 4, ur.angleTo(b));

    EXPECT_DOUBLE_EQ(M_PI / 4, ul.angleTo(u));
    EXPECT_DOUBLE_EQ(-M_PI / 4, ul.angleTo(l));
    EXPECT_DOUBLE_EQ(-3 * M_PI / 4, ul.angleTo(b));
    EXPECT_DOUBLE_EQ(3 * M_PI / 4, ul.angleTo(r));

    EXPECT_DOUBLE_EQ(M_PI / 4, bl.angleTo(l));
    EXPECT_DOUBLE_EQ(-M_PI / 4, bl.angleTo(b));
    EXPECT_DOUBLE_EQ(-3 * M_PI / 4, bl.angleTo(r));
    EXPECT_DOUBLE_EQ(3 * M_PI / 4, bl.angleTo(u));

    EXPECT_DOUBLE_EQ(M_PI / 4, br.angleTo(b));
    EXPECT_DOUBLE_EQ(-M_PI / 4, br.angleTo(r));
    EXPECT_DOUBLE_EQ(-3 * M_PI / 4, br.angleTo(u));
    EXPECT_DOUBLE_EQ(3 * M_PI / 4, br.angleTo(l));
}

TEST_F(Math2DTest, VecRotate) {
    auto ur = Vec { 1.0, 1.0 };
    auto ul = Vec { -1.0, 1.0 };
    auto bl = Vec { -1.0, -1.0 };
    auto br = Vec { 1.0, -1.0 };

    EXPECT_EQ(ul, ur.rotate(M_PI / 2));
    EXPECT_EQ(bl, ur.rotate(M_PI));
    EXPECT_EQ(br, ur.rotate(-M_PI / 2));
    EXPECT_EQ(br, ur.rotate(3 * M_PI / 2));

    EXPECT_EQ(bl, ul.rotate(M_PI / 2));
    EXPECT_EQ(br, ul.rotate(M_PI));
    EXPECT_EQ(ur, ul.rotate(-M_PI / 2));
    EXPECT_EQ(ur, ul.rotate(3 * M_PI / 2));

    EXPECT_EQ(ur, ur.rotate(2 * M_PI));
    EXPECT_EQ(ul, ul.rotate(2 * M_PI));
}

TEST_F(Math2DTest, LineContains) {
    auto l = Line { Point { 4., 0. }, Point { 0., 2. } };
    EXPECT_TRUE(l.contains(Point { 4., 0. }));
    EXPECT_TRUE(l.contains(Point { 2., 1. }));
    EXPECT_TRUE(l.contains(Point { 0., 2. }));
    EXPECT_FALSE(l.contains(Point { 0., 0. }));
    EXPECT_FALSE(l.contains(Point { 4., 1. }));
}

TEST_F(Math2DTest, PointArithmetic) {
    auto p = Point { 2.0, -3.0 };
    auto v = Vec { -2.0, 4.0 };
    auto q = p + v;
    EXPECT_EQ((Point { 0.0, 1.0 }), q);
    EXPECT_DOUBLE_EQ(v.length(), p.distanceTo(q));
    EXPECT_EQ(q, p - (-v));
}

TEST_F(Math2DTest, SinCos) {
    // TODO: improve
    double tolerance = 0.0000001;
    for (int i = 0; i <= 360; i++) {
        auto phi = M_PI * i / 180;
        EXPECT_NEAR(sin(phi), mySin(phi), tolerance);
    }
    for (int i = 0; i <= 360; i++) {
        auto phi = M_PI * i / 180;
        EXPECT_NEAR(cos(phi), myCos(phi), tolerance);
    }
}
