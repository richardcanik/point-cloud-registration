#include <registration_core/randomer.h>
#include <registration_core/base.h>
#include <registration_core/math.h>
#include <registration_core/timer.h>
#include <registration_core/set.h>
#include <registration_core/octo_map.h>
#include <gtest/gtest.h>
#include <random>

#define EXPECT_IN_RANGE(val, min, max) EXPECT_GE(val, min); EXPECT_LE(val, max)

struct range{
    long min;
    long max;
};

const size_t numberOfSetPoints = 10000;
const size_t numberOfBasePoints = 4;
const double leafSize = 4;      // Convention: threshold <= leafsize/3
const range space = {-100, 100};

double getRandomNumber(const double &min, const double &max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(min, max);
    auto out = static_cast<double>(dist(gen));
    EXPECT_IN_RANGE(out, min, max);
    return out;
}

double getRandomNumberInSpace() {
    return getRandomNumber(static_cast<double>(space.min), static_cast<double>(space.max));
}

void generateRandomVectorOfPoints(std::vector<Point> &set) {
    double x, y;
    auto space_range = static_cast<double>(space.max - space.min);
    auto step = static_cast<double>(space_range / sqrt(numberOfSetPoints));

    for (x = space.min; x < space.max; x = x + step) {
        for (y = space.min; y < space.max; y = y + step) {
            set.emplace_back(x, y, 10 * sin(x / 10) * cos(y / 10));
        }
    }
}

void generateRandomVectorOfPoints(std::vector<Point> &set, const size_t &numOfPoints) {
    for (size_t i=0; i < numOfPoints; i++) {
        set.emplace_back(getRandomNumberInSpace(), getRandomNumberInSpace(), getRandomNumberInSpace());
    }
    EXPECT_EQ(set.size(), numOfPoints);
}

void getPoints(const std::vector<Point> &pointCloud, const std::vector<Condition*> &conditions, const double &distanceThreshold, std::vector<const Point*> &points) {
    bool flag;

    points.clear();
    for (auto &point : pointCloud) {
        flag = true;
        for (auto condition : conditions) {
            if (abs(getLineLength(point, *condition->point) - *condition->descriptor) > distanceThreshold) {
                flag = false;
                break;
            }
        }
        if (flag)
            points.push_back(&point);
    }
}

TEST(TimerTest, TestName1) {
    //TODO timer test
}

TEST(RandomerTest, TestRandomerClass) {
    const size_t min = 10, max = 5000;
    Randomer rand(min, max);

    EXPECT_IN_RANGE(rand(), min, max);
}

TEST(MathTest, TestLineLength) {
    Point p1, p2;
    double a, b;

    p1 = {getRandomNumberInSpace(), getRandomNumberInSpace(), getRandomNumberInSpace()};
    p2 = {getRandomNumberInSpace(), getRandomNumberInSpace(), getRandomNumberInSpace()};
    a = getLineLength(p1, p2);
    b = sqrt(pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2) + pow(p2.z() - p1.z(), 2));
    EXPECT_LE(abs(a - b), 0.001);

    getLineLength(p1, p2, a);
    EXPECT_LE(abs(a - b), 0.001);
}

TEST(MathTest, TestFactorial) {
    EXPECT_EQ(factorial(0), 1);
    EXPECT_EQ(factorial(1), 1);
    EXPECT_EQ(factorial(2), 2);
    EXPECT_EQ(factorial(3), 6);
    EXPECT_EQ(factorial(4), 24);
    EXPECT_EQ(factorial(5), 120);
    EXPECT_EQ(factorial(6), 720);
    EXPECT_EQ(factorial(7), 5040);
    EXPECT_EQ(factorial(8), 40320);
    EXPECT_EQ(factorial(9), 362880);
    EXPECT_EQ(factorial(10), 3628800);
}

TEST(MathTest, TestCombinationWithoutRepetition) {
    EXPECT_THROW(combinationWithoutRepetition(0), std::invalid_argument);
    EXPECT_THROW(combinationWithoutRepetition(1), std::invalid_argument);
    EXPECT_THROW(combinationWithoutRepetition(2, 3), std::invalid_argument);
    EXPECT_EQ(combinationWithoutRepetition(2), 1);
    EXPECT_EQ(combinationWithoutRepetition(3), 3);
    EXPECT_EQ(combinationWithoutRepetition(4), 6);
    EXPECT_EQ(combinationWithoutRepetition(5), 10);
    EXPECT_EQ(combinationWithoutRepetition(6), 15);
    EXPECT_EQ(combinationWithoutRepetition(7), 21);
    EXPECT_EQ(combinationWithoutRepetition(8), 28);
    EXPECT_EQ(combinationWithoutRepetition(9), 36);
    EXPECT_EQ(combinationWithoutRepetition(10), 45);
}

TEST(MathTest, TestAreNumbersSame) {
    EXPECT_TRUE(areNumbersSame(1, 2, 3, 4));
    EXPECT_FALSE(areNumbersSame(1, 2, 3, 3));
    EXPECT_FALSE(areNumbersSame(1, 2, 2, 3));
    EXPECT_FALSE(areNumbersSame(1, 1, 2, 3));
    EXPECT_FALSE(areNumbersSame(1, 1, 1, 2));
    EXPECT_FALSE(areNumbersSame(1, 2, 2, 2));
    EXPECT_FALSE(areNumbersSame(1, 1, 2, 2));
    EXPECT_FALSE(areNumbersSame(1, 1, 1, 1));
}

TEST(MathTest, TestSphereParametricEquation) {
    Point p;
    const double delta = 0.0001;

    sphereParametricEquation({0, 0, 0}, 1, 0, 0, p);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (1)), delta);
    sphereParametricEquation({0, 0, 0}, 1, 0, M_PI_2, p);
    EXPECT_LE(abs(p.x() - (1)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    sphereParametricEquation({0, 0, 0}, 1, 0, M_PI, p);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (-1)), delta);
    sphereParametricEquation({0, 0, 0}, 1, 0, M_PI+M_PI_2, p);
    EXPECT_LE(abs(p.x() - (-1)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    sphereParametricEquation({0, 0, 0}, 1, M_PI_2, M_PI_2, p);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (1)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    sphereParametricEquation({0, 0, 0}, 1, M_PI, M_PI_2, p);
    EXPECT_LE(abs(p.x() - (-1)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    sphereParametricEquation({0, 0, 0}, 1, M_PI+M_PI_2, M_PI_2, p);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (-1)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
}

TEST(MathTest, TestLineParametricEquation) {
    Point p;
    const double delta = 0.0001;

    lineParametricEquation({0, 0, 0}, {1, 0, 0}, 0, p);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    lineParametricEquation({0, 0, 0}, {1, 0, 0}, 0.5, p);
    EXPECT_LE(abs(p.x() - (0.5)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    lineParametricEquation({0, 0, 0}, {1, 0, 0}, 1, p);
    EXPECT_LE(abs(p.x() - (1)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
}

TEST(MathTest, TestCircleParametricEquation) {
    Point p;
    const double delta = 0.0001;

    circleParametricEquation({0, 0, 0}, 1, 0, {1, 0, 0}, {0, 1, 0}, p);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (1)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    circleParametricEquation({0, 0, 0}, 1, M_PI_2, {1, 0, 0}, {0, 1, 0}, p);
    EXPECT_LE(abs(p.x() - (1)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    circleParametricEquation({0, 0, 0}, 1, M_PI, {1, 0, 0}, {0, 1, 0}, p);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (-1)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
    circleParametricEquation({0, 0, 0}, 1, M_PI + M_PI_2, {1, 0, 0}, {0, 1, 0}, p);
    EXPECT_LE(abs(p.x() - (-1)), delta);
    EXPECT_LE(abs(p.y() - (0)), delta);
    EXPECT_LE(abs(p.z() - (0)), delta);
}

TEST(MathTest, TestIsTriangle) {
    EXPECT_TRUE(isTriangle(1, 1, 1));
    EXPECT_TRUE(isTriangle(1, 1, 1.9));
    EXPECT_FALSE(isTriangle(1, 1, 2));
    EXPECT_FALSE(isTriangle(0, 0, 0));
    EXPECT_FALSE(isTriangle(1, 0, 0));
    EXPECT_FALSE(isTriangle(1, 1, 0));
    EXPECT_FALSE(isTriangle(-1, 10, 8));
    EXPECT_FALSE(isTriangle(0, 10, 8));
}

TEST(MathTest, TestIsPointOnSphereSurface) {
    EXPECT_FALSE(isPointOnSphereSurface({0, 0, 0}, {0, 0, 0}, 2));
    EXPECT_FALSE(isPointOnSphereSurface({1, 2, 3}, {2, 2, 2}, 2));
    EXPECT_TRUE(isPointOnSphereSurface({2, 0, 0}, {0, 0, 0}, 2));
    EXPECT_TRUE(isPointOnSphereSurface({-1, 2, 3}, {1, 2, 3}, 2));
    EXPECT_TRUE(isPointOnSphereSurface({1, 0, 3}, {1, 2, 3}, 2));
    EXPECT_TRUE(isPointOnSphereSurface({1, 2, 1}, {1, 2, 3}, 2));
}

TEST(MathTest, TestTransformPoint) {
    Point p;
    Matrix4 t;
    const double delta = 0.0001;

    p = {1, 1, 1};
    t.setIdentity();
    t.block<3,1>(0,3) = Point(1, 2, 3);
    transformPoint(p, t);
    EXPECT_EQ(p.x(), 2);
    EXPECT_EQ(p.y(), 3);
    EXPECT_EQ(p.z(), 4);
    p = {1, 1, 1};
    t.setIdentity();
    t.block<3,3>(0,0) = Matrix3(Eigen::AngleAxisd(M_PI_2, Vector3::UnitZ()));
    transformPoint(p, t);
    EXPECT_LE(abs(p.x() - (-1)), delta);
    EXPECT_LE(abs(p.y() - (1)), delta);
    EXPECT_LE(abs(p.z() - (1)), delta);
    p = {1, 1, 1};
    t.setIdentity();
    t.block<3,3>(0,0) = Matrix3(Eigen::AngleAxisd(M_PI_2, Vector3::UnitX()));
    transformPoint(p, t);
    EXPECT_LE(abs(p.x() - (1)), delta);
    EXPECT_LE(abs(p.y() - (-1)), delta);
    EXPECT_LE(abs(p.z() - (1)), delta);
    p = {1, 1, 1};
    t.setIdentity();
    t.block<3,3>(0,0) = Matrix3(Eigen::AngleAxisd(M_PI_2, Vector3::UnitY()));
    transformPoint(p, t);
    EXPECT_LE(abs(p.x() - (1)), delta);
    EXPECT_LE(abs(p.y() - (1)), delta);
    EXPECT_LE(abs(p.z() - (-1)), delta);
    p = {1, 1, 1};
    t.setIdentity();
    t.block<3,3>(0,0) = Matrix3(Eigen::AngleAxisd(M_PI_2, Vector3::UnitZ()));
    t.block<3,1>(0,3) = Point(1, 2, 3);
    transformPoint(p, t);
    EXPECT_LE(abs(p.x() - (0)), delta);
    EXPECT_LE(abs(p.y() - (3)), delta);
    EXPECT_LE(abs(p.z() - (4)), delta);
}

TEST(MathTest, TestRotateVector) {
    Vector3 v;
    const double delta = 0.0001;

    rotateVector({1, 0, 0}, {0, 1, 0}, M_PI_2, v);
    EXPECT_LE(abs(v.x() - (0)), delta);
    EXPECT_LE(abs(v.y() - (0)), delta);
    EXPECT_LE(abs(v.z() - (-1)), delta);
    rotateVector({0, 1, 0}, {1, 0, 0}, M_PI_2, v);
    EXPECT_LE(abs(v.x() - (0)), delta);
    EXPECT_LE(abs(v.y() - (0)), delta);
    EXPECT_LE(abs(v.z() - (1)), delta);
    rotateVector({0, 0, 1}, {1, 0, 0}, M_PI_2, v);
    EXPECT_LE(abs(v.x() - (0)), delta);
    EXPECT_LE(abs(v.y() - (-1)), delta);
    EXPECT_LE(abs(v.z() - (0)), delta);
}

TEST(MathTest, TestTwoSpheresIntersection) {
    Point c1, c2, center;
    double radius;
    Vector3 v1, v2;
    INTERSECTION_STATUS status;
    const double delta = 0.0001;

    twoSpheresIntersection({1, 1, 0}, 1, {-2, 1, 0}, 1, center, radius, v1, v2, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::NONE);
    twoSpheresIntersection({1, 1, 1}, 1, {1, 1, 1}, 2, center, radius, v1, v2, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::NONE);
    twoSpheresIntersection({1, 1, 0}, 1, {-1, 1, 0}, 1, center, radius, v1, v2, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::TOUCH_POINT);
    EXPECT_LE(abs(center.x() - (0)), delta);
    EXPECT_LE(abs(center.y() - (1)), delta);
    EXPECT_LE(abs(center.z() - (0)), delta);
    twoSpheresIntersection(c1 = {1, 1, 0}, 2, c2 = {-1, 1, 0}, 2, center, radius, v1, v2, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::MORE);
    EXPECT_LE(abs(center.x() - (0)), delta);
    EXPECT_LE(abs(center.y() - (1)), delta);
    EXPECT_LE(abs(center.z() - (0)), delta);
    EXPECT_LE(abs(radius - (sqrt(3))), delta);
    EXPECT_EQ(c1.dot(v1), 0);
    EXPECT_EQ(c2.dot(v1), 0);
    EXPECT_EQ(c1.cross(c2).dot(v2), 0);
    twoSpheresIntersection({1, 1, 1}, 2, {1, 1, 1}, 2, center, radius, v1, v2, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::SAME);
}

TEST(MathTest, TestCircleSphereIntersection) {
    std::vector<Point> points;
    INTERSECTION_STATUS status;
    const double delta = 0.0001;

    circleSphereIntersection({1, 1, 0}, 1, {1, 0, 0}, {0, 1, 0}, {-2, 1, 2}, 1, points, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::NONE);
    EXPECT_EQ(points.size(), 0);
    circleSphereIntersection({1, 1, 0}, 1, {1, 0, 0}, {0, 1, 0}, {-2, 1, 1}, 1, points, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::NONE);
    EXPECT_EQ(points.size(), 0);
    circleSphereIntersection({1, 1, 0}, 1, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, 1, points, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::SAME);
    EXPECT_EQ(points.size(), 0);
    circleSphereIntersection({1, 1, 0}, 1, {1, 0, 0}, {0, 1, 0}, {-1, 1, 0}, 1, points, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::TOUCH_POINT);
    EXPECT_EQ(points.size(), 1);
    EXPECT_LE(abs(points[0].x() - (0)), delta);
    EXPECT_LE(abs(points[0].y() - (1)), delta);
    EXPECT_LE(abs(points[0].z() - (0)), delta);
    circleSphereIntersection({1, 1, 1}, 1, {1, 0, 0}, {0, 1, 0}, {0, 1, 0}, 1, points, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::TOUCH_POINT);
    EXPECT_EQ(points.size(), 1);
    EXPECT_LE(abs(points[0].x() - (0)), delta);
    EXPECT_LE(abs(points[0].y() - (1)), delta);
    EXPECT_LE(abs(points[0].z() - (1)), delta);
    circleSphereIntersection({1, 1, 0}, sqrt(5), {1, 0, 0}, {0, 1, 0}, {-1, 1, 0}, 1, points, status);
    EXPECT_EQ(status, INTERSECTION_STATUS::MORE);
    EXPECT_EQ(points.size(), 2);
    EXPECT_LE(abs(points[0].x() - (-1)), delta);
    EXPECT_LE(abs(points[0].y() - (0)), delta);
    EXPECT_LE(abs(points[0].z() - (0)), delta);
    EXPECT_LE(abs(points[1].x() - (-1)), delta);
    EXPECT_LE(abs(points[1].y() - (2)), delta);
    EXPECT_LE(abs(points[1].z() - (0)), delta);
}

TEST(BaseTest, TestPoints) {
    Base base;
    std::vector<Point> points;

    generateRandomVectorOfPoints(points, numberOfBasePoints);
    base.setBase(points[0], points[1], points[2], points[3]);
    EXPECT_EQ(base.getPoints().size(), numberOfBasePoints);
    for (size_t i = 0; i < numberOfBasePoints; i++) {
        EXPECT_EQ(points[i].x(), base.getPoints()[i].x());
        EXPECT_EQ(points[i].y(), base.getPoints()[i].y());
        EXPECT_EQ(points[i].z(), base.getPoints()[i].z());
    }
}

TEST(BaseTest, TestDescriptors) {
    Base base;
    std::vector<Point> points;
    std::vector<double> expectedDescriptors;

    generateRandomVectorOfPoints(points, numberOfBasePoints);
    base.setBase(points[0], points[1], points[2], points[3]);
    for (size_t i = 0; i < numberOfBasePoints; i++) {
        for (size_t j = i + 1; j < numberOfBasePoints; j++) {
            expectedDescriptors.push_back(getLineLength(points[i], points[j]));
        }
    }
    EXPECT_EQ(expectedDescriptors.size(), base.getDescriptors().size());
    EXPECT_EQ(expectedDescriptors.size(), combinationWithoutRepetition(numberOfBasePoints));
    for (size_t i = 0; i < expectedDescriptors.size(); i++) {
        EXPECT_EQ(base.getDescriptors()[i], expectedDescriptors[i]);
    }
}

TEST(BaseTest, TestFrame) {
    Base base;
    std::vector<Point> points;

    generateRandomVectorOfPoints(points, numberOfBasePoints);
    base.setBase(points[0], points[1], points[2], points[3]);
    EXPECT_EQ(base.getFrame()(0, 3), points[0].x());
    EXPECT_EQ(base.getFrame()(1, 3), points[0].y());
    EXPECT_EQ(base.getFrame()(2, 3), points[0].z());
}

TEST(SetTest, TestSetClass) {
    std::vector<Point> pointSet;
    Set set;

    generateRandomVectorOfPoints(pointSet);
    set.setSet(pointSet);
    EXPECT_EQ(set.getSet().size(), numberOfSetPoints);
    EXPECT_LE(set.getWidth(), space.max - space.min);
    EXPECT_LE(set.getHeight(), space.max - space.min);
    EXPECT_LE(set.getDepth(), space.max - space.min);
    EXPECT_LE(set.getMinBoundingBox().x(), set.getMaxBoundingBox().x());
    EXPECT_LE(set.getMinBoundingBox().y(), set.getMaxBoundingBox().y());
    EXPECT_LE(set.getMinBoundingBox().z(), set.getMaxBoundingBox().z());
    EXPECT_IN_RANGE(set.getMinBoundingBox().x(), space.min, space.max);
    EXPECT_IN_RANGE(set.getMaxBoundingBox().x(), space.min, space.max);
    EXPECT_IN_RANGE(set.getMinBoundingBox().y(), space.min, space.max);
    EXPECT_IN_RANGE(set.getMaxBoundingBox().y(), space.min, space.max);
    EXPECT_IN_RANGE(set.getMinBoundingBox().z(), space.min, space.max);
    EXPECT_IN_RANGE(set.getMaxBoundingBox().z(), space.min, space.max);
}

// TODO test big point cloud
// TODO test big space
// TODO test high density point cloud
TEST(OctoMap, TestPointsFromSphere) {
    std::vector<const Point*> pointsFromOctoMap, pointsFromValidateFunction;
    std::vector<Condition*> conditions;
    std::vector<Point> pointSet;
    OctoMap octoMap(leafSize);
    Condition condition;
    double distance, threshold;
    Point point;
    Set set;
    Randomer random{0, numberOfSetPoints};

    conditions.clear();
    conditions.push_back(&condition);

    generateRandomVectorOfPoints(pointSet);
    set.setSet(pointSet);
    octoMap.fromSet(set);

    distance = getRandomNumber(30, 0.75 * std::max(set.getWidth(), set.getHeight()));     // TODO get max from all (width, height, depth)
    point = pointSet[random()];
    threshold = getRandomNumber(1, leafSize / 3);
    condition.point = &point;
    condition.descriptor = &distance;

    octoMap.getPoints(conditions, threshold, pointsFromOctoMap);                // Get points using octo map
    getPoints(set.getSet(), conditions, threshold, pointsFromValidateFunction); // Get points using function for validation

    // Test if size of vectors is equal
    EXPECT_EQ(pointsFromOctoMap.size(), pointsFromValidateFunction.size());
    std::cout << "size from octo map: " << pointsFromOctoMap.size() << std::endl;
    std::cout << "size from validate: " << pointsFromValidateFunction.size() << std::endl;

    for (auto &p : pointsFromValidateFunction) {
        EXPECT_LE(abs(getLineLength(point, *p) - distance), threshold);
    }

    for (auto &p : pointsFromOctoMap) {
        EXPECT_LE(abs(getLineLength(point, *p) - distance), threshold);

        auto count1 = std::count(pointsFromOctoMap.begin(), pointsFromOctoMap.end(), p);
        // Test if the point p is only once in the pointsFromOctoMap vector
        EXPECT_EQ(count1, 1);

        auto count2 = std::count(pointsFromValidateFunction.begin(), pointsFromValidateFunction.end(), p);
        // Test if the point p is only once in the pointsFromValidateFunction vector
        EXPECT_EQ(count2, 1);

        auto it = std::find(set.getSet().begin(), set.getSet().end(), *p);
        // Test if the point p is in the set
        EXPECT_NE(it, set.getSet().end());
        // Test if the pointer of p is the same as in the set
        EXPECT_EQ(&*it, p);
    }
}

// TODO test big point cloud
// TODO test big space
// TODO test high density point cloud
TEST(OctoMap, TestPointsFromTwoSpheres) {
    std::vector<const Point*> pointsFromOctoMap, pointsFromValidateFunction;
    std::vector<Condition*> conditions;
    std::vector<Point> pointSet;
    OctoMap octoMap(leafSize);
    Condition condition1, condition2;
    double distance1, distance2, threshold;
    Point point1, point2;
    Set set;
    Randomer random{0, numberOfSetPoints};

    conditions.clear();
    conditions.push_back(&condition1);
    conditions.push_back(&condition2);

    generateRandomVectorOfPoints(pointSet);
    set.setSet(pointSet);
    octoMap.fromSet(set);
    do {
        distance1 = getRandomNumber(30, 0.75 * std::max(set.getWidth(), set.getHeight()));    // TODO get max from all (width, height, depth)
        distance2 = getRandomNumber(30, 0.75 * std::max(set.getWidth(), set.getHeight()));    // TODO get max from all (width, height, depth)
        point1 = pointSet[random()];
        point2 = pointSet[random()];
    } while (! isTriangle(distance1, distance2, getLineLength(point1, point2)));

    threshold = getRandomNumber(1, leafSize / 3);
    condition1.point = &point1;
    condition1.descriptor = &distance1;
    condition2.point = &point2;
    condition2.descriptor = &distance2;

    octoMap.getPoints(conditions, threshold, pointsFromOctoMap);               // Get points using octo map
    getPoints(set.getSet(), conditions, threshold,pointsFromValidateFunction); // Get points using function for validation

    // Test if size of vectors is equal
    EXPECT_EQ(pointsFromOctoMap.size(), pointsFromValidateFunction.size());
    std::cout << "size from octo map: " << pointsFromOctoMap.size() << std::endl;
    std::cout << "size from validate: " << pointsFromValidateFunction.size() << std::endl;

    for (auto &p : pointsFromValidateFunction) {
        EXPECT_LE(abs(getLineLength(point1, *p) - distance1), threshold);
        EXPECT_LE(abs(getLineLength(point2, *p) - distance2), threshold);
    }

    for (auto &p : pointsFromOctoMap) {
        EXPECT_LE(abs(getLineLength(point1, *p) - distance1), threshold);
        EXPECT_LE(abs(getLineLength(point2, *p) - distance2), threshold);

        auto count1 = std::count(pointsFromOctoMap.begin(), pointsFromOctoMap.end(), p);
        // Test if the point p is only once in the pointsFromOctoMap vector
        EXPECT_EQ(count1, 1);

        auto count2 = std::count(pointsFromValidateFunction.begin(), pointsFromValidateFunction.end(), p);
        // Test if the point p is only once in the pointsFromValidateFunction vector
        EXPECT_EQ(count2, 1);

        auto it = std::find(set.getSet().begin(), set.getSet().end(), *p);
        // Test if the point p is in the set
        EXPECT_NE(it, set.getSet().end());
        // Test if the pointer of p is the same as in the set
        EXPECT_EQ(&*it, p);
    }
}

// TODO test big point cloud
// TODO test big space
// TODO test high density point cloud
TEST(OctoMap, TestPointsFromThreeSpheres) {
    std::vector<const Point*> pointsFromOctoMap, pointsFromValidateFunction;
    std::vector<Condition*> conditions;
    std::vector<Point> pointSet;
    OctoMap octoMap(leafSize);
    Condition condition1, condition2, condition3;
    double distance1, distance2, distance3, threshold;
    Point point1, point2, point3;
    Set set;
    Randomer random{0, numberOfSetPoints};

    conditions.clear();
    conditions.push_back(&condition1);
    conditions.push_back(&condition2);
    conditions.push_back(&condition3);

    generateRandomVectorOfPoints(pointSet);
    set.setSet(pointSet);
    octoMap.fromSet(set);

    do {
        distance1 = getRandomNumber(30, 60);    // TODO get max from all (width, height, depth)
        distance2 = getRandomNumber(30, 60);    // TODO get max from all (width, height, depth)
        distance3 = getRandomNumber(30, 60);    // TODO get max from all (width, height, depth)
        point1 = pointSet[random()];
        point2 = pointSet[random()];
        point3 = pointSet[random()];
    } while (! (isTriangle(distance1, distance2, getLineLength(point1, point2)) &&
               isTriangle(distance2, distance3, getLineLength(point2, point3)) &&
               isTriangle(distance3, distance1, getLineLength(point3, point1)) &&
               isTriangle(distance1, distance2, distance3) &&
               getLineLength(point1, point2) > 60 &&
               getLineLength(point2, point3) > 60 &&
               getLineLength(point3, point1) > 60) );
    threshold = getRandomNumber(0.5, leafSize / 3);

    condition1.point = &point1;
    condition1.descriptor = &distance1;
    condition2.point = &point2;
    condition2.descriptor = &distance2;
    condition3.point = &point3;
    condition3.descriptor = &distance3;

    // TODO check same conditions
    octoMap.getPoints(conditions, threshold, pointsFromOctoMap);               // Get points using octo map
    getPoints(set.getSet(), conditions, threshold,pointsFromValidateFunction); // Get points using function for validation

    // Test if size of vectors is equal
    EXPECT_EQ(pointsFromOctoMap.size(), pointsFromValidateFunction.size());
    std::cout << "size from octo map: " << pointsFromOctoMap.size() << std::endl;
    std::cout << "size from validate: " << pointsFromValidateFunction.size() << std::endl;

    for (auto &p : pointsFromValidateFunction) {
        EXPECT_LE(abs(getLineLength(point1, *p) - distance1), threshold);
        EXPECT_LE(abs(getLineLength(point2, *p) - distance2), threshold);
        EXPECT_LE(abs(getLineLength(point3, *p) - distance3), threshold);
    }

    for (auto &p : pointsFromOctoMap) {
        EXPECT_LE(abs(getLineLength(point1, *p) - distance1), threshold);
        EXPECT_LE(abs(getLineLength(point2, *p) - distance2), threshold);
        EXPECT_LE(abs(getLineLength(point3, *p) - distance3), threshold);

        auto count1 = std::count(pointsFromOctoMap.begin(), pointsFromOctoMap.end(), p);
        // Test if the point p is only once in the pointsFromOctoMap vector
        EXPECT_EQ(count1, 1);

        auto count2 = std::count(pointsFromValidateFunction.begin(), pointsFromValidateFunction.end(), p);
        // Test if the point p is only once in the pointsFromValidateFunction vector
        EXPECT_EQ(count2, 1);

        auto it = std::find(set.getSet().begin(), set.getSet().end(), *p);
        // Test if the point p is in the set
        EXPECT_NE(it, set.getSet().end());
        // Test if the pointer of p is the same as in the set
        EXPECT_EQ(&*it, p);
    }

//    VoxelCoordinate voxel;
//    for (auto &w : pointsFromValidateFunction) {
//        octoMap.point2Coordinate(*w, voxel);
//        std::cout << "validate: " << voxel.x << " " << voxel.y << " " << voxel.z << std::endl;
//    }

//    std::cout << "distance1: " << distance1 << std::endl;
//    std::cout << "distance2: " << distance2 << std::endl;
//    std::cout << "distance3: " << distance3 << std::endl;
//    size_t k = 1;
//    for (auto &w : pointSet) {
//        std::cout << "#" << k++ << "::" << w.x() << "::" << w.y() << "::" << w.z() << "::" << w.z() << "::1::A::1::0::0::0::0;" << std::endl;
//    }
//    std::cout << "#" << k++ << "::" << point1.x() << "::" << point1.y() << "::" << point1.z() << "::" << point1.z() << "::5::A::1::0::0::0::0;" << std::endl;
//    std::cout << "#" << k++ << "::" << point2.x() << "::" << point2.y() << "::" << point2.z() << "::" << point2.z() << "::5::A::1::0::0::0::0;" << std::endl;
//    std::cout << "#" << k++ << "::" << point3.x() << "::" << point3.y() << "::" << point3.z() << "::" << point3.z() << "::5::A::1::0::0::0::0;" << std::endl;
//    for (auto &w : pointsFromValidateFunction) {
//        std::cout << "#" << k++ << "::" << w->x() << "::" << w->y() << "::" << w->z() << "::" << w->z() << "::1::A::1::0::0::0::0;" << std::endl;
//    }
//    for (auto &w : pointsFromOctoMap) {
//        std::cout << "#" << k++ << "::" << w->x() << "::" << w->y() << "::" << w->z() << "::" << w->z() << "::10::B::1::0::0::0::0;" << std::endl;
//    }
}

TEST(RegistrationTest, TestName1) {
    //TODO registration test
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
