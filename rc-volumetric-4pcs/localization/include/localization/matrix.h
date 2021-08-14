#ifndef SRC_MATRIX_H
#define SRC_MATRIX_H

#include <localization/set.h>

class Matrix {
public:
    explicit Matrix(double leafSize);
    void fromSet(const Set &set);
    void getPoints(const Point &point, double distance, std::vector<Point*> points);

private:
    void getMatrixCoordinates(const Point &point, MatrixCoordinate &coordinate);
    void indexFromPoint(const Point &point, long &index);
    long width;
    long height;
    double leafSize;
    Point offset;
    std::vector<std::vector<Point*>> data;
    MatrixCoordinate coordinateHelper;
};

#endif //SRC_MATRIX_H
