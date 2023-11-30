#ifndef K_MEANS
#define K_MEANS

#include <algorithm>
#include <cfloat>
#include <vector>
#include <random>
#include <time.h>


struct Point {
    float x, y;
    float mind_dist = FLT_MAX;
    int cluster_index = -1;

    explicit constexpr Point(float _x = 0, float _y = 0)
        : x(_x), y(_y) {}

    float distance(const Point& p) {
        return powf(p.x-x, 2) + powf(p.y-y, 2);
    }
};

inline Point operator / (const Point& p, float d) {
    return Point(p.x/d, p.y/d);
}

inline Point operator + (const Point& p1, const Point& p2) {
    return Point(p1.x + p2.x, p1.y + p2.y);
}

inline Point& operator += (Point& p1, const Point& p2) {
    p1 = p1 + p2;
    return p1;
}


// Computes k-means clustering fot the given point set.
// Loops untill convergence or untill maxIter is reached, forceCentroid param specifies if we want the
// centroids to be from within the point set or if any vector in the cartesian space can be a candidate center
//
// Modifies the input points set specifying for each one its cluster index and the distance from its center
std::vector<Point> kmeans(std::vector<Point>& detectedPoints, int K, bool forceCentroids, int maxIter = 100);

// Given a labeled set of point computes its silhouette coefficient
float silhouette(const std::vector<Point>& points, int K);


#endif