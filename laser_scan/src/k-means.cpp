#include "k-means.h"

using namespace std;

vector<Point> kmeans(vector<Point>& points, int K, bool forceCentroids, int maxIter) {
    srand(time(NULL));
   
    // Initialize the centers with kmeans++
    vector<Point> centers(K);
    centers[0] = points[rand()%points.size()];
    for (int i = 1; i < K; i++) {
        float maxDist = 0;
        for (Point p : points) {
            for (Point c : centers) {
                float dist = p.distance(c);
                if (dist < p.mind_dist) p.mind_dist = dist;
            }
        
            if (p.mind_dist > maxDist) {
                maxDist = p.mind_dist;
                centers[i] = p;
            }
        }        
    }

    // Start the algorithm
    bool stable = false;
    for (int iter = 0; !(stable) && iter < maxIter; iter++) {
        vector<Point> clustersSum(K);
        vector<int> clustersSize(K, 0);
        stable = true;
        
        // Find the current clusters
        for (int i = 0; i < points.size(); i++) {
            for (int j = 0; j < K; j++) {
                float dist = points[i].distance(centers[j]);
                if (dist < points[i].mind_dist) {
                    points[i].mind_dist = dist;
                    points[i].cluster_index = j;
                    stable = false;
                }
            }

            int index = points[i].cluster_index;
            clustersSum[index] += points[i];
            clustersSize[index]++;
        }

        // Update the centers according to Lloyd's algorithm
        //
        // Can either force the new centroids to be selected from the set of given 
        // points or allow them to be any point in the cartesian space
        if (forceCentroids) {
            for (int i = 0; i < K; i++) {
                Point newCenter = clustersSum[i]/clustersSize[i];

                for(int j = 0; j < points.size(); j++) {
                    if (points[j].cluster_index == i) {
                        float dist = newCenter.distance(points[j]);
                        if (dist < newCenter.mind_dist) {
                            newCenter.mind_dist = dist;
                            centers[i] = Point(points[j].x, points[j].y);
                        }
                    }
                }
            }
        }
        else {
            for (int i = 0; i < K; i++) {
                centers[i] = clustersSum[i]/clustersSize[i];
            }
        }
    }

    return centers;
}


float silhouette(const vector<Point>& points, int K) {
    float s = 0;
    for (Point p1 : points) {
        vector<float> clustersDist(K, 0);
        vector<int> clustersCount(K, 0);
        for (Point p2 : points) {
            clustersDist[p2.cluster_index] += p1.distance(p2);
            clustersCount[p2.cluster_index]++;
        }

        if (clustersCount[p1.cluster_index] > 1) {
            for (int i = 0; i < K; i++) {
                if (i == p1.cluster_index) clustersDist[i] /= (clustersCount[i]-1);
                else clustersDist[i] /= (clustersCount[i]);
            }

            float a = clustersDist[p1.cluster_index];
            clustersDist[p1.cluster_index] = FLT_MAX;

            int minIndex = distance(clustersDist.begin(),  min_element(clustersDist.begin(), clustersDist.end()));
            float b = clustersDist[minIndex];

            float pointS = (a > b) ? (b-a)/a : (b-a)/b;
            s += pointS;
        }
    }

    return s/points.size();
}