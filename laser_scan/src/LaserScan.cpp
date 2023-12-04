#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <cmath>
#include <set>
#include "k-means.h"

using namespace std;
using namespace ros;

const string TOPIC = "/scan";
const int MAX_PEOPLE = 10;

void detectPositions(const sensor_msgs::LaserScan::ConstPtr& msg) {
    vector<Point> detectedPoints;

    // Convert the detected points from polar to cartesian 2D coordinates
    for (int i = 0; i < msg->ranges.size(); i++) {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i*msg->angle_increment;
        
        if (range >= msg->range_min && range <= msg->range_max) {
            Point p = Point(range*cos(angle), range*sin(angle));
            detectedPoints.push_back(p);
        }
    }

    // Use k-means clustering and the silhoutte coefficient to guess the number of people
    // General idea: if we suppose to have N people then we will have 2*N feet.
    //      Mapping the detected points in space we then expect to compute two "good" clusterings
    //      for two different values of K:
    //      1) Fixing K=2*N we expect to assign a center to each points cloud of each foot
    //      2) Fixing K=N we expect to assign a center in between each pair of feet
    //
    // Therefore we select the number of people N as follow:
    //      - At each iter increase N by 1 and compute two clusterings fixing K=N and K=2*N
    //      - Compute and sum the silhouette coefficients for the two clusterings
    //      - Select N from the iteration with the highest sum 
    //
    // The silhouette coefficient is not defined for K=1, and therefore we use K=2 to evaluate
    // both clusterings. In fact, if we assume there is only 1 person and therefore 2 feet, then
    // we expect both the clustering with "fixed" centroids and the one with "free" centroids
    // to be close and to assign a center to each foot getting a good silhouette score for both.
    // Instead, if we have 2 or more people (4+ feet), the clustering with 2 fixed centroids will be
    // less good than the free one resulting in a lower total silhouette score.
    //
    int bestK = -1;
    float bestSilhouette = -2;
    vector<Point> bestFeetClusters;
    vector<Point> bestPeopleClusters;
    for (int K = 1; K <= MAX_PEOPLE && 2*K <= detectedPoints.size(); K++) {
        // Cluster the points assigning a cluster to each person and evaluate the corresponding silhouette
        vector<Point> peopleClusters = detectedPoints;
        float s = (K == 1) ? kmeansSilhouette(peopleClusters, 2, false) : kmeansSilhouette(peopleClusters, K, false);

        // Cluster the points assigning a cluster to each foot and evaluate the corresponding silhouette
        vector<Point> feetClusters = detectedPoints;
        s += kmeansSilhouette(feetClusters, K*2, true);

        if (bestSilhouette < s) {
            bestPeopleClusters = (K == 1) ? detectedPoints : peopleClusters; // Despite being K=1, peopleClusters has been clustered with K=2
            bestFeetClusters = feetClusters;
            bestSilhouette = s;
            bestK = K;
        }
    }

    ROS_INFO("Detected number of people: %d", bestK);
    if (bestK == -1) return; // No one detected

    // Computes the center of each foot and couple their indexes according to the corresponding person
    vector<Point> feetPos(2*bestK);
    vector<int> feetPointsCount(2*bestK, 0);
    vector<set<int>> peopleFeetIndexes (bestK);
    for (int i = 0; i < bestFeetClusters.size(); i++) {
        int footIndex = bestFeetClusters[i].cluster_index;
        feetPos[footIndex] += bestFeetClusters[i];
        feetPointsCount[footIndex]++;

        // Math the current foot index to the corresponding person index
        int personIndex = bestPeopleClusters[i].cluster_index;
        peopleFeetIndexes[personIndex].insert(footIndex);
    }
    for(int i = 0; i < 2*bestK; i++) {
        feetPos[i] = feetPos[i]/feetPointsCount[i];
    }

    // Computes the points in between each pairs of feet
    vector<Point> peoplePos(bestK);
    for (int i = 0; i < bestK; i++) {
        for (int feetIndex : peopleFeetIndexes[i]) {
            peoplePos[i] += feetPos[feetIndex];
        }

        // Middle point between the two feet
        peoplePos[i] = peoplePos[i]/2;
    }

    // Prints the results
    for (int i = 0; i < bestK; i++) {
        ROS_INFO("PERSON NUMBER %d ---> POS: (%f, %f)", i+1, peoplePos[i].x, peoplePos[i].y);
        for (int feetIndex : peopleFeetIndexes[i]) {
            ROS_INFO("\t-FOOT POS: (%f ,%f)", feetPos[feetIndex].x, feetPos[feetIndex].y); 
        }
    }
}


int main(int argc, char** argv) {
    init(argc, argv, "LaserScan");

    NodeHandle n;
    Subscriber sub = n.subscribe(TOPIC, 1000, detectPositions);
    ROS_INFO("SUBSCRIBED TO %s - WAITING", TOPIC.c_str());
   
    spin();

    return 0;
}