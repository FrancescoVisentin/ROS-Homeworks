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
Publisher pub;                  //TODO REMOVE!!!!

// Function to visualize results on Rviz for debugging                               //TODO REMOVE!!!!
void printPoints(const vector<Point>& feetPos, const vector<Point>& peoplePos) {
    // Green points for feet positions
    visualization_msgs::Marker feetPoints;
    feetPoints.header.frame_id = "base_scan";
    feetPoints.action = visualization_msgs::Marker::ADD;
    feetPoints.header.stamp = ros::Time::now();
    feetPoints.pose.orientation.w = 1.0;
    feetPoints.id = 0;
    feetPoints.type = visualization_msgs::Marker::POINTS;
    feetPoints.scale.x = 0.05;
    feetPoints.scale.y = 0.05;
    feetPoints.color.g = 1.0;
    feetPoints.color.a = 1.0;

    // Blue points for people positions
    visualization_msgs::Marker peoplePoints = feetPoints;
    peoplePoints.id = 1;
    peoplePoints.color.g = 0.0;
    peoplePoints.color.b = 1.0;

    ROS_INFO("FEET POSITIONS:");
    for (int i = 0; i < feetPos.size(); i++) {
        ROS_INFO("Foot %d ---> (%f, %f)", i, feetPos[i].x, feetPos[i].y);
        
        geometry_msgs::Point p;
        p.x = feetPos[i].x;
        p.y = feetPos[i].y;
        p.z = 0;
        feetPoints.points.push_back(p);
    }

    ROS_INFO("CORRESPONDING PEOPLE POSITIONS:");
    for (int i = 0; i < peoplePos.size(); i++) {
        ROS_INFO("Person %d ---> (%f, %f)", i, peoplePos[i].x, peoplePos[i].y);

        geometry_msgs::Point p;
        p.x = peoplePos[i].x;
        p.y = peoplePos[i].y;
        p.z = 0;
        peoplePoints.points.push_back(p);
    }

    pub.publish(feetPoints);
    pub.publish(peoplePoints);
}

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

        ROS_INFO("Sum of silhouettes for %d person and %d feet ---> %f", K, 2*K, s);
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

    printPoints(feetPos, peoplePos);
}


int main(int argc, char** argv) {
    init(argc, argv, "LaserScan");

    NodeHandle n;
    Subscriber sub = n.subscribe(TOPIC, 1000, detectPositions);
    pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

    spin();

    return 0;
}