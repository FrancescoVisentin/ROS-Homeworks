#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <cmath>
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
        float angle = i*msg->angle_increment;
        
        if (range >= msg->range_min && range <= msg->range_max) {
            Point p = Point(range*cos(angle), range*sin(angle));
            detectedPoints.push_back(p);
        }
    }

    // Use k-means clustering and the silhoutte coefficient to guess the number of people
    // General idea: if we suppose to have N people then we will have 2*N feet.
    //      Mapping the detected points in space we then expect to compute two "good" clustering
    //      for two different values of K:
    //      1) Fixing K=2*N we expect to assign a center to each points cloud of each foot
    //      2) Fixing K=N we expect to assign a center in between each pair of feet
    //
    // Therefore we select the number of people N as follow:
    //      - At each iter increase N by 1 and compute two clusteings fixing K=N and K=2*N
    //      - Compute and sum the silhouette coefficients for the two clusterings
    //      - Select N from the iteration with the highest sum 
    //
    float bestSilhouette = -2;
    vector<Point> bestFeetPos;
    vector<Point> bestPeoplePos;
    for (int K = 1; K < MAX_PEOPLE; K++) {
        // Get the position of each person
        vector<Point> peopleDetected = detectedPoints;
        vector<Point> peoplePos = kmeans(peopleDetected, K, false);
        float s = silhouette(peopleDetected, K);

        // Get the position of each foot
        vector<Point> feetDetected = detectedPoints;
        vector<Point> feetPos = kmeans(feetDetected, K*2, true);
        s += silhouette(feetDetected, K*2);

        ROS_INFO("Sum of silhouettes for %d person and %d feet ---> %f", K, 2*K, s);
        if (bestSilhouette < s) {
            bestPeoplePos = peoplePos;
            bestFeetPos = feetPos;
            bestSilhouette = s;
        }
    }

    ROS_INFO("Detected number of people: %ld", bestPeoplePos.size());
    printPoints(bestFeetPos, bestPeoplePos);
}


int main(int argc, char** argv) {
    init(argc, argv, "LaserScan");

    NodeHandle n;
    Subscriber sub = n.subscribe(TOPIC, 1000, detectPositions);
    pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

    spin();

    return 0;
}