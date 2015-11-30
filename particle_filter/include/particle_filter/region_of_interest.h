#ifndef REGION_OF_INTEREST_H_
#define REGION_OF_INTEREST_H_

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<Eigen/StdVector>

#include <stdlib.h>
#include <opencv2/opencv.hpp>


namespace particle_filter
{
typedef  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ListVector2d;

class RegionOfInterest{
public:
    RegionOfInterest();

    void filterCandidate(ListVector2d &left, ListVector2d &right);

    cv::Rect getCvRect() const;
    int getX() const;
    int getY() const;
    int getWidth() const;
    int getHeight() const;
    void setX(const int x);
    void setY(const int y);
    void setWidth(const int w);
    void setHeight(const int h);
private:
    void setBoundaryAroundTarget(Eigen::Vector2d left, Eigen::Vector2d right);
    bool hit(Eigen::Vector2d v);
    bool hit(Eigen::Vector2d l, Eigen::Vector2d r);

    int x, y, w, h;
    unsigned int forgetCounter;
    static const unsigned int MAX_FORGET = 20;
};

}


#endif /* REGION_OF_INTEREST_H_ */
