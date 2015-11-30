#include "particle_filter/region_of_interest.h"

using namespace std;

namespace particle_filter
{

RegionOfInterest::RegionOfInterest():
    x(0),
    y(0),
    w(0),
    h(0),
    forgetCounter(0){
}

void RegionOfInterest::filterCandidate(ListVector2d &left,
                                       ListVector2d &right){
    ListVector2d filteredLeft;
    ListVector2d filteredRight;

    for(int i = 0; i < left.size(); i++){
        if(this->hit(left[i], right[i])){
            filteredLeft.push_back(left[i]);
            filteredRight.push_back(right[i]);
        }
    }
    if(filteredLeft.size() == 0){
       this->forgetCounter++;
    }
    else{
        this->forgetCounter = 0;
        this->setBoundaryAroundTarget(filteredLeft[0], filteredRight[0]);
    }


    if(this->forgetCounter > this->MAX_FORGET && !left.empty()){
        ROS_INFO("Forget current boundary, choose one at random");
        int r = rand() % left.size();
        this->setBoundaryAroundTarget(left[r], right[r]);
        filteredLeft.push_back(left[r]);
        filteredRight.push_back(right[r]);
    }
    left = filteredLeft;
    right = filteredRight;
    ROS_INFO("Size: %i", left.size());
}

bool RegionOfInterest::hit(Eigen::Vector2d v){
    return v[0] >= x && v[1] >= y &&
           v[0] <= x + w && v[1] <= y + h;
}

bool RegionOfInterest::hit(Eigen::Vector2d l, Eigen::Vector2d r){
    return this->hit(l) &&  this->hit(r);
}

void RegionOfInterest::setBoundaryAroundTarget(Eigen::Vector2d left, Eigen::Vector2d right){
    Eigen::Vector2d center = (right - left) *0.5 + left;
    double d = right[0] - left[0];
    d *= 4;
    w = d;
    h = d*0.5;
    x = center[0] - w*0.5 ;
    y = center[1] - h*0.5;
}

cv::Rect RegionOfInterest::getCvRect() const{
    return cv::Rect(x, y, w, h);
}

int RegionOfInterest::getX() const{
    return x;
}
int RegionOfInterest::getY() const{
    return y;
}
int RegionOfInterest::getWidth() const{
    return w;
}
int RegionOfInterest::getHeight() const{
    return h;
}
void RegionOfInterest::setX(const int x){
    this->x = x;
}

void RegionOfInterest::setY(const int y){
    this->y = y;
}
void RegionOfInterest::setWidth(const int w){
    this->w = w;
}
void RegionOfInterest::setHeight(const int h){
    this->h = h;
}

} // namespace particle_filter

