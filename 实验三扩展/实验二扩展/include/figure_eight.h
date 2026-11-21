#pragma once
#include <vector>
#include <cmath>
#include <stdexcept>
namespace lab2ext {
struct Point { double x,y; };
inline std::vector<Point> path(double radius,bool outer,double x,double y,double yaw) {
  if(!std::isfinite(radius)||radius<0.5||radius>10)throw std::invalid_argument("Radius must be 0.5..10 m");
  constexpr double pi=3.14159265358979323846;
  const int n=static_cast<int>(std::ceil(2*pi*radius/0.04/2))*2;
  std::vector<Point> base,result;
  for(int i=0;i<2*n;++i){double a=2*pi*(i%n)/n;double side=i<n?1:-1;base.push_back({radius*std::sin(a),side*radius*(1-std::cos(a))});}
  const int start=outer?n/2:0;const auto origin=base[start];
  const double angle=yaw-(outer?pi:0),c=std::cos(angle),s=std::sin(angle);
  for(int i=0;i<=2*n;++i){auto p=base[(start+i)%(2*n)];double dx=p.x-origin.x,dy=p.y-origin.y;result.push_back({x+c*dx-s*dy,y+s*dx+c*dy});}
  return result;
}
}
