#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace lab26 {
// Supplement the lidar-plane obstacle band with returns above a locally
// measured floor. A nearby change of floor level invalidates the estimate.
class GroundReturns {
 public:
  void observe(double x,double y,double z){
    if(!inside(x,y)||z<-.72||z>-.35||!std::isfinite(z))return;
    auto& cell=cells_[index(x,y)];
    for(auto& low:cell.low)if(z<low)std::swap(z,low);
  }
  void finish(){
    for(int j=0;j<side;++j)for(int i=0;i<side;++i){
      std::array<double,9> floor{};
      size_t n=0;
      for(int y=std::max(0,j-1);y<=std::min(side-1,j+1);++y)
        for(int x=std::max(0,i-1);x<=std::min(side-1,i+1);++x){
          const double z=cells_[y*side+x].low[2];
          if(z<-.38)floor[n++]=z;
        }
      if(n<4)continue;
      std::sort(floor.begin(),floor.begin()+n);
      const double low=floor[n/4],high=floor[(3*n)/4];
      if(high-low<=.055)cells_[j*side+i].floor=high;
    }
  }
  bool obstacle(double x,double y,double z)const{
    if(std::abs(z)<=.35)return true;
    if(z>=-.35||!inside(x,y))return false;
    const double floor=cells_[index(x,y)].floor;
    return floor<0. && z>=floor+.085;
  }
 private:
  static constexpr int side=16;
  struct Cell{std::array<double,3> low{{0.,0.,0.}};double floor=0.;};
  std::array<Cell,side*side> cells_{};
  static bool inside(double x,double y){return x>=-4. && x<4. && y>=-4. && y<4.;}
  static int index(double x,double y){return int((y+4.)*2)*side+int((x+4.)*2);}
};
}
