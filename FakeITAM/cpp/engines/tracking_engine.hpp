//
//  tracking_engine.hpp
//  FakeITAM
//
//  Created by Soap on 15/11/21.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_TRACKING_ENGINE_HPP_
#define FAKEITAM_CPP_ENGINES_TRACKING_ENGINE_HPP_

#include "utilities/matrix.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

struct CameraPose;
struct View;
class PointCloud;

class TrackingEngine {
 public:
  TrackingEngine() = default;
  virtual ~TrackingEngine() = default;

  virtual void TrackCamera(const View& view_in,
                           const PointCloud& global_pcl_in_,
                           const CameraPose& pose_in,
                                 CameraPose* pose_out) = 0;

 private:
  TrackingEngine(const TrackingEngine&);
  TrackingEngine& operator=(const TrackingEngine&);
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_TRACKING_ENGINE_HPP_ */
