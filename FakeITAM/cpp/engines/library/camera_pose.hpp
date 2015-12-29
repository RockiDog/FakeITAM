//
//  camera_pose.hpp
//  FakeITAM
//
//  Created by Soap on 15/12/24.
//  Copyright © 2015年 Soap. All rights reserved.
//

#ifndef FAKEITAM_CPP_ENGINES_LIBRARY_CAMERA_POSE_HPP_
#define FAKEITAM_CPP_ENGINES_LIBRARY_CAMERA_POSE_HPP_

#include "utilities/matrix.hpp"
#include "utilities/vector.hpp"

namespace fakeitam {
namespace engine {

struct CameraPose {
  CameraPose() : m(utility::Matrix4f::Identity()) {}

  utility::Matrix3f R() const {
    return utility::Matrix3f {m(0, 0), m(0, 1), m(0, 2),
                              m(1, 0), m(1, 1), m(1, 2),
                              m(2, 0), m(2, 1), m(2, 2)};
  }
  utility::Vector3f t() const {
    return utility::Vector3f {m(0, 3), m(1, 3), m(2, 3)};
  }

  utility::Matrix4f m;
};

}
}

#endif  /* FAKEITAM_CPP_ENGINES_LIBRARY_CAMERA_POSE_HPP_ */
