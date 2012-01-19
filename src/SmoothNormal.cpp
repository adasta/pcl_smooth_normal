/*
 * SmoothNormal.cpp
 *
 *  Created on: Oct 31, 2011
 *      Author: Adam Stambler
 */

#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#include <bimlib/segmentation/SmoothNormal.h>
#include <bimlib/segmentation/impl/SmoothNormal.hpp>

namespace bim{
namespace segmentation{
PCL_INSTANTIATE(SmoothNormal, PCL_XYZ_POINT_TYPES);
}
}



