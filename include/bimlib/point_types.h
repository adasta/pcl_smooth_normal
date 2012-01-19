/*
 * point_types.h
 *
 *  Created on: Oct 28, 2011
 *      Author: asher
 */

#ifndef POINT_TYPES_H_
#define POINT_TYPES_H_

#include <pcl/point_types.h>
 #include <pcl/point_cloud.h>

namespace bim {
 struct PointXYZL
 {
   PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
   int32_t label;
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
 } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment


 struct PointXYZNormalL
 {
   PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
   PCL_ADD_NORMAL4D;
   union
     {
       struct
       {
         float curvature;
       };
       float data_c[4];
     };
   int32_t label;
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
 } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment


 struct PointLabel{
	 int32_t label;
 };

}


POINT_CLOUD_REGISTER_POINT_STRUCT ( bim::PointXYZNormalL,
                                    (float , x, x)
                                    (float, y, y )
                                    (float, z, z )
                                    (float, normal_x, normal_x)
                                    (float, normal_y, normal_y)
                                    (float, normal_z, normal_z)
                                    (float, curvature, curvature)
							        (int32_t, label , label)
)


POINT_CLOUD_REGISTER_POINT_STRUCT ( bim::PointXYZL,
                                    (float , x, x)
                                    (float, y, y )
                                    (float, z, z )
			            (int32_t, label , label)
)


POINT_CLOUD_REGISTER_POINT_STRUCT ( bim::PointLabel,
                                   (int32_t, label , label)
)


POINT_CLOUD_REGISTER_POINT_STRUCT ( pcl::RGB,          
                                  (float, rgba, rgb)
                                  )
                                  

#endif /* POINT_TYPES_H_ */
