/*
 * SmoothNormal.h
 *
 *  Created on: Oct 31, 2011
 *      Author: Adam Stambler
 *
 *    This class is based on the paper
 *    SEGMENTATION OF POINT CLOUDS USING SMOOTHNESS CONSTRAINT
 *    by
 *    T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc
 *
 *	This class creates segments with smoothly varying normals.
 *	This implementation deviates from the paper because it does not
 *	calculate the residual for all points and then choose the points with the least
 *	residual as the first seeds.  Instead it uses
 */

#ifndef SMOOTHNORMAL_H_
#define SMOOTHNORMAL_H_

#include <bimlib/segmentation/Segmentation.h>

namespace bim {
namespace segmentation {
template<typename PointT>

class SmoothNormal: public Segmentation<PointT> {
public:
	SmoothNormal();
	typedef boost::shared_ptr<SmoothNormal<PointT> > Ptr;

	/*
	 * Theta is the  normal angle smooththess threshold for a seed point and its neighbors
	 * If a neighbor's normal is within theta radians, it is added to the cluster
	 * || np ns || > cos(theta)
	 */
	void setSmoothnessThreshold(const float& theta);

	/*
	 * rthresh controls the degree of over/under segmentation
	 * Points need to be within rthresh meters of the local plane of the
	 * current seed point in order to be used as a new seed point for clustering.
	 */
	void setResidualThreshhold(const float& rthresh);

	/** \brief set the number of nearest neighbors used for clustering
	 */
	void setClusterNbrs(int nbrs);

	void setInputNormals(const typename pcl::PointCloud<pcl::Normal>::Ptr& normals);


	virtual ~SmoothNormal();

protected:
	virtual int applySegmentation(std::list<std::list<int> >& segments);
	float smooth_thresh_;
	float rthresh_;
	boost::shared_ptr<pcl::PointCloud<pcl::Normal> > normals_;

	int cluster_N_nbrs;

	float residual_radius_;
	float calculateResidual(int idx);

};

/** \brief  Cluster based about the point given by the index seedi based on the smooth normal constraint
 *   seedi   -  seed point cloud index
 *   cloud   -  input point cloud
 *   normals -  point normals for input cloud
 *   search  -  search class for input cloud
 *   used    -  boolean array indicating whether the points have already been used in a cluster
 *   nk
 */
template<typename PointT, typename PointN>
int smoothNormalCluster(int seedi, std::list<int>& segment,
                        const pcl::PointCloud<PointT>& cloud, const pcl::PointCloud<PointN>& normals,
                        pcl::search::Search<PointT>& search, std::vector<bool>& used, int nk =30,
                        float theta_thresh = 0.1, float residual_thresh = 0.05);

} /* namespace segmentation */
} /* namespace bim */

#endif /* SMOOTHNORMAL_H_ */
