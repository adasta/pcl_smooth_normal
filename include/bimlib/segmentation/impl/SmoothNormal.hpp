/*
 * SmoothNormal.cpp
 *
 *  Created on: Oct 31, 2011
 *      Author: Adam Stambler
 */

#include <bimlib/segmentation/SmoothNormal.h>

#include <queue>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <list>

namespace bim {
namespace segmentation {

template<typename PointT>
SmoothNormal<PointT>::SmoothNormal() {
	smooth_thresh_ =  30.0f/180.0f*M_PI;
	rthresh_ =  0.05;
	cluster_N_nbrs = 30;
}

template<typename PointT> void
SmoothNormal<PointT>::setSmoothnessThreshold(const float& theta){
	assert(theta>0);
	this->smooth_thresh_= theta;
}


template<typename PointT> void
SmoothNormal<PointT>::setResidualThreshhold(const float& rthresh){
	rthresh_ =rthresh;
}

template<typename PointT> void
SmoothNormal<PointT>::setInputNormals(const typename pcl::PointCloud<pcl::Normal>::Ptr& normals){
	this->normals_ = normals;
}

template<typename PointT> void
SmoothNormal<PointT>::setClusterNbrs(int nbrs){
  cluster_N_nbrs  = nbrs;
}

template<typename PointT, typename PointN>
int smoothNormalCluster(int seedi, std::list<int>& segment,
                                  const pcl::PointCloud<PointT>& cloud, const pcl::PointCloud<PointN>& normals,
                                  pcl::search::Search<PointT>& search, std::vector<bool>& used, int nk =30,
                                  float theta_thresh = 0.1, float residual_thresh = 0.05){

	std::priority_queue<std::pair<float,int> > seeds;

	seeds.push(std::make_pair(0,seedi));
	segment.push_back(seedi);

	float costheta_thresh = cos(theta_thresh);
        used[seedi] = true;
	while (!seeds.empty()){
		int seedi = seeds.top().second;
		seeds.pop();

		std::vector<int>   nbr_idx;
		std::vector<float> nbr_dist;
		search.nearestKSearch(seedi, nk, nbr_idx, nbr_dist);
		Eigen::Map<Eigen::Vector3f> seed_n((float*) normals.points[seedi].normal);
		Eigen::Map<Eigen::Vector3f> seed_p((float*) cloud.points[seedi].data);
		for(int i=0; i < nbr_idx.size(); i++){
                    int idx = nbr_idx[i];
                    if (used[idx]) continue;

                    Eigen::Map<Eigen::Vector3f> nbr_n((float*) normals.points[idx].normal);

                    if ( fabs(nbr_n.dot(seed_n)) < costheta_thresh ) continue;
                    used[idx] = true;
                    segment.push_back(idx);

                    //Calculate residual and see if is small enough
                    Eigen::Map<Eigen::Vector3f> nbr_p((float*) cloud.points[idx].data);
                    float res = fabs(seed_n.dot(seed_p-nbr_p));

                    if (res < residual_thresh){
                            seeds.push(std::pair<float, int>(res,idx));
                    }
		}
	}
	return segment.size();
}

/*
 * Get the residual
 */

template<typename PointT> float
SmoothNormal<PointT>::calculateResidual(int idx){
	std::vector<int> knbrs;
	std::vector<float> kdist;

	this->search_->nearestKSearch(idx, cluster_N_nbrs, knbrs, kdist);
	Eigen::Map<Eigen::Vector3f> idxp((float*) this->input_->points[idx].data);
	Eigen::Map<Eigen::Vector3f> idxn(this->normals_->points[idx].normal);
	float res=0;
	for(int i=0; i<knbrs.size(); i++){
		Eigen::Map<Eigen::Vector3f> pt((float*) this->input_->points[knbrs[i]].data);
		res = res + fabs(idxn.dot((pt-idxp)));
	}
	return res/(float) knbrs.size();
}

template<typename PointT> int
SmoothNormal<PointT>::applySegmentation(std::list<std::list<int> >& segments){

	std::vector<float> pt_residuals;
	pt_residuals.resize(this->input_->points.size(), -1);

	std::vector<bool> used;
	used.resize(this->input_->points.size(), false);

	int pts_segmented =0;
	int search_idx=0;
	while(pts_segmented < this->input_->points.size()){
		//iterate through the points and either find a
	        // point which is a really low residual or find the lowest
	        // residual left.
		int idx =-1;
		float res = std::numeric_limits<float>::max();

		for(int i=search_idx+1; i<this->input_->points.size(); i++){
			if(used[i]) continue;
			if(pt_residuals[i] < 0){
                            pt_residuals[i] = calculateResidual(i);
			}
			if (pt_residuals[i] <  rthresh_*2){ // if its 2 times smaller than rthresh
                            idx = i;                     // its small enough to use as a seed point
                            break;
			}
			else{
			  if (res > pt_residuals[i] ){
			    idx = i;
			    res = pt_residuals[i];
			  }
			}
		}

		if (idx >=0){
			std::list<int> nseg;
			int s = smoothNormalCluster(idx,nseg,*this->input_, *this->normals_, *this->search_,
						used,cluster_N_nbrs, this->smooth_thresh_,rthresh_);
			std::cout << " Created segment " << idx << " with "  << s << " pts \n";
			if (s>2) segments.push_back(nseg);
			pts_segmented += s;
			search_idx = idx;
		}
		else{
			std::cout << "There was no good seed\n";
			break;
		}
	}
	std::cout << "There were "  << segments.size() << " segments \n";
	return segments.size();
}


template<typename PointT>
SmoothNormal<PointT>::~SmoothNormal() {

}

#define PCL_INSTANTIATE_SmoothNormal(T) template class PCL_EXPORTS SmoothNormal<T>;

} /* namespace segmentation */
} /* namespace bim */
