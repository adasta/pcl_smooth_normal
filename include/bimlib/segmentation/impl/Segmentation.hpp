/*
 * Segmentation.cpp
 *
 *  Created on: Oct 31, 2011
 *      Author: asher
 */
#ifndef _SEGMENTATION_HPP__
#define _SEGMENTATION_HPP__

#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <list>
#include <map>

namespace bim {
namespace segmentation {

template<typename PointT>
Segmentation<PointT>::Segmentation() {

}

template<typename PointT>  int
Segmentation<PointT>::segment(std::vector<std::vector<int> >& segments){

	std::list<std::list<int> > segment_list;

	int total_segments = applySegmentation(segment_list);

	//do some book keeping and move things into vectors
	segments.resize(segment_list.size());
	std::list<std::list<int> >::iterator seg_iter= segment_list.begin();
	for(int i=0; seg_iter != segment_list.end(); seg_iter++, i++){
		segments[i].resize(seg_iter->size());
		std::list<int>::iterator iter = seg_iter->begin();
		for(int k=0; iter!= seg_iter->end(); k++, iter++){
			segments[i][k] = *iter;
		}
	}

	return total_segments;
}

template<typename PointT>  int
Segmentation<PointT>::segment(std::list<std::list<int> >& segments){
	if (! this->initCompute()) return 0;
		if (search_ == NULL){
		if (this->input_->isOrganized()) {
			boost::shared_ptr< pcl::search::OrganizedNeighbor<PointT> > searcher(new pcl::search::OrganizedNeighbor<PointT> );
			search_ =searcher;
		}
		else {
			boost::shared_ptr< pcl::search::Octree<PointT> > searcher(new pcl::search::Octree<PointT>(0.04) );
			search_=searcher;
		}
		search_->setInputCloud(this->input_);
		}

		if (surf_cloud_ == NULL){
			surf_cloud_ = this->input_;
			surf_search_ = search_;
		}

		int total_segments = applySegmentation(segments);

		return total_segments;
}

template<typename PointT>
Segmentation<PointT>::~Segmentation() {
	// TODO Auto-generated destructor stub
}


template<typename PointLabelT>
void extractSegments(const pcl::PointCloud<PointLabelT>& lcloud, std::map<int, std::list<int> >& label_map ){

	for(int i=0; i< lcloud.points.size(); i++){
		int l = lcloud.points[i].label ;
		if (label_map.count(l)) label_map[l].push_back(i);
		else {
			std::list<int> seg;
			label_map[l] = seg;
			label_map[l].push_back(i);
		}
	}
}

} /* namespace segmentation */
} /* namespace bim */

#endif
