/*
 * Segmentation.h
 *
 *  Created on: Oct 31, 2011
 *      Author: asher
 */

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <pcl/pcl_base.h>
#include <pcl/search/search.h>

#include <list>
#include <map>

namespace bim {
namespace segmentation {

template<typename PointT>
class Segmentation : public pcl::PCLBase<PointT> {
public:
	Segmentation();

	typedef boost::shared_ptr<Segmentation<PointT> >  Ptr;

	int segment(std::vector<std::vector<int> >& segments);
	int segment(std::list<std::list<int> >& segments);

	void setSearch(typename pcl::search::Search<PointT>::Ptr search){
		search_ = search; }

	//The underlying surface can be a more dense cloud.
	void setSurfaceCloud(const typename pcl::PointCloud<PointT>::Ptr& surface_cloud,
						const typename pcl::search::Search<PointT>::Ptr& surf_search){
		this->surf_cloud_  =surface_cloud;
		surf_search_ = surf_search;
	}

	virtual ~Segmentation();

protected:
	virtual int applySegmentation(std::list<std::list<int> >& segments)=0;
	typename pcl::search::Search<PointT>::Ptr search_;
	typename pcl::PointCloud<PointT>::ConstPtr surf_cloud_;
	typename pcl::search::Search<PointT>::Ptr surf_search_;
};


void removeOutliers(int minimum_size, std::list<std::list<int> >& segments, std::list<std::list<int> >& outliers);


template< typename PointLabelT>
void labelCloud( std::list<std::list<int> >& segment_list,  pcl::PointCloud<PointLabelT>& cloud){

	for(int i=0; i<cloud.points.size(); i++){
		cloud.points[i].label =-1;
	}

	std::list<std::list<int> >::iterator seg_iter= segment_list.begin();
	for(int i=0; seg_iter != segment_list.end(); seg_iter++, i++){
		std::list<int>::iterator iter = seg_iter->begin();
		for(int k=0; iter!= seg_iter->end(); k++, iter++){
			cloud.points[*iter].label =  i;
		}
	}

}

template<typename PointLabelT, typename PointRGBT>
void colorLabels(const pcl::PointCloud<PointLabelT>& lcloud, pcl::PointCloud<PointRGBT>& cloud){
	int total_labels=0;
	for(int i=0; i<cloud.points.size(); i++){
			if (total_labels < lcloud.points[i].label)  total_labels = lcloud.points[i].label;
		}

	unsigned char colors[3*total_labels];
	for(int i=0; i<total_labels; i++){
		colors[i*3] = rand()%255;
		colors[i*3+1] = rand()%255;
		colors[i*3+2] = rand()%255;
	}

	for(int i=0; i<cloud.points.size(); i++){
		int l = lcloud.points[i].label;
		if (l<0){
			cloud.points[i].r = 0;
			cloud.points[i].g =0;
			cloud.points[i].b = 0;
		}
		else{
			int idx = 3*l;
			cloud.points[i].r = colors[idx];
			cloud.points[i].g = colors[idx+1];
			cloud.points[i].b = colors[idx+2];
		}
	}
}

template<typename PointLabelT>
void extractSegments(const pcl::PointCloud<PointLabelT>& lcloud, std::map<int, std::list<int> >& label_map );


} /* namespace segmentation */
} /* namespace bim */

#include "impl/Segmentation.hpp"
#endif /* SEGMENTATION_H_ */
