/*
 * segmentation.cpp
 *
 *  Created on: Nov 2, 2011
 *      Author: asher
 */

#include <bimlib/segmentation/Segmentation.h>

void bim::segmentation::removeOutliers(int minimum_size, std::list<std::list<int> >& segments, std::list<std::list<int> >& outliers){
	std::list<std::list<int> >::iterator iter = segments.begin();
	while( iter!= segments.end()){
		if ( iter->size() < minimum_size){
			outliers.push_back(*iter);
			std::list<std::list<int> >::iterator old = iter;

			iter++;
			segments.erase(old);

		}
		else{
			iter++;
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
