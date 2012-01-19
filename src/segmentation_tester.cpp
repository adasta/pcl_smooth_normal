/*
 * segmentation_tester.cpp
 *
 *  Created on: Nov 1, 2011
 *      Author: Adam Stambler
 *
 *  This program is a command line interface to the bim::segmentation algorithms
 *  it allows easy testing of segmentation results.
 */


#include <bimlib/segmentation/SmoothNormal.h>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <bimlib/point_types.h>

#include <string>
#include <iostream>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/surface/mls.h>

namespace po=boost::program_options;
using namespace std;




//Labels the output with the max vote of the k nearest neighbors
template<typename PointTLIn, typename PointTS, typename PointTLOut>
void NNLabel(const pcl::PointCloud<PointTLIn>& licloud, const boost::shared_ptr<pcl::search::Search<PointTS> >& lisearch,
			pcl::PointCloud<PointTLOut>& locloud, int knn=5){

	for(int i=0; i<locloud.points.size(); i++){
		std::vector<int> nbrs;
		std::vector<float> dist;
		PointTS p;
		p.x=locloud.points[i].x;
		p.y=locloud.points[i].y;
		p.z=locloud.points[i].z;
		lisearch->nearestKSearch(p, knn, nbrs, dist);
		std::map<int, int> votes;
		for (int k=0; k<nbrs.size(); k++) {
			votes[ licloud.points[ nbrs[k] ].label ]++;
		}
		std::map<int,int>::iterator iter;

		int max_vote_amount=0;
		int max_vote_label = 0;

		for(iter = votes.begin(); iter!= votes.end(); iter++ ){
			if (iter->second> max_vote_amount) {
				max_vote_amount = iter->second;
				max_vote_label = iter->first;
			}
		}

		locloud.points[i].label = max_vote_label;
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


template<typename PointT>
void run(typename pcl::PointCloud<PointT>::Ptr& surface_cloud, po::variables_map& vm, std::string& output_file){

	  boost::shared_ptr< pcl::search::KdTree<PointT> > surf_search( new pcl::search::KdTree<PointT>);
	  surf_search->setInputCloud(surface_cloud);
	  boost::shared_ptr< pcl::search::KdTree<PointT> > ds_search( new pcl::search::KdTree<PointT>(0.02));

	typename bim::segmentation::Segmentation<PointT>::Ptr segmenter;

	typename  pcl::PointCloud<PointT>::Ptr ds_cloud(new pcl::PointCloud<PointT>());
	 if(vm.count("voxelize")){
		       float leaf_size = vm["voxelize"].as<float >();
			   std::cout << " Down sampling Point cloud with " << leaf_size << " m grid.\n";
			   pcl::VoxelGrid<PointT> vg;
			   vg.setLeafSize(leaf_size,leaf_size, leaf_size);
			   vg.setInputCloud(surface_cloud);
			   vg.setDownsampleAllData(true);
			   vg.filter(*ds_cloud);
			   ds_search->setInputCloud(ds_cloud);
	 }
	else{
		 ds_cloud = surface_cloud;
		 ds_search = surf_search;
	}

	 surf_search->setInputCloud(surface_cloud);

	 pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		//segmentation needs normals
		if (vm.count("smooth_normal")){
			pcl::NormalEstimation<PointT, pcl::Normal> nest;
			nest.setSearchSurface(surface_cloud);
			nest.setInputCloud(ds_cloud);
			nest.setKSearch(30);
			nest.compute(*normals);
		}

	   if (vm.count("smooth_normal")){
		   std::cout<< "Segmentation with a smooth normal constraint chosen.\n";

		   std::vector<float> params = vm["smooth_normal"].as<std::vector<float> >();
	        bim::segmentation::SmoothNormal<pcl::PointXYZ>::Ptr snseg(new bim::segmentation::SmoothNormal<pcl::PointXYZ>());

	        float k=-1, bandwidth=-1;
	        if (params.size()==2){
	        	snseg->setSmoothnessThreshold(params[0]);
	        	snseg->setResidualThreshhold(params[1]);
	        }
	        else if (params.size()==1){
	        	snseg->setSmoothnessThreshold(params[0]);
	        }
	        else{
	        	snseg->setSmoothnessThreshold(0.20);
	        	snseg->setResidualThreshhold(0.01);
	        }
	        segmenter = snseg;
	        snseg->setInputNormals(normals);
	    }


	   std::cout << "Now segmenting!\n";

	   segmenter->setInputCloud(ds_cloud);
	   segmenter->setSearch(ds_search);
	   segmenter->setSurfaceCloud(surface_cloud,surf_search);


	   std::list<std::list<int> > segments;
	   segmenter->segment(segments);

	   std::cout<< "There were " << segments.size() << " segments \n";

	   if (vm.count("minimum_size") ){
		   int min_size = vm["minimum_size"].as<int>();
		   std::list<std::list<int> > outliers;
		   bim::segmentation::removeOutliers(min_size, segments,outliers);
		   std::cout << "Removed " << outliers.size() << " segments smaller than " << min_size <<" and left " << segments.size() << "segments\n";
	   }

	   pcl::PointCloud<bim::PointXYZL>::Ptr lcloud(new pcl::PointCloud<bim::PointXYZL>());

	   pcl::copyPointCloud(*ds_cloud, *lcloud);
	   bim::segmentation::labelCloud<bim::PointXYZL>( segments,*lcloud);

	   if (vm.count("label_surface")){
		   std::cout << "Labeling the surface!\n";
		   pcl::PointCloud<bim::PointXYZL>::Ptr lscloud(new pcl::PointCloud<bim::PointXYZL>()); //surface cloud
		   pcl::copyPointCloud(*surface_cloud, *lscloud);
		   NNLabel<bim::PointXYZL, PointT,  bim::PointXYZL>(*lcloud, ds_search, *lscloud);
		   std::cout << "Saving surface cloud labels\n";

		   pcl::io::savePCDFile(output_file+"_surf_seg.pcd",*lscloud);

	   }

	   pcl::PointCloud<bim::PointXYZNormalL> lncloud;
	   pcl::copyPointCloud(*lcloud, lncloud);
	   pcl::copyPointCloud(*normals, lncloud);
	   std::string lnfile = output_file+"_seg_norms.pcd";
	   pcl::io::savePCDFile(lnfile,lncloud,true);
	   
	   pcl::PointCloud<pcl::PointXYZRGB> ccloud;
	   pcl::copyPointCloud(*ds_cloud, ccloud);
	   colorLabels(*lcloud, ccloud);
	   pcl::io::savePCDFile(output_file+"_cseg.pcd",ccloud, true);
	   std::cout << "Now saving colored segmentation to " << output_file+"_cseg.pcd" << " \n";


	   output_file = output_file+"_seg.pcd";
	   std::cout << "Now saving labeled segmentation to " << output_file << " \n";
	   
	   pcl::io::savePCDFile(output_file,*lcloud,true);
}




int main(int argc, char** argv){
  po::options_description desc("./segmentation_tester input [options]\nOptions:");

  desc.add_options()
	  ("input-file,i",po::value<std::string>()->required(), "input pcd ")
	  ("smooth_normal,n", po::value<std::vector<float> >()->multitoken(),
			         "Smooth Normal -theta residual [ 0.20 0.01]")
	  ("output,o", po::value<std::string>(), "specify output file")
	  ("voxelize,v",po::value<float>(), "voxelize with leaf size v")
	  ("oulier,O","statistical outlier removal")
	  ("minimum_size,M", po::value<int>(), "Remove segments smaller than arg")
	  ("label_surface,L", "Output a Labeled surface cloud")
	  ;

  if (argc <2 ){
	   std::cout << "Incorrect number of arguments\n";
	   desc.print(cout);
	   return -1;
   }

  	  po::positional_options_description p;
 	  p.add("input-file", -1);

 	  po::variables_map vm;
 	  po::store(po::command_line_parser(argc, argv).
 	  options(desc).positional(p).run(), vm);
 	  po::notify(vm);


   string input_file = vm["input-file"].as<std::string>();
   std::cout << "input file : " << input_file << std::endl;

   string output_file;

   if (vm.count("output")){
	   output_file = vm["output"].as<std::string >();
   }
   else{
	   std::string path, fname, ext;
	   int start = input_file.rfind('/'), end = input_file.rfind('.');
	   output_file = input_file.substr(start+1, end-start-1);
   }

   std::cout << "Loading Point Cloud : " << input_file << std::endl;

   pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointXYZ>());
   pcl::io::loadPCDFile(input_file,*surface_cloud);
   run<pcl::PointXYZ>(surface_cloud, vm, output_file);

return 0;
}
