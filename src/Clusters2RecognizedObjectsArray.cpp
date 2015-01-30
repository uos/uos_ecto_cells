#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_conversions/pcl_conversions.h>

#include <boost/make_shared.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

struct Clusters2RecognizedObjectArray {
	static void declare_params(tendrils& p){}

	static void declare_io(const tendrils& p, tendrils& input, tendrils& output){
		input.declare(&Clusters2RecognizedObjectArray::indices_, "indices", "List of cluster indices.");

		output.declare(&Clusters2RecognizedObjectArray::output_, "output", "RecognizedObjectArray with PointClouds.");
	}

	void configure(const tendrils& p, const tendrils& input, const tendrils& output){}

	template <typename PointT>
	int process(const tendrils& input, const tendrils& output, boost::shared_ptr<const ::pcl::PointCloud<PointT>>& cloud){
		object_recognition_msgs::RecognizedObjectArray::Ptr out= boost::make_shared<object_recognition_msgs::RecognizedObjectArray>();
		out->objects.reserve( indices_->size() );

		std::string object_key("PointCloud");
		std::string database_type("{\"type\": \"empty\"}");

		::pcl::ExtractIndices<PointT> extractor;
		extractor.setInputCloud(cloud);

		for(const ::pcl::PointIndices& i : *indices_){
			::pcl::PointIndices::ConstPtr ip= boost::make_shared<const ::pcl::PointIndices>(i);
			extractor.setIndices(ip);

			::pcl::PointCloud<PointT> cluster;
			extractor.filter(cluster);

			if( cluster.size() == 0 )
				continue;

			sensor_msgs::PointCloud2 view;
			pcl::toROSMsg(cluster, view);

			object_recognition_msgs::RecognizedObject obj;
			obj.pose.header= obj.header= view.header;

			geometry_msgs::Pose& p= obj.pose.pose.pose;

			Eigen::Vector4f minpt, maxpt;
			pcl::getMinMax3D(cluster, minpt, maxpt);
			p.position.x= (minpt[0]+maxpt[0])/2;
			p.position.y= (minpt[1]+maxpt[1])/2;
			p.position.z= (minpt[2]+maxpt[2])/2;

			p.orientation.x= p.orientation.y= p.orientation.z= 0; p.orientation.w= 1;

			obj.type.key= object_key;
			obj.type.db= database_type;
			obj.point_clouds.reserve(1);
			obj.point_clouds.push_back(view);

			out->objects.push_back( obj );
		}

		out->header= pcl_conversions::fromPCL(cloud->header);

		*output_= out;
		return ecto::OK;
	}

	ecto::spore<std::vector< ::pcl::PointIndices >> indices_;
	ecto::spore<object_recognition_msgs::RecognizedObjectArray::ConstPtr> output_;
};

ECTO_CELL(uos_ecto_cells, ecto::pcl::PclCell<Clusters2RecognizedObjectArray>, "Clusters2RecognizedObjectArray",
          "Package a set of clusters into one RecognizedObjectArray.")
