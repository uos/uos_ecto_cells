#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <tf/transform_listener.h>

struct CloudReframer {
	static void declare_params(tendrils& p){
		p.declare(&CloudReframer::target_frame_, "target_frame", "The name of the target frame.", "/frame_id");
		p.declare(&CloudReframer::timeout_, "timeout", "Maximum number of seconds to wait for required transform.", 1.0);
		p.declare(&CloudReframer::tf_cache_time_, "tf_cache_time", "Number of seconds to keep TF transforms around", tf::Transformer::DEFAULT_CACHE_TIME);
	}

	static void declare_io(const tendrils& p, tendrils& input, tendrils& output){
		output.declare<ecto::pcl::PointCloud>(&CloudReframer::output_, "output", "Transformed Cloud.");
	}

	void configure(const tendrils& p, const tendrils& input, const tendrils& output){
		// pointclouds are sometimes _really_ slow, so the default 10s memory is often not enough
		tf_.reset(new tf::TransformListener(ros::Duration(*tf_cache_time_)));
	}

	template <typename Point>
	int process(const tendrils& input, const tendrils& output, boost::shared_ptr<const ::pcl::PointCloud<Point>>& cloud){
		typename ::pcl::PointCloud<Point>::Ptr transformed_cloud(new ::pcl::PointCloud<Point>);

		std_msgs::Header header= pcl_conversions::fromPCL(cloud->header);

		tf_->waitForTransform(*target_frame_, header.frame_id, header.stamp, ros::Duration(*timeout_));
		std::string reason_for_failure;
		if(!tf_->canTransform(*target_frame_, header.frame_id, header.stamp, &reason_for_failure)){
			ROS_ERROR("Could not transform PointCloud from %s to %s: %s", header.frame_id.c_str(), target_frame_->c_str(), reason_for_failure.c_str());
			*output_= ecto::pcl::xyz_cloud_variant_t(transformed_cloud);
			return ecto::BREAK;
		}

		tf::StampedTransform trans;
		tf_->lookupTransform(*target_frame_, header.frame_id, header.stamp, trans);

		pcl_ros::transformPointCloud(*cloud, *transformed_cloud, trans);
		transformed_cloud->header.frame_id= *target_frame_;

		*output_= ecto::pcl::xyz_cloud_variant_t(transformed_cloud);
		return ecto::OK;
	}

	boost::shared_ptr<tf::TransformListener> tf_;

	ecto::spore<std::string> target_frame_;
	ecto::spore<double> timeout_;
	ecto::spore<double> tf_cache_time_;

	ecto::spore<ecto::pcl::PointCloud> output_;
};

ECTO_CELL(uos_ecto_cells, ecto::pcl::PclCell<CloudReframer>, "CloudReframer",
          "Transforms a given PointCloud to a specific ROS frame")
