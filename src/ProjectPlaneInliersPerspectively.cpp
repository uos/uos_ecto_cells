#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Geometry>

#include <boost/make_shared.hpp>

struct ProjectPlaneInliersPerspectively {
	static void declare_params(tendrils& p){}

	static void declare_io(const tendrils& p, tendrils& input, tendrils& output){
		input.declare(&ProjectPlaneInliersPerspectively::indices_, "indices", "Indices of points of interest in input.");
		input.declare(&ProjectPlaneInliersPerspectively::model_, "model", "Model of plane.");
		output.declare(&ProjectPlaneInliersPerspectively::output_, "output", "Projected points.");
	}

	void configure(const tendrils& p, const tendrils& input, const tendrils& output){}

	template <typename Point>
	int process(const tendrils& input, const tendrils& output, boost::shared_ptr<const ::pcl::PointCloud<Point>>& cloud){
		auto projected_points= boost::make_shared< ::pcl::PointCloud<Point> >(*cloud);

		Eigen::Vector3f plane_normal= Eigen::Vector3f::Map(&(*model_)->values[0]);
		Eigen::Hyperplane<float,3> plane(plane_normal, (*model_)->values[3]);

		if(!indices_.user_supplied()){
			for( auto& point : projected_points->points )
				point.getVector3fMap()= Eigen::ParametrizedLine<float,3>::Through(Eigen::Vector3f::Zero(), point.getVector3fMap()).intersectionPoint(plane);
		}
		else {
			for( const auto& i : (*indices_)->indices )
				projected_points->points[i].getVector3fMap()= Eigen::ParametrizedLine<float,3>::Through(Eigen::Vector3f::Zero(), projected_points->points[i].getVector3fMap()).intersectionPoint(plane);
		}

		*output_= ecto::pcl::xyz_cloud_variant_t(projected_points);
		return ecto::OK;
	}

	ecto::spore<pcl::PointIndices::ConstPtr> indices_;
	ecto::spore<pcl::ModelCoefficients::ConstPtr> model_;
	ecto::spore<ecto::pcl::PointCloud> output_;
};

ECTO_CELL(uos_ecto_cells, ecto::pcl::PclCell<ProjectPlaneInliersPerspectively>, "ProjectPlaneInliersPerspectively",
          "Projects points on a plane perspectively.")
