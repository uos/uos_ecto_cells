#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <Eigen/Eigenvalues>

#include <pcl_conversions/pcl_conversions.h>

#include <boost/make_shared.hpp>

#include <geometry_msgs/Point.h>
#include <object_recognition_msgs/Table.h>
#include <object_recognition_msgs/TableArray.h>

namespace {
	geometry_msgs::Point convert(const Eigen::Vector3f& x){
		geometry_msgs::Point p;
		p.x= x[0];
		p.y= x[1];
		p.z= x[2];
		return p;
	}

	geometry_msgs::Quaternion convert(const Eigen::Quaternionf& x){
		geometry_msgs::Quaternion q;
		q.x= x.coeffs()[0];
		q.y= x.coeffs()[1];
		q.z= x.coeffs()[2];
		q.w= x.coeffs()[3];
		return q;
	}
}

struct ConvexHull2Table {
	static void declare_params(tendrils& p){}

	static void declare_io(const tendrils& p, tendrils& input, tendrils& output){
		output.declare(&ConvexHull2Table::output_, "output", "TableArray message containing the convex hull.");
	}

	void configure(const tendrils& p, const tendrils& input, const tendrils& output){}

	template <typename PointT>
	int process(const tendrils& input, const tendrils& output, boost::shared_ptr<const ::pcl::PointCloud<PointT>>& cloud){
		object_recognition_msgs::TableArray::Ptr out= boost::make_shared<object_recognition_msgs::TableArray>();
		out->tables.resize(1);
		object_recognition_msgs::Table& table= out->tables[0];

		out->header= table.header= pcl_conversions::fromPCL(cloud->header);

		// compute pose
		Eigen::Vector4f minpt, maxpt;
		pcl::getMinMax3D(*cloud, minpt, maxpt);
		const Eigen::Vector4f center4f((minpt+maxpt)/2);
		const Eigen::Vector3f center(center4f[0], center4f[1], center4f[2]);
		table.pose.position= convert(center);

		Eigen::Matrix3f cov;
		pcl::computeCovarianceMatrixNormalized(*cloud, center4f, cov);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);

		Eigen::Matrix3f basis= solver.eigenvectors();
		// eigen returns the eigenvectors by _increasing_ eigenvalues and we want them the other way around
		basis.col(2).swap(basis.col(0));

		// basis could be degenerate, so compute third dimension orthogonal to first two eigenvectors
		basis.col(2)= basis.col(0).cross(basis.col(1));

		// x axis should point away from the camera
		basis.col(0)*= (center.dot(basis.col(0)) > 0) ? 1 : -1;
		// z axis should point towards the camera
		basis.col(2)*= (center.dot(basis.col(2)) < 0) ? 1 : -1;

		// make sure the rotation matrix' determinant is 1 and not -1
		basis.col(1)*= basis.determinant();

		table.pose.orientation= convert(Eigen::Quaternionf(basis));

		// describe points relative to pose
		table.convex_hull.reserve(cloud->size());
		for(const PointT& p : cloud->points)
			table.convex_hull.push_back(convert( basis.transpose()*(Eigen::Vector3f(p.x, p.y, p.z) - center) ));

		*output_= out;
		return ecto::OK;
	}

	ecto::spore<object_recognition_msgs::TableArray::ConstPtr> output_;
};

ECTO_CELL(uos_ecto_cells, ecto::pcl::PclCell<ConvexHull2Table>, "ConvexHull2Table",
          "Take a ConvexHull and build a Table(Array) message out of it.")
