#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}
	/*
	//remove NAN
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
*/
	// We must build a condition.
	// And "And" condition requires all tests to check true. "Or" conditions also available.
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
	// First test, the point's Z value must be greater than (GT) 0.
	condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
	// Second test, the point's Z value must be less than (LT) 2.
	condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2.0)));
	// Checks available: GT, GE, LT, LE, EQ.

	// Filter object.
	pcl::ConditionalRemoval<pcl::PointXYZ> filter;
	filter.setCondition(condition);
	filter.setInputCloud(cloud);
	// If true, points that do not pass the filter will be set to a certain value (default NaN).
	// If false, they will be just removed, but that could break the structure of the cloud
	// (organized clouds are clouds taken from camera-like sensors that return a matrix-like image).
	filter.setKeepOrganized(true);
	// If keep organized was set true, points that failed the test will have their Z value set to this.
	filter.setUserFilterValue(0.0);

	filter.filter(*filteredCloud);
	pcl::io::savePCDFileASCII("no_remove_nan_conditional_removel.pcd", *filteredCloud);
}
