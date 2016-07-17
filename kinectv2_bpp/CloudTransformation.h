#ifndef CloudTransformation_H
#define CloudTransformation_H

#include "AllHeader.h"


class CloudTransformation
{
public:
    CloudTransformation();
    
	void CalculatePlaneOriginTranformation(
		PointCloudXYZRGB::Ptr cloud);
    void TranslatePlaneToCenter(
		PointCloudXYZRGB::Ptr cloud);

	void TranslateItemCornerToOrigin(
		PointCloudXYZRGB::Ptr cloud);

	void TranslateItemToXYZ(PointCloudXYZRGB::Ptr cloud, float x, float y, float z);

	Eigen::Vector3f GetPlaneCenter();
	double GetPlaneFirstXRot();
	double GetPlaneSecondYRot();
	PointTypeXYZRGB GetPlaneMinPoint();
	PointTypeXYZRGB GetPlaneMaxPoint();

	void SetPlaneCenter(Eigen::Vector3f val);
	void SetPlaneFirstXRot(double val);
	void SetPlaneSecondYRot(double val);
	void SetPlaneMinPoint(PointTypeXYZRGB val);
	void SetPlaneMaxPoint(PointTypeXYZRGB val);

	double CalculateAngleBetween(
		double coef_a1, double coef_a2, double coef_a3,
		double coef_b1, double coef_b2, double coef_b3);

	void RotateCloudAxis(PointCloudXYZRGB::Ptr cloud, double angle, Eigen::Vector3f axis);


	void CalculateOBB(PointCloudXYZRGB::Ptr cloud, 
		PointTypeXYZRGB &min_OBB, 
		PointTypeXYZRGB &max_OBB,
		PointTypeXYZRGB &pos_OBB,
		Eigen::Matrix3f &rot_OBB);

	void CalculateAABB(PointCloudXYZRGB::Ptr cloud,
		PointTypeXYZRGB &min_point_plane_AABB,
		PointTypeXYZRGB &max_point_plane_AABB);



private:
	pcl::MomentOfInertiaEstimation <PointTypeXYZRGB> feature_extractor;

	//for reset plane to origin point
	Eigen::Vector3f plane_center_translate;
	double plane_first_x_axis_rotate;
	double plane_second_y_axis_rotate;

	//for crop top area on plane
    PointTypeXYZRGB min_point_plane_AABB;
    PointTypeXYZRGB max_point_plane_AABB;


};

#endif // CloudTransformation_H
