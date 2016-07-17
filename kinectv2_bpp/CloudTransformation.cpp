#include "CloudTransformation.h"


CloudTransformation::CloudTransformation()
{

}

/*
when user press [Plane Segment] Button
input cloud = plane only cloud
*/
void CloudTransformation::CalculatePlaneOriginTranformation(
	PointCloudXYZRGB::Ptr cloud)
{
	Eigen::Vector3f mass_center;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;

	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	cout << "\n first calc" << endl;
	cout << "mass_center is \n" << mass_center << endl;
	cout << "major_vector is \n" << major_vector << endl;
	cout << "middle_vector is \n" << middle_vector << endl;
	cout << "minor_vector is \n" << minor_vector << endl;

	plane_center_translate = mass_center;

	//move cloud to origin
	for( int i=0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x-=mass_center[0];
		cloud->points[i].y-=mass_center[1];
		cloud->points[i].z-=mass_center[2];
	}

	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	cout << "\n after move to center" << endl;
	cout << "mass_center is \n" << mass_center << endl;
	cout << "major_vector is \n" << major_vector << endl;
	cout << "middle_vector is \n" << middle_vector << endl;
	cout << "minor_vector is \n" << minor_vector << endl;

	//calculate angle to make plane lie in xz plane
	double rotate_to_horizontal;
	rotate_to_horizontal = CalculateAngleBetween(
		0.0, 1.0, 0.0, //y axis
		minor_vector(0), minor_vector(1), minor_vector(2));
	cout << "rotate_to_horizontal " << rotate_to_horizontal << endl;

	plane_first_x_axis_rotate = rotate_to_horizontal;
	//rotate plane to xz plane
	RotateCloudAxis(cloud, plane_first_x_axis_rotate, Eigen::Vector3f::UnitX());

	
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	cout << "\n after rotate around UnitX" << endl;
	cout << "mass_center is \n" << mass_center << endl;
	cout << "major_vector is \n" << major_vector << endl;
	cout << "middle_vector is \n" << middle_vector << endl;
	cout << "minor_vector is \n" << minor_vector << endl;

	//calculate angle to make plane lie in front direction
	double rotate_to_vertical;
	rotate_to_vertical = CalculateAngleBetween(
		0.0, 0.0, 1.0, //z axis
		middle_vector(0), middle_vector(1), middle_vector(2));
	cout << "rotate_to_vertical " << rotate_to_vertical << endl;

	plane_second_y_axis_rotate = -1 * rotate_to_vertical;
	//rotate plane to lie in front direction
	RotateCloudAxis(cloud, plane_second_y_axis_rotate, Eigen::Vector3f::UnitY());

	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getAABB(min_point_plane_AABB, max_point_plane_AABB); 
	

	
	cout << "\n after rotate around UnitY" << endl;
	cout << "mass_center is \n" << mass_center << endl;
	cout << "major_vector is \n" << major_vector << endl;
	cout << "middle_vector is \n" << middle_vector << endl;
	cout << "minor_vector is \n" << minor_vector << endl;
	cout << "min_point_plane_AABB is \n" << min_point_plane_AABB << endl;
	cout << "max_point_plane_AABB is \n" << max_point_plane_AABB << endl;


}
void CloudTransformation::TranslatePlaneToCenter(PointCloudXYZRGB::Ptr cloud)
{
	//move cloud to origin
	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x -= plane_center_translate[0];
		cloud->points[i].y -= plane_center_translate[1];
		cloud->points[i].z -= plane_center_translate[2];
	}

	//rotate plane to xz plane
	RotateCloudAxis(cloud, plane_first_x_axis_rotate, Eigen::Vector3f::UnitX());

	//rotate plane to lie in front direction
	RotateCloudAxis(cloud, plane_second_y_axis_rotate, Eigen::Vector3f::UnitY());


}


void CloudTransformation::TranslateItemCornerToOrigin(
	PointCloudXYZRGB::Ptr cloud)
{
	//Parent::TestCallPCLViewer();

	Eigen::Vector3f mass_center;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	PointTypeXYZRGB min_point_AABB;
	PointTypeXYZRGB max_point_AABB;


	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);

	cout << "first min_point_AABB = " << min_point_AABB << endl;

	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x -= min_point_AABB.x;
		cloud->points[i].y -= min_point_AABB.y;
		cloud->points[i].z -= min_point_AABB.z;
	}

	//compare vector with x axis
	//but how to know which major minor middle vector will use

	//see which vector close t y axiis so that i y direction
	//another remain 2 vector take one, comepare with x and z
	//then rorate inthe axis that closest

	//compare which vector is at y axis

	cout << "major_vector " << major_vector << endl;
	cout << "middle_vector " << middle_vector << endl;
	cout << "minor_vector " << minor_vector << endl;
	cout << " " << endl;

	double major_angle,middle_angle,minor_angle;
	major_angle = CalculateAngleBetween(
		0.0, 1.0, 0.0, //y axis
		major_vector(0), major_vector(1), major_vector(2));

	middle_angle = CalculateAngleBetween(
		0.0, 1.0, 0.0, //y axis
		middle_vector(0), middle_vector(1), middle_vector(2));

	minor_angle = CalculateAngleBetween(
		0.0, 1.0, 0.0, //y axis
		minor_vector(0), minor_vector(1), minor_vector(2));

	cout << "major_angle " << major_angle << endl;
	cout << "middle_angle " << middle_angle << endl;
	cout << "minor_angle " << minor_angle << endl;
	cout << " " << endl;

	double rotate_angle;
	
	//find the angle that paralel to y axis
	//find the most small angle
	if (major_angle < middle_angle)
	{
		if (major_angle < minor_angle) //major is smallest
		{
			cout << "y axis align vector is * major_angle" << endl;
			double major_angle, middle_angle, minor_angle;
			//compare with x axis
			middle_angle = CalculateAngleBetween(
				1.0, 0.0, 0.0, //x axis
				middle_vector(0), middle_vector(1), middle_vector(2));

			minor_angle = CalculateAngleBetween(
				1.0, 0.0, 0.0, //x axis
				minor_vector(0), minor_vector(1), minor_vector(2));
			cout << "middle_angle " << middle_angle << endl;
			cout << "minor_angle " << minor_angle << endl;

			//rotate the smallest to x axis
			if (middle_angle < minor_angle)
			{
				cout << "middle_angle is near x axis" << endl;
				
				//compare with y axis vector
				if (middle_vector(1)<0.0) rotate_angle = middle_angle;
				else rotate_angle = -1 * middle_angle;				
				cout << "rotate_angle = " << rotate_angle << endl;					
				RotateCloudAxis(cloud, rotate_angle, Eigen::Vector3f::UnitY());
				
			}
			else
			{
				cout << "minor_angle is near x axis" << endl;
				
				//compare with y axis vector
				if (minor_vector(1)<0.0) rotate_angle = minor_angle;
				else rotate_angle = -1 * minor_angle;
				cout << "rotate_angle = " << rotate_angle << endl;				
				RotateCloudAxis(cloud, rotate_angle, Eigen::Vector3f::UnitY());

			}

		}

	}
	
	if (middle_angle < major_angle)
	{
		if (middle_angle < minor_angle) //middle is smallest
		{
			cout << "y axis align vector is * middle_angle" << endl;
			double major_angle, middle_angle, minor_angle;
			//compare with x axis
			major_angle = CalculateAngleBetween(
				1.0, 0.0, 0.0, //x axis
				major_vector(0), major_vector(1), major_vector(2));

			minor_angle = CalculateAngleBetween(
				1.0, 0.0, 0.0, //x axis
				minor_vector(0), minor_vector(1), minor_vector(2));
			cout << "major_angle " << major_angle << endl;
			cout << "minor_angle " << minor_angle << endl;

			//rotate the smallest to x axis
			if (major_angle < minor_angle)
			{
				cout << "major_angle is near x axis" << endl;
				//compare with y axis vector
				if (major_vector(1)<0.0) rotate_angle = major_angle;
				else rotate_angle = -1 * major_angle;
				cout << "rotate_angle = " << rotate_angle << endl;
				RotateCloudAxis(cloud, rotate_angle, Eigen::Vector3f::UnitY());
			}
			else
			{
				cout << "minor_angle is near x axis" << endl;
				//compare with y axis vector
				if (minor_vector(1)<0.0) rotate_angle = minor_angle;
				else rotate_angle = -1 * minor_angle;
				cout << "rotate_angle = " << rotate_angle << endl;
				RotateCloudAxis(cloud, rotate_angle, Eigen::Vector3f::UnitY());


			}

		}

	}

	if (minor_angle < major_angle)
	{
		if (minor_angle < middle_angle) //middle is smallest
		{
			cout << "y axis align vector is * minor_angle" << endl;
			double major_angle, middle_angle, minor_angle;
			//compare with x axis
			major_angle = CalculateAngleBetween(
				1.0, 0.0, 0.0, //x axis
				major_vector(0), major_vector(1), major_vector(2));

			middle_angle = CalculateAngleBetween(
				1.0, 0.0, 0.0, //x axis
				middle_vector(0), middle_vector(1), middle_vector(2));

			cout << "major_angle " << major_angle << endl;
			cout << "middle_angle " << middle_angle << endl;

			//rotate the smallest to x axis
			if (major_angle < middle_angle)
			{
				cout << "major_angle is near x axis" << endl;
				//compare with y axis vector
				if (major_vector(1)<0.0) rotate_angle = major_angle;
				else rotate_angle = -1 * major_angle;
				cout << "rotate_angle = " << rotate_angle << endl;
				RotateCloudAxis(cloud, rotate_angle, Eigen::Vector3f::UnitY());

			}
			else
			{
				cout << "middle_angle is near x axis" << endl;
				//compare with y axis vector
				if (middle_vector(1)<0.0) rotate_angle = middle_angle;
				else rotate_angle = -1 * middle_angle;
				cout << "rotate_angle = " << rotate_angle << endl;
				RotateCloudAxis(cloud, rotate_angle, Eigen::Vector3f::UnitY());

			}
		}

	}


	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);

	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x -= min_point_AABB.x;
		cloud->points[i].y -= min_point_AABB.y;
		cloud->points[i].z -= min_point_AABB.z;
	}
	cout << "second min_point_AABB = " << min_point_AABB << endl;
	cout << "finish TranslateItemCornerToOrigin" << endl;


}


Eigen::Vector3f CloudTransformation::GetPlaneCenter()
{
	return plane_center_translate;
}
double CloudTransformation::GetPlaneFirstXRot()
{
	return plane_first_x_axis_rotate;
}
double CloudTransformation::GetPlaneSecondYRot()
{
	return plane_second_y_axis_rotate;
}
PointTypeXYZRGB CloudTransformation::GetPlaneMinPoint()
{
	return min_point_plane_AABB;
}
PointTypeXYZRGB CloudTransformation::GetPlaneMaxPoint()
{
	return max_point_plane_AABB;
}

void CloudTransformation::SetPlaneCenter(Eigen::Vector3f val)
{
	plane_center_translate = val;
}
void CloudTransformation::SetPlaneFirstXRot(double val)
{
	plane_first_x_axis_rotate = val;
}
void CloudTransformation::SetPlaneSecondYRot(double val)
{
	plane_second_y_axis_rotate = val;
}
void CloudTransformation::SetPlaneMinPoint(PointTypeXYZRGB val)
{
	min_point_plane_AABB = val;
}
void CloudTransformation::SetPlaneMaxPoint(PointTypeXYZRGB val)
{
	max_point_plane_AABB = val;
}





double CloudTransformation::CalculateAngleBetween(
	double coef_a1, double coef_a2, double coef_a3,
	double coef_b1, double coef_b2, double coef_b3)
{
	/* 
	 angle between 2 planes is EQUAL 
	 to the acute angle of normal vector of 2 planes

	 equation : n1 dot n2 = length(n1)*length(n2)*cos(angle)

	*/

	double dA[3] = { coef_a1, coef_a2, coef_a3 };
	double dB[3] = { coef_b1, coef_b2, coef_b3 };

    double dotproduct=dA[0]*dB[0]+dA[1]*dB[1]+dA[2]*dB[2];

    double lenghta=sqrt(dA[0]*dA[0]+dA[1]*dA[1]+dA[2]*dA[2]);

    double lenghtb=sqrt(dB[0]*dB[0]+dB[1]*dB[1]+dB[2]*dB[2]);

    double costheta = dotproduct/(lenghta*lenghtb);

    double angle = acos(costheta) * 180 / M_PI;

    //cout << "dot product =  " << dotproduct << endl;
    //cout << "lenghta =  " << lenghta << endl;
    //cout << "lenghtb =  " << lenghtb << endl;
    //cout << "costheta =  " << costheta << endl;
    //cout << "angle =  " << angle << endl;

    while(angle>90.0)
    {
        angle = 180.0-angle;
    }

    return angle;
    // inverse normol direction face to xz plane may be get a  true value
    // how do we know axis to rotate from 1 angle?


}

void CloudTransformation::RotateCloudAxis(PointCloudXYZRGB::Ptr cloud, double angle, Eigen::Vector3f axis)
{
    PointCloudXYZRGB::Ptr rotatedCloud (new PointCloudXYZRGB);

    Eigen::Affine3f transform_angle=Eigen::Affine3f::Identity();
	transform_angle.rotate(Eigen::AngleAxisf((angle)*M_PI / 180, axis));
	pcl::transformPointCloud(*cloud, *rotatedCloud, transform_angle);
    pcl::copyPointCloud( *rotatedCloud, *cloud );

}


void CloudTransformation::CalculateOBB(PointCloudXYZRGB::Ptr cloud,
	PointTypeXYZRGB &min_OBB,
	PointTypeXYZRGB &max_OBB,
	PointTypeXYZRGB &pos_OBB,
	Eigen::Matrix3f &rot_OBB)
{

	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getOBB(min_OBB, max_OBB, pos_OBB, rot_OBB);

}

void CloudTransformation::CalculateAABB(PointCloudXYZRGB::Ptr cloud,
	PointTypeXYZRGB &min_point_AABB,
	PointTypeXYZRGB &max_point_AABB)
{
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
}






void CloudTransformation::TranslateItemToXYZ(PointCloudXYZRGB::Ptr cloud, float x, float y, float z)
{

	//if(x==y==z==0.0) return;
	
	for( int i=0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x += x;
		cloud->points[i].y += y;
		cloud->points[i].z += z;
	}
}
void resetPointCloudPosition(PointCloudXYZRGB::Ptr cloud, PointTypeXYZRGB min_point)
{
	/*
	for( int i=0; i < cloud->points.size(); i++){
	cloud->points[i].x-=min_point.x;
	cloud->points[i].y-=min_point.y;
	cloud->points[i].z-=min_point.z;
	}
	*/

	//return pointcloud
	//pcl::copyPointCloud( *cloud_filtered, *cloud );

}


int float_scale_to_int(float number)
{
	return (int)(number * 1000);
}
float int_scale_to_float(int number)
{
	float value = number*0.001;
	return value;
}
