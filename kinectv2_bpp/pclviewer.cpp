#include "pclviewer.h"
#include "build/ui_pclviewer.h"


#define PASSTHROUGH_FILENAME "../passthrough_value.txt"
#define AXIS_SIZE 0.3

//create new viewer to show packing output
#define USE_PROJECTOR 


PCLViewer::PCLViewer (QWidget *parent) : QMainWindow (parent), ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("3DBPP");

    cv::setUseOptimized(true);
	
	timerId = 0;
	last_select_item_index = 0;
	current_order = 0;
	count_timer = 0;

	is_axis_on = false;

	InitialValueToUi();
	InitialQVTKWindow();
	ConnectFunctionToUi();

    filter = new CloudFilter();
    segment = new Segmentation();
	transform = new CloudTransformation();
    plane_transform = new CloudTransformation();
	fileio = new ReadWriteFile();
	binpack = new BinpackingCalculate();
	misc = new Miscellaneous();

	has_plane_param = false;

	boxes_w = new int;
	boxes_h = new int;
	boxes_d = new int;

	boxes_x_pos = new int;
	boxes_y_pos = new int;
	boxes_z_pos = new int;
	boxes_bin_num = new int;

	neworder = new int;
   
	cout << "press connect to start" << endl;


#ifdef USE_PROJECTOR 
	ReadTextFileToList(string("../pcd_test/pcd_test_all.txt"));
/*	ReadPcdFileToList("../pcd_test/p01.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p02.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p03.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p04.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p05.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p06.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p07.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p08.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p09.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p10.pcd", -1, -1, -1);
	ReadPcdFileToList("../pcd_test/p11.pcd", -1, -1, -1);
*/
	//ButtonBinPackingPressed(); //vtk error if do it at this line
#endif

}

void PCLViewer::InitialValueToUi()
{
	//read passthrough value from file
	//which is defined value of last run
	fileio->ReadPassthroughFilterValueFile(
		PASSTHROUGH_FILENAME,
		passthrough_xmin, passthrough_xmax,
		passthrough_ymin, passthrough_ymax, 
		passthrough_zmin, passthrough_zmax);
	//pass by reference, to get return of value from readfile function
	
	//set up initial value in ui
	ui->in_passthrough_xmax->setText(QString::number(passthrough_xmax));
	ui->in_passthrough_xmin->setText(QString::number(passthrough_xmin));
	ui->in_passthrough_ymax->setText(QString::number(passthrough_ymax));
	ui->in_passthrough_ymin->setText(QString::number(passthrough_ymin));
	ui->in_passthrough_zmax->setText(QString::number(passthrough_zmax));
	ui->in_passthrough_zmin->setText(QString::number(passthrough_zmin));

	//SetBinWidth(30);
	//SetBinHeight(40);
	//SetBinDepth(20);

	ui->in_clusterextract_min->setText(QString::number(10));
	ui->in_clusterextract_max->setText(QString::number(100));
}
void PCLViewer::InitialQVTKWindow()
{
	// Set up the QVTK window
	viewer.reset(new pcl::visualization::PCLVisualizer("input viewer", false));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(AXIS_SIZE);
	viewer->initCameraParameters();

	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());


	viewer_bpp.reset(new pcl::visualization::PCLVisualizer("3D viewer_bpp"));
	viewer_bpp->setBackgroundColor(0, 0, 0);
	viewer_bpp->setPosition(800, 0);
	viewer_bpp->addCoordinateSystem(AXIS_SIZE);
	viewer_bpp->initCameraParameters();
	viewer_bpp->addCube(0, 2, 0, 2, 0, 2, 0.0, 0.0, 1.0, "test");
	viewer_bpp->spinOnce(100);

}

void PCLViewer::ConnectFunctionToUi()
{
	//hilight item when clicked
	connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(PressedTreeItem(QTreeWidgetItem *)));

	//menuKinect
	connect(ui->actionConnect, SIGNAL(triggered()), this, SLOT(ButtonConnectPressed()));
	connect(ui->actionDisconnect, SIGNAL(triggered()), this, SLOT(ButtonDisconnectPressed()));

	//menuPlane_Detection
	connect(ui->actionRealtime_detection, SIGNAL(triggered()), this, SLOT(ButtonPlaneSegmentPressed()));
	connect(ui->actionApply_segmentation, SIGNAL(triggered()), this, SLOT(ButtonApplyPlanePressed()));
	connect(ui->actionLoad_Apply_plane, SIGNAL(triggered()), this, SLOT(ButtonLoadPlanePressed()));
	connect(ui->actionSave_plane_parameter, SIGNAL(triggered()), this, SLOT(ButtonSavePlanePressed()));

	//menuCluster_Extraction
	connect(ui->actionColoring_cluster, SIGNAL(triggered()), this, SLOT(ButtonClusterExtractPressed()));
	connect(ui->actionCapture_cluster, SIGNAL(triggered()), this, SLOT(ButtonCaptureCloudPressed()));

	//menuBin_Packing
	connect(ui->actionScan_bin, SIGNAL(triggered()), this, SLOT(ButtonBinScanPressed()));
	connect(ui->actionCalculation, SIGNAL(triggered()), this, SLOT(ButtonBinPackingPressed()));
	connect(ui->actionSave_result, SIGNAL(triggered()), this, SLOT(ButtonLoadBppPressed()));
	connect(ui->actionLoad_result, SIGNAL(triggered()), this, SLOT(ButtonSaveBppPressed()));
	connect(ui->actionToggle_animation, SIGNAL(triggered()), this, SLOT(ButtonAnimationPressed()));


	//parameter adjustment area
	connect(ui->bt_apply_1, SIGNAL(clicked()), this, SLOT(ButtonApplyPassthrough()));
	connect(ui->bt_apply_2, SIGNAL(clicked()), this, SLOT(ButtonApplyVoxelgrid()));
	connect(ui->bt_apply_3, SIGNAL(clicked()), this, SLOT(ButtonApplyOutlierRemove()));

	//show each item area
	connect(ui->bt_item_load, SIGNAL(clicked()), this, SLOT(ButtonLoadEachItemPressed()));
	connect(ui->bt_item_save, SIGNAL(clicked()), this, SLOT(ButtonSaveEachItemPressed()));
	connect(ui->bt_all_load, SIGNAL(clicked()), this, SLOT(ButtonLoadAllItemPressed()));
	connect(ui->bt_all_save, SIGNAL(clicked()), this, SLOT(ButtonSaveAllItemPressed()));

	connect(ui->bt_item_setcenter, SIGNAL(clicked()), this, SLOT(ButtonSetCenterItemPressed()));
	//connect(ui->bt_item_showcloud, SIGNAL(clicked()), this, SLOT(ButtonShowCloudItemPressed()));
	connect(ui->bt_item_remove, SIGNAL(clicked()), this, SLOT(ButtonRemoveItemPressed()));
	connect(ui->bt_item_clearall, SIGNAL(clicked()), this, SLOT(ButtonClearAllItemPressed()));

	//binpacking area
	connect(ui->bt_bin_reset, SIGNAL(clicked()), this, SLOT(ButtonResetBinPressed()));
	connect(ui->bt_order_previous, SIGNAL(clicked()), this, SLOT(ButtonPreviousOrderPressed()));
	connect(ui->bt_order_next, SIGNAL(clicked()), this, SLOT(ButtonNextOrderPressed()));

	//reserve area
	connect(ui->bt_toggle_axis, SIGNAL(clicked()), this, SLOT(ButtonToggleAxisPressed()));
	//connect(ui->bt_bt2, SIGNAL(clicked()), this, SLOT(Button2Pressed()));
	connect(ui->bt_apply_rot, SIGNAL(clicked()), this, SLOT(ButtonApplyRotationPressed()));
	connect(ui->bt_apply_pos, SIGNAL(clicked()), this, SLOT(ButtonApplyTranslationPressed()));
}

void PCLViewer::ButtonConnectPressed()
{
	//do nothing if camera already connected
	if (timerId != 0) return;

	printf("\nButtonConnectPressed\n");

	viewer->initCameraParameters();

	kinect = new KinectV2Interface();
	int return_val = kinect->openKinect();

	printf("\nkinect open return value=%d\n", return_val);

	//mode is determind to show raw input, or transform input
	show_output_mode = raw_input;

	//get value from ui
	ButtonApplyPassthrough();
	ButtonApplyVoxelgrid();

	// Setup the cloud pointer
	pointcloud.reset(new PointCloudXYZRGB);
	nofilter_pointcloud.reset(new PointCloudXYZRGB);

	StartTimerEvent();
}
void PCLViewer::ButtonDisconnectPressed()
{
	printf("\nButtonDisconnectPressed\n");
	
	//do nothing if camera already disconnected
	if (timerId == 0) return;

	StopTimerEvent();

	ClearViewer();

	cout << "\npress connect to start" << endl;

	delete kinect;

}

void PCLViewer::StartTimerEvent()
{
	StopTimerEvent();//cancel previous timer (if exist)

    //nframes = 0; // init
    timerId = startTimer(100); //call timerEvemt every 100 msec
    time.start(); // start time
	cout << "start new timer id = " << timerId << endl;
}
void PCLViewer::StopTimerEvent()
{
	//timerId will = 0, if no timerEvent set
	cout << "stop timer id = " << timerId << endl;
	if (timerId!=0)
	{
		killTimer(timerId);
	}
	timerId = 0;	
}
void PCLViewer::timerEvent(QTimerEvent *event)
{
    switch(show_output_mode)
    { 

	case raw_input: // connect button pressed, show raw input
		DisplayRawCameraInput(); 
		break;

	case plane_segment:// segment button pressed, show plane segment result
		DisplayPlaneSegmentCameraInput();
		break;

	case above_plane:// apply plane button pressed, show segmented plane at origin, and item on plane
		DisplayTransformedCameraInput();
		break;
	case auto_bpp:
		count_timer++;
		if (count_timer == 10)
		{
			DisplayBinpackingLoop();
			count_timer = 0;
		}
		break;

    }
}
void PCLViewer::ButtonPlaneSegmentPressed()
{
	printf("\nButtonPlaneSegmentPressed\n");

	show_output_mode = plane_segment;
	StartTimerEvent();
}
void PCLViewer::ButtonApplyPlanePressed()
{
	printf("\nButtonApplyPlanePressed\n");
	
	if (!has_plane_param)
	{
		StopTimerEvent();
		CalculatePlaneOrientation();//calculate plane transform parameter (set plane to center)
	}
	show_output_mode = above_plane;
	StartTimerEvent();
}

void PCLViewer::CalculatePlaneSegmentation()
{
	//get plane pointcloud
	plane_threshold = GetPlaneThreshold();
	inliers = segment->planeSegmentation(pointcloud, plane_threshold, 1);//colored  plane
}
void PCLViewer::CalculatePlaneOrientation()
{

	//remove environment (not plane) out from pointcloud
	filter->extractIndices(pointcloud, inliers, false);

	//remove noise
	meank_percentage = GetOutlierMeanK();
	meank_size = pointcloud->size()*meank_percentage / 100;
	std_dev = GetOutlierStdDev();
	filter->outlierRemoval(pointcloud, meank_size, std_dev);//meank, stddev

	//move+rotate pointcloud to 0,0,0
	plane_transform->CalculatePlaneOriginTranformation(pointcloud);
	has_plane_param = true;
	//UpdatePointcloudName(pointcloud, "pointcloud");
}

void PCLViewer::DisplayRawCameraInput()
{
	GetColorAndDepthImage();
	ApplyPassthroughFilterToRawInput(pointcloud); //crop input image area (assign value in ui)
	UpdatePointcloudName(pointcloud, "pointcloud");

	//when camera is capture image from corner.
	//rotate input a bit for easy do a passthrough
	////!!  the rotate control point is 0,0,0 not cloud center,
	////!!  so when rotate it, the cloud's position go to far from center
	////!!  so to rotate cloud befor do passthrough is not gain good result
	//transform->RotateCloudAxis(pointcloud, 90.0, Eigen::Vector3f::UnitY());
}
void PCLViewer::DisplayPlaneSegmentCameraInput()
{
	GetColorAndDepthImage();
	ApplyPassthroughFilterToRawInput(pointcloud);
	ApplyVoxelGridFilter(pointcloud);	//reduce cloud size
	CalculatePlaneSegmentation();
	UpdatePointcloudName(pointcloud, "pointcloud");
}
void PCLViewer::DisplayTransformedCameraInput()
{
	GetColorAndDepthImage();
	ApplyPassthroughFilterToRawInput(pointcloud);
	plane_transform->TranslatePlaneToCenter(pointcloud);//rotate & translate segmented plane to center, from plane orientation parameter
	ApplyPassthroughFilterToPlane(pointcloud);//crop only area on top of plane

	//show area of capture in bounding box
	viewer->removeShape("AABB");
	viewer->addCube(
		plane_transform->GetPlaneMinPoint().x, plane_transform->GetPlaneMaxPoint().x,
		plane_transform->GetPlaneMinPoint().y, plane_transform->GetPlaneMaxPoint().y+0.5,
		plane_transform->GetPlaneMinPoint().z, plane_transform->GetPlaneMaxPoint().z, 
		1.0, 1.0, 0.0, "AABB");

	UpdatePointcloudName(pointcloud, "pointcloud");

}




void PCLViewer::ButtonLoadPlanePressed()
{
	printf("\nButtonLoadPlanePressed\n");

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Open plane parameter"), "../", tr("Text Files (*.txt)"));

	cout << filename.toStdString() << endl;

	if (filename.trimmed().isEmpty()) // no file selected
	{
		return;
	}

	//for reset plane to origin point
	Eigen::Vector3f plane_center_translate;
	double plane_first_x_axis_rotate;
	double plane_second_y_axis_rotate;

	//for crop top area on plane
	PointTypeXYZRGB min_point_plane_AABB;
	PointTypeXYZRGB max_point_plane_AABB;

	fileio->ReadPlaneParameterFile(filename.toStdString(),
		plane_center_translate,
		plane_first_x_axis_rotate,
		plane_second_y_axis_rotate,
		min_point_plane_AABB,
		max_point_plane_AABB);

	//cout << "plane_center_translate is \n" << plane_center_translate << endl;
	//cout << "plane_first_x_axis_rotate is \n" << plane_first_x_axis_rotate << endl;
	//cout << "plane_second_y_axis_rotate is \n" << plane_second_y_axis_rotate << endl;
	//cout << "min_point_plane_AABB is \n" << min_point_plane_AABB << endl;
	//cout << "max_point_plane_AABB is \n" << max_point_plane_AABB << endl;

	plane_transform->SetPlaneCenter(plane_center_translate);
	plane_transform->SetPlaneFirstXRot(plane_first_x_axis_rotate);
	plane_transform->SetPlaneSecondYRot(plane_second_y_axis_rotate);
	plane_transform->SetPlaneMinPoint(min_point_plane_AABB);
	plane_transform->SetPlaneMaxPoint(max_point_plane_AABB);
	has_plane_param = true;
	printf("Finished\n");

	ClearViewer();

	ButtonApplyPlanePressed();

}
void PCLViewer::ButtonSavePlanePressed()
{
	printf("\nButtonSavePlanePressed\n");

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save plane parameter"), "../", tr("Text Files (*.txt)"));

	fileio->WritePlaneParameterFile(filename.toStdString(),
		plane_transform->GetPlaneCenter(),
		plane_transform->GetPlaneFirstXRot(),
		plane_transform->GetPlaneSecondYRot(),
		plane_transform->GetPlaneMinPoint(),
		plane_transform->GetPlaneMaxPoint());

	printf("Finished\n");
}

void PCLViewer::ButtonClusterExtractPressed()
{
	printf("\nButtonClusterExtractPressed\n");

	StopTimerEvent();
	ExtractObjectsFromPlane();
}
void PCLViewer::ButtonCaptureCloudPressed()
{
	printf("\nButtonCaptureCloudPressed\n");


	AddObjectsToList();

	//clear screen
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();

	//show_output_mode = above_plane;
	//StartTimerEvent();

}

void PCLViewer::GetColorAndDepthImage()
{
	//printf("\nGetColorAndDepthImage\n");

	//show color image via opencv window
	colorMat = kinect->get_colorframe(0.3);//scale size of image down
	cv::imshow("kinect", colorMat);

	//get point cloud
	kinect->get_depthframe();
	kinect->mapping_pointcloud(pointcloud);
}
/*
[Connect] Button
initial kinect, get rgb image and depth image.
Realtime show RAW camera input, area of display crop by value of Passthrough filter
when assign new passthrough filter value, 
press apply to remember the new value
*/


/*
[Cluster extract] Button
use input pointcloud from last call of DisplayTransformedCameraInput()
-do crop out some area at border
-do segment plane out
-do cluster extraction for segment noise out
then calc boundingbox and show it
*/
void PCLViewer::ExtractObjectsFromPlane()
{

	//reduce border of top area on plane
	//filter->passThrough(pointcloud, "x", plane_transform->GetPlaneMinPoint().x+0.1, plane_transform->GetPlaneMaxPoint().x-0.1);
	//filter->passThrough(pointcloud, "y", plane_transform->GetPlaneMaxPoint().y, plane_transform->GetPlaneMaxPoint().y+0.5);
	//filter->passThrough(pointcloud, "z", plane_transform->GetPlaneMinPoint().z+0.1, plane_transform->GetPlaneMaxPoint().z-0.1);

	CalculatePlaneSegmentation();// no inliers in case of load plane, so segment again
	filter->extractIndices(pointcloud, inliers, true);//remove plane out from pointcloud
	


	//cluster extraction parameter
	cluster_tolerance = GetClusterExtractTolerance();
	cluster_min_percentage = GetClusterExtractMin();
	cluster_min_size = cluster_min_percentage*pointcloud->points.size() / 100;
	cluster_max_percentage = GetClusterExtractMax();
	cluster_max_size = cluster_max_percentage*pointcloud->points.size() / 100;

	//cluster extract to get only real item (remove noise)
	array_cluster_cloud = segment->ClusterExtractItems(
		pointcloud, cluster_tolerance, 
		cluster_min_size, cluster_max_size);


	
	/*
	//draw bounding box around cluster
	transform->CalculateOBB(pointcloud, min_OBB, max_OBB, pos_OBB, rot_OBB);

	Eigen::Vector3f position(pos_OBB.x, pos_OBB.y, pos_OBB.z);
	Eigen::Quaternionf rotation(rot_OBB);


	viewer->removeShape("obb");
	viewer->addCube(position, rotation,
		max_OBB.x - min_OBB.x,
		max_OBB.y - min_OBB.y, 
		max_OBB.z - min_OBB.z,"obb");

	viewer->removeShape("min_OBB");
	viewer->addSphere(min_OBB, 0.01, 1.0, 0.0, 0.0,"min_OBB");

	viewer->removeShape("max_OBB");
	viewer->addSphere(max_OBB, 0.01, 0.0, 1.0, 0.0, "max_OBB");

	viewer->removeShape("pos_OBB");
	viewer->addSphere(pos_OBB, 0.01, 0.0, 0.0, 1.0, "pos_OBB");

	delete transform;
	cout << "min_OBB is \n" << min_OBB << endl;
	cout << "max_OBB is \n" << max_OBB << endl;
	cout << "pos_OBB is \n" << pos_OBB << endl;
	cout << "rot_OBB is \n" << rot_OBB << endl;
	cout << "position is \n" << position << endl;
*/
	UpdatePointcloudName(pointcloud, "pointcloud");


}

/*
reset center of current display item
add it (in obb) to list
*/
void PCLViewer::AddObjectsToList()
{

	for (int i = 0; i < array_cluster_cloud.size(); i++)
	{
		cout << i << " :" << array_cluster_cloud[i]->points.size() << endl;
		
		array_pointcloud.push_back(array_cluster_cloud[i]);

		int index = array_pointcloud.size() - 1;
		AddNewItemToList(-1, -1, -1, array_pointcloud[index]->points.size());
	}

	//UpdatePointcloudName(pointcloud, "pointcloud-toadd");

	
	
	/*
	for (int i = 0; i < array_pointcloud[index]->points.size(); ++i)
		cout 
		<< " " << array_pointcloud[index]->points[i].x
		<< " " << array_pointcloud[index]->points[i].y
		<< " " << array_pointcloud[index]->points[i].z << endl;
*/

	/*
	//transform->TranslateItemToOrigin(pointcloud);
	for (int i = 0; i < pointcloud->points.size(); i++){
		pointcloud->points[i].x -= pos_OBB.x;
		pointcloud->points[i].y -= pos_OBB.y;
		pointcloud->points[i].z -= pos_OBB.z;
	}



	CloudTransformation *transform = new CloudTransformation();
	transform->CalculateOBB(pointcloud, min_OBB, max_OBB, pos_OBB, rot_OBB);

	Eigen::Vector3f position(pos_OBB.x, pos_OBB.y, pos_OBB.z);
	Eigen::Quaternionf rotation(rot_OBB);

	cout << "min_OBB is \n" << min_OBB << endl;
	cout << "max_OBB is \n" << max_OBB << endl;
	cout << "pos_OBB is \n" << pos_OBB << endl;
	cout << "rot_OBB is \n" << rot_OBB << endl;
	cout << "position is \n" << position << endl;

	viewer->removeShape("obb-toadd");
	viewer->addCube(position, rotation,
		max_OBB.x - min_OBB.x,
		max_OBB.y - min_OBB.y,
		max_OBB.z - min_OBB.z, "obb-toadd");

	delete transform;
*/
	



}

/********************************************** 
  AREA OF [GET TEXT INPUT FROM UI] SECTION
**********************************************/
double PCLViewer::GetDouble(QLineEdit *item)
{
    QString valueinstring = item->text();

	//cout << "string = " << valueinstring.toStdString() << endl;

    return valueinstring.toDouble();
}
int PCLViewer::GetInt(QLineEdit *item)
{
	QString valueinstring = item->text();
	return valueinstring.toInt();
}

void PCLViewer::SetCloudSize1(int size)
{
	ui->a_label_cloudsize_1->setText(QString::number(size));
}
void PCLViewer::SetCloudSize2(int size)
{
	ui->a_label_cloudsize_2->setText(QString::number(size));
}
void PCLViewer::SetCloudSize3(int size)
{
	ui->a_label_cloudsize_3->setText(QString::number(size));
}

double PCLViewer::GetPassthroughXmin()
{
	return GetDouble(ui->in_passthrough_xmin);
}
double PCLViewer::GetPassthroughXmax()
{
	return GetDouble(ui->in_passthrough_xmax);
}
double PCLViewer::GetPassthroughYmin()
{
	return GetDouble(ui->in_passthrough_ymin);
}
double PCLViewer::GetPassthroughYmax()
{
	return GetDouble(ui->in_passthrough_ymax);
}
double PCLViewer::GetPassthroughZmin()
{
	return GetDouble(ui->in_passthrough_zmin);
}
double PCLViewer::GetPassthroughZmax()
{
	return GetDouble(ui->in_passthrough_zmax);
}

double PCLViewer::GetVoxelLeafsize()
{
	return GetDouble(ui->in_voxel_leafsize);
}

double PCLViewer::GetPlaneThreshold()
{
	return GetDouble(ui->in_plane_threshold);
}

int PCLViewer::GetOutlierMeanK()
{
	return GetInt(ui->in_outlier_meank);
}
int PCLViewer::GetOutlierStdDev()
{
	return GetInt(ui->in_outlier_stddev);
}


double PCLViewer::GetClusterExtractMin()
{
	return GetDouble(ui->in_clusterextract_min);
}
double PCLViewer::GetClusterExtractMax()
{
	return GetDouble(ui->in_clusterextract_max);
}
double PCLViewer::GetClusterExtractTolerance()
{
	return GetDouble(ui->in_clusterextract_tolerance);
}

int PCLViewer::GetTotalBoxFromList()
{
	//cout << "GetTotalBoxFromList= " << ui->treeWidget->topLevelItemCount() << endl;
	return ui->treeWidget->topLevelItemCount();
}
void PCLViewer::AddNewItemToList(int _w, int _h, int _d, int _size)
{
	int item_number = GetTotalBoxFromList()+1;
	//cout << "item_number" << item_number << endl;

	

	//CalculateItemSizeUpdateList(item_number);

	QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);
	//total_boxes++;

	item->setText(0, QString::number(item_number));
	item->setText(1, QString::number(_w));
	item->setText(2, QString::number(_h));
	item->setText(3, QString::number(_d));
	item->setText(4, QString::number(_size));

	item->setTextAlignment(0, Qt::AlignHCenter);
	item->setTextAlignment(1, Qt::AlignHCenter);
	item->setTextAlignment(2, Qt::AlignHCenter);
	item->setTextAlignment(3, Qt::AlignHCenter);
	item->setTextAlignment(4, Qt::AlignHCenter);

	item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);


	ui->treeWidget->addTopLevelItem(item);

	int index = item_number - 1;
	if (_w == -1) // no dimension yet
	{
		CalculateItemSizeUpdateList(index);
	}
}


int PCLViewer::GetBinWidth()
{
	return GetInt(ui->in_bin_w);
}
int PCLViewer::GetBinHeight()
{
	return GetInt(ui->in_bin_h);
}
int PCLViewer::GetBinDepth()
{
	return GetInt(ui->in_bin_d);
}

void PCLViewer::SetBinWidth(int size)
{
	ui->in_bin_w->setText(QString::number(size));
}
void PCLViewer::SetBinHeight(int size)
{
	ui->in_bin_h->setText(QString::number(size));
}
void PCLViewer::SetBinDepth(int size)
{
	ui->in_bin_d->setText(QString::number(size));
}

int PCLViewer::GetRotation()
{
	return GetInt(ui->in_rot);
}
double PCLViewer::GetTranslation()
{
	return GetDouble(ui->in_pos);
}


//parameter adjustment area
void PCLViewer::ButtonApplyPassthrough()
{
	printf("\nButtonApplyPassthrough\n");

	passthrough_xmin = GetPassthroughXmin();
	passthrough_xmax = GetPassthroughXmax();
	passthrough_ymin = GetPassthroughYmin();
	passthrough_ymax = GetPassthroughYmax();
	passthrough_zmin = GetPassthroughZmin();
	passthrough_zmax = GetPassthroughZmax();

	//save passthrough adjust value in file
	//use for store value, of next time programe run 
	fileio->WritePassthroughFilterValueFile(
		PASSTHROUGH_FILENAME,
		passthrough_xmin, passthrough_xmax,
		passthrough_ymin, passthrough_ymax,
		passthrough_zmin, passthrough_zmax);
	//pass by value, just sent a value to writefile function

}
void PCLViewer::ButtonApplyVoxelgrid()
{
	printf("\nButtonApplyVoxelgrid\n");
	voxel_leafsize=GetVoxelLeafsize();
}

void PCLViewer::ButtonApplyOutlierRemove()
{
	printf("\nButtonApplyOutlierRemove\n");

	//should filter size down before remove noise
	//otherwise take long time to calculate

	//remove noise
	meank_percentage = GetOutlierMeanK();
	meank_size = pointcloud->size()*meank_percentage / 100;
	std_dev = GetOutlierStdDev();
	filter->outlierRemoval(pointcloud, meank_size, std_dev);//meank, stddev

	UpdatePointcloudName(pointcloud, "pointcloud");
	SetCloudSize3(pointcloud->points.size());

}

//hilight item clicked
void PCLViewer::PressedTreeItem(QTreeWidgetItem *item)//, int col
{
	//cout << "mousePressEvent " << ui->treeWidget->currentIndex().row() << endl;

	QTreeWidgetItem* last_selected_item = ui->treeWidget->topLevelItem(last_select_item_index);
	last_selected_item->setBackgroundColor(0, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(1, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(2, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(3, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(4, QColor(255, 255, 255));

	item->setBackgroundColor(0, QColor(200, 200, 200));
	item->setBackgroundColor(1, QColor(200, 200, 200));
	item->setBackgroundColor(2, QColor(200, 200, 200));
	item->setBackgroundColor(3, QColor(200, 200, 200));
	item->setBackgroundColor(4, QColor(200, 200, 200));
	last_select_item_index = ui->treeWidget->currentIndex().row();

	ButtonShowCloudItemPressed();

}
//show each item area
void PCLViewer::ButtonLoadEachItemPressed()
{
	printf("\nButtonLoadEachItemPressed\n");

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load pcd"), "../pcd_file/", tr("point cloud (*.pcd)"));

	
	if (filename.trimmed().isEmpty()) return;
	
	ReadPcdFileToList(filename.toStdString(),-1,-1,-1);
	

}
void PCLViewer::ButtonLoadAllItemPressed()
{
	printf("\nButtonLoadAllItemPressed\n");

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load txt"), "../pcd_test/", tr("list of pcd (*.txt)"));


	if (filename.trimmed().isEmpty()) return;

	ReadTextFileToList(filename.toStdString());


}
void PCLViewer::ReadTextFileToList(string _filename)
{

	fileio->ReadListofPcdfromTextFile(
		_filename, total_boxes,
		bin_w, bin_h, bin_d,
		array_pcd_filename, boxes_w, boxes_h, boxes_d);
	
	SetBinWidth(bin_w);
	SetBinHeight(bin_h);
	SetBinDepth(bin_d);

	for (int i = 0; i < total_boxes; i++)
	{
		ReadPcdFileToList(array_pcd_filename[i], boxes_w[i], boxes_h[i], boxes_d[i]);
	}

}
void PCLViewer::ReadPcdFileToList(string _filename,int w,int h,int d)
{
	//cout << "_filename " << _filename << endl;
	PointCloudXYZRGB::Ptr readcloud(new PointCloudXYZRGB);

	fileio->ReadPcdFile(_filename,readcloud);

	array_pcd_filename.push_back(_filename);

	array_pointcloud.push_back(readcloud);
	

	int index = array_pointcloud.size() - 1;

	//cout << "index=" << index << endl;
	

	AddNewItemToList(w, h, d, array_pointcloud[index]->points.size());
	
	QTreeWidgetItem* last_selected_item = ui->treeWidget->topLevelItem(last_select_item_index);
	last_selected_item->setBackgroundColor(0, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(1, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(2, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(3, QColor(255, 255, 255));
	last_selected_item->setBackgroundColor(4, QColor(255, 255, 255));

	//QTreeWidgetItem* select_item = ui->treeWidget->topLevelItem(index);
	//PressedTreeItem(select_item);
}



void PCLViewer::ButtonSaveEachItemPressed()
{
	printf("\nButtonSaveEachItemPressed\n");

	cout << "last_select_item_index="<< last_select_item_index << endl;

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save pcd"), "../", tr("point cloud (*.pcd)"));


	fileio->WritePcdFile(filename.toStdString(),
		array_pointcloud[last_select_item_index]);

	printf("Finished\n");

}
void PCLViewer::ButtonSaveAllItemPressed()
{
	printf("\nButtonSaveAllItemPressed\n");

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save txt"), "../", tr("list of pcd (*.txt)"));

	if (filename.trimmed().isEmpty()) return;

	bin_w = GetBinWidth();
	bin_h = GetBinHeight();
	bin_d = GetBinDepth();
	binpack->SetBinSize(bin_w, bin_h, bin_d);

	total_boxes = GetTotalBoxFromList();


	CopyEachItemSizeToArray(total_boxes);

	fileio->WriteListofPcdtoTextFile(
		filename.toStdString(), total_boxes,
		bin_w, bin_h, bin_d,
		array_pcd_filename, boxes_w, boxes_h, boxes_d);

	//delete boxes_w;
	//delete boxes_h;
	//delete boxes_d;


	printf("Finished\n");

}

void PCLViewer::ButtonLoadBppPressed()
{

}
void PCLViewer::ButtonSaveBppPressed()
{

}


void PCLViewer::CalculateItemSizeUpdateList(int index)
{
	pointcloud = array_pointcloud[index];

	PointTypeXYZRGB min_AABB;
	PointTypeXYZRGB max_AABB;
	transform->CalculateAABB(pointcloud, min_AABB, max_AABB);

	QTreeWidgetItem* item = ui->treeWidget->topLevelItem(index);

	int item_width = (max_AABB.x - min_AABB.x) * 100;
	int item_height = (max_AABB.y - min_AABB.y) * 100;
	int item_depth = (max_AABB.z - min_AABB.z) * 100;


	item->setText(1, QString::number(item_width));
	item->setText(2, QString::number(item_height));
	item->setText(3, QString::number(item_depth));


	UpdatePointcloudName(pointcloud, "pointcloud");
}
void PCLViewer::ButtonSetCenterItemPressed()
{
	printf("\nButtonSetCenterItemPressed\n");

	//when user didn't click on any item
	//this mode should not effect
	total_boxes = GetTotalBoxFromList();
	if (total_boxes == 0) return;

	StopTimerEvent();

	pointcloud = array_pointcloud[last_select_item_index];

	cout << pointcloud->points.size() << endl;

	transform->TranslateItemCornerToOrigin(pointcloud);

	//pointcloud = array_pointcloud[last_select_item_index];

	//cout << pointcloud->points.size() << endl;

	//CalculateItemSizeUpdateList(last_select_item_index);
	UpdatePointcloudName(pointcloud, "pointcloud");
	



}
void PCLViewer::ButtonShowCloudItemPressed()
{
	//printf("\nButtonShowCloudItemPressed\n");
	//cout << "last_select_item_index=" << last_select_item_index << endl;

	//when user didn't click on any item
	//this mode should not effect
	total_boxes = GetTotalBoxFromList();
	if (total_boxes == 0) return;

	StopTimerEvent();

	pointcloud = array_pointcloud[last_select_item_index];

	//cout << pointcloud->points.size() << endl;

	//CalculateItemSizeUpdateList(last_select_item_index);
	UpdatePointcloudName(pointcloud, "pointcloud");



}
void PCLViewer::ButtonRemoveItemPressed()
{
	////remove from list////
	QTreeWidgetItem* selected_item = ui->treeWidget->currentItem();
	delete selected_item;

	total_boxes = GetTotalBoxFromList();

	//rerun index number
	for (int i = 0; i < total_boxes; ++i)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		item->setText(0, QString::number(i + 1));

	}
	////////////////////////////////////



	////remove from point cloud array////

	//do code
}
void PCLViewer::ButtonClearAllItemPressed()
{
	////remove from list
	
	total_boxes = GetTotalBoxFromList();

	for (int i = total_boxes - 1; i >= 0; i--)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		delete item;
	}
	total_boxes = 0;
	////////////////////////////////////

	////remove from point cloud array////

	//do code


}


//binpacking area
void PCLViewer::ButtonBinScanPressed()
{
	printf("\nButtonBinPackingPressed \n");
	show_output_mode = above_plane;





}
void PCLViewer::ButtonBinPackingPressed()
{
	printf("\nButtonBinPackingPressed \n");

	total_boxes = GetTotalBoxFromList();
	if (total_boxes == 0)
	{
		cout << "total =0, can't do in packing" << endl;
		return;
	}

	StopTimerEvent();


	//0.set packing type
	binpack->SetPackingType(0);//general?

	//1.set bin size
	bin_w = GetBinWidth();
	bin_h = GetBinHeight();
	bin_d = GetBinDepth();

	binpack->SetBinSize(bin_w, bin_h, bin_d);

	//2.set boxes size
	CopyEachItemSizeToArray(total_boxes);
	binpack->SetBoxesSize(total_boxes, boxes_w, boxes_h, boxes_d);


	//check is all items volumn can fit into box?
	if (!binpack->CheckAllCanFit())
	{
		QMessageBox::information(0, QString("Cannot fit all boxes"), QString("All Boxes volumn is larger than bin"), QMessageBox::Ok);
		return;
	}


	//3.init box position
	for (int i = 0; i < total_boxes; ++i)
	{
		boxes_x_pos[i] = 0;
		boxes_y_pos[i] = 0;
		boxes_z_pos[i] = 0;
		boxes_bin_num[i] = 0;
		neworder[i] = i;
	}

	//4.calc & get result box position
	binpack->SetLimit(0, 0, 1);//nodelimit, iterlimit, timelimit
	
	binpack->CalculateBinpack(neworder, boxes_x_pos, boxes_y_pos, boxes_z_pos, boxes_bin_num);


	//5.move pointcloud to position in bin
	int item_not_fit = 0;
	for (int i = 0; i < total_boxes; i++)
	{
		cout
		<< i << ":"
		<< boxes_w[i] << " " << boxes_h[i] << " " << boxes_d[i] << " "
		<< "bin_num:" << boxes_bin_num[i] << " "
		<< "pos:"
		<< boxes_x_pos[i] << " " << boxes_y_pos[i] << " " << boxes_z_pos[i] << " "
		<< endl;
		
		// ignore cloud that not in bin #1	
		if (boxes_bin_num[i] != 1)
		{
			item_not_fit++;
			continue;
		}

		float item_f_x, item_f_y, item_f_z;

		item_f_x = misc->int_scale_to_float(boxes_x_pos[i]);
		item_f_y = misc->int_scale_to_float(boxes_y_pos[i]);
		item_f_z = misc->int_scale_to_float(boxes_z_pos[i]);

		// translate point cloud to calculated position
		int new_index = neworder[i];
		transform->TranslateItemToXYZ(array_pointcloud[new_index],
			item_f_x, item_f_y, item_f_z);
		
		//copy show cloud to vector of pointcloud
		array_resultbpp_cloud.push_back(array_pointcloud[new_index]);

	}

	if (item_not_fit != 0)
	{
		//cout << " cannot fit another " << item_not_fit << " boxes to bin" << endl;
		QString text = "Another " + QString::number(item_not_fit) + " boxes cannot fit in to bin"; // CORRECT
		QMessageBox::information(0, QString("Cannot fit all boxes"), text, QMessageBox::Ok);
	}


	bin_f_w = misc->int_scale_to_float(bin_w);
	bin_f_h = misc->int_scale_to_float(bin_h);
	bin_f_d = misc->int_scale_to_float(bin_d);


	current_order = array_pointcloud.size();
	cout << "current_order" << current_order << endl;

	ShowPackingByOrder();


}
void PCLViewer::DisplayBinpackingLoop()
{
	
	if (current_order > array_pointcloud.size())
	{
		current_order = 1;
	}

	ShowPackingByOrder();
	
	current_order++;

}

void PCLViewer::CopyEachItemSizeToArray(int total_boxes)
{



	for (int i = 0; i < total_boxes; ++i)
	{

		//reset item center
		pointcloud = array_pointcloud[i];
		//transform->TranslateItemCornerToOrigin(pointcloud);

		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);

		boxes_w[i] = item->text(1).toInt();
		boxes_h[i] = item->text(2).toInt();
		boxes_d[i] = item->text(3).toInt();

	}

	
}


void PCLViewer::ButtonResetBinPressed()
{
	StopTimerEvent();

	viewer->initCameraParameters();
	viewer_bpp->initCameraParameters();
	//SetCameraPosition();
	ClearViewer();
}
void PCLViewer::ButtonAnimationPressed()
{
	show_output_mode = auto_bpp;
	StartTimerEvent();

}
void PCLViewer::ShowPackingByOrder()
{
#ifdef USE_PROJECTOR
	viewer_bpp->removeAllShapes();
	viewer_bpp->removeAllPointClouds();

	viewer_bpp->addCube(
		0, bin_f_w,
		0, bin_f_h,
		0, bin_f_d,
		1.0, 0.0, 0.0, "container");

	for (int i = 0; i < current_order; i++)
	{
		//assign name for keep track draw pointcloud/boundingbox
		string cloud_name = "cloud" + to_string(i);

		//cout << "\n addcloud : " << cloud_name << endl;
		//cout << "size " << array_orderedbpp_cloud[i]->points.size() << endl;

		//draw color point cloud
		viewer_bpp->addPointCloud(array_resultbpp_cloud[i], cloud_name);
		viewer_bpp->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
		
	}
	//
	viewer_bpp->spinOnce(100);
	//ui->qvtkWidget->update();
#endif
}
void PCLViewer::ButtonPreviousOrderPressed()
{
	if (current_order > 0)
	{
		current_order--;
	}
	//cout << "current_order" << current_order << endl;
	ShowPackingByOrder();
}
void PCLViewer::ButtonNextOrderPressed()
{
	if (current_order < array_pointcloud.size())
	{
		current_order++;
	}
	//cout << "current_order" << current_order << endl;
	ShowPackingByOrder();
}


//reserve area
void PCLViewer::ButtonToggleAxisPressed()
{
	printf("\nButtonToggleAxisPressed \n");
	cout << is_axis_on << endl;

	if (is_axis_on)
	{
		viewer->removeCoordinateSystem();
	}
	else
	{

		viewer->addCoordinateSystem(AXIS_SIZE);
	}

	is_axis_on = !is_axis_on;
	ui->qvtkWidget->update();

}
void PCLViewer::ButtonApplyRotationPressed()
{
	if (ui->radio_rot_x->isChecked())
	{
		cout << "radio_rot_x" << endl;
	}

	if (ui->radio_rot_y->isChecked())
	{
		cout << "radio_rot_y" << endl;
	}

	if (ui->radio_rot_z->isChecked())
	{
		cout << "radio_rot_z" << endl;
	}


}
void PCLViewer::ButtonApplyTranslationPressed()
{
	double translate = GetTranslation();
	
	vector<pcl::visualization::Camera> cam;

	viewer_bpp->getCameras(cam);

	if (ui->radio_pos_x->isChecked())
	{
		cout << "radio_pos_x" << endl;
		cam[0].focal[0] += translate;
		cam[0].pos[0] += translate;
	}

	if (ui->radio_pos_y->isChecked())
	{
		cout << "radio_pos_y" << endl;
		cam[0].focal[1] += translate;
		cam[0].pos[1] += translate;
	}

	if (ui->radio_pos_z->isChecked())
	{
		cout << "radio_pos_z" << endl;
		cam[0].focal[2] += translate;
		cam[0].pos[2] += translate;
	}

	//
	viewer_bpp->setCameraPosition(
		cam[0].pos[0], cam[0].pos[1], cam[0].pos[2],
		cam[0].focal[0], cam[0].focal[1], cam[0].focal[2],
		cam[0].view[0], cam[0].view[1], cam[0].view[2]
		);
	
	//viewer_bpp->updateCamera();
	

}

/*************************************************
  END OF [BUTTON PRESSED] SECTION
*************************************************/



void PCLViewer::SetCameraPosition()
{
	//at lab
/*	viewer->setCameraPosition(
		0, 0, -0.4, //position
		0, 0, 1, //focalpoint
		0, 1, 0);//viewup
*/
	//at home
	viewer->setCameraPosition(
		0, 0, -0.6, //position   //zoom out a bit
		0, 0, 1, //focalpoint
		0, 1, 0);//viewup

    /*viewer->setCameraPosition( -0.0690809, 0.566668, -0.542421, //position
                                0.00377059, -0.536094, 0.741419, //focalpoint
                                0.0225769, 0.759016, 0.65068);//viewup


    viewer2->setCameraPosition( -0.0164582, 0.435901, -0.62491, //position
                                0.0256269, -0.392997, 0.851841, //focalpoint
                                0.0111341, 0.872104, 0.489194);//viewup
								*/


	ui->qvtkWidget->update();
}
void PCLViewer::ApplyPassthroughFilterToRawInput(PointCloudXYZRGB::Ptr pt)
{
	//printf("\nApplyPassthroughFilterToRawInput\n");

	filter->passThrough(pt, "x", passthrough_xmin, passthrough_xmax);
	filter->passThrough(pt, "y", passthrough_ymin, passthrough_ymax);
	filter->passThrough(pt, "z", passthrough_zmin, passthrough_zmax);
	SetCloudSize2(pt->points.size());
}
void PCLViewer::ApplyPassthroughFilterToPlane(PointCloudXYZRGB::Ptr pt)
{
	//printf("\nApplyPassthroughFilterToRawPlane\n");
	//crop, to show plane + top 0.5y area on plane (=remove plane by y axis position?)
	filter->passThrough(pointcloud, "x", plane_transform->GetPlaneMinPoint().x, plane_transform->GetPlaneMaxPoint().x);
	filter->passThrough(pointcloud, "y", plane_transform->GetPlaneMinPoint().y, plane_transform->GetPlaneMaxPoint().y + 0.5);
	filter->passThrough(pointcloud, "z", plane_transform->GetPlaneMinPoint().z, plane_transform->GetPlaneMaxPoint().z);
	SetCloudSize2(pt->points.size());
}
void PCLViewer::ApplyVoxelGridFilter(PointCloudXYZRGB::Ptr pt)
{
	filter->voxelGrid(pointcloud, voxel_leafsize);
	SetCloudSize3(pointcloud->points.size());
}

void PCLViewer::ClearViewer()
{
	printf("\nClearViewer\n");

	viewer->removeAllShapes();
	viewer->removeAllPointClouds();
	//viewer->removePointCloud("pointcloud");

	ui->qvtkWidget->update();
	SetCloudSize1(0);
	//viewer->initCameraParameters();

	//viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	viewer_bpp->removeAllShapes();
	viewer_bpp->removeAllPointClouds();
	viewer_bpp->spinOnce(100);
}

void PCLViewer::UpdatePointcloudName(PointCloudXYZRGB::Ptr pt, string cloudname)
{
	SetCloudSize1(pt->points.size());
	if (!viewer->updatePointCloud(pt, cloudname))
	{
		viewer->addPointCloud(pt, cloudname);
	}
	ui->qvtkWidget->update();
}



void PCLViewer::TestCallPCLViewer()
{
	cout << "I'm here PCLViewer" << endl;
}


PCLViewer::~PCLViewer ()
{
  delete ui;
}
