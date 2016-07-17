#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include "AllHeader.h"

#include "KinectV2Interface.h"
#include "cloudfilter.h"
#include "segmentation.h"
#include "CloudTransformation.h"
#include "ReadWriteFile.h"
#include "miscellaneous.h"
#include "BinpackingCalculate.h"


enum output_mode { raw_input, plane_segment, above_plane, auto_bpp };

namespace Ui
{
	class PCLViewer;
}

class PCLViewer : public QMainWindow
{
	Q_OBJECT

public:
	explicit PCLViewer (QWidget *parent = 0);
	~PCLViewer ();
	void UpdatePointcloudName(PointCloudXYZRGB::Ptr pt, string cloudname);
	void TestCallPCLViewer();

public slots:
	//hilight item when clicked
	void PressedTreeItem(QTreeWidgetItem *item);//, int col

	//process mode area
	void ButtonConnectPressed();
	void ButtonDisconnectPressed();
	void ButtonPlaneSegmentPressed();
	void ButtonApplyPlanePressed();
	void ButtonClusterExtractPressed();
	void ButtonCaptureCloudPressed();
	void ButtonBinScanPressed();
  
	//parameter adjustment area
	void ButtonApplyPassthrough();
	void ButtonApplyVoxelgrid();
	void ButtonLoadPlanePressed();
	void ButtonSavePlanePressed();
	void ButtonApplyOutlierRemove();

	//show each item area
	void ButtonLoadEachItemPressed();
	void ButtonSaveEachItemPressed();
	void ButtonLoadAllItemPressed();
	void ButtonSaveAllItemPressed();
	void ButtonSetCenterItemPressed();
	void ButtonShowCloudItemPressed();
	void ButtonRemoveItemPressed();
	void ButtonClearAllItemPressed();

	//binpacking area
	void ButtonBinPackingPressed();
	void ButtonLoadBppPressed();
	void ButtonSaveBppPressed();
	void ButtonResetBinPressed();
	void ButtonAnimationPressed();
	void ButtonPreviousOrderPressed();
	void ButtonNextOrderPressed();

	//reserve area
	void ButtonToggleAxisPressed();
	void ButtonApplyRotationPressed();
	void ButtonApplyTranslationPressed();



private:
	Ui::PCLViewer *ui;
	cv::Mat colorMat;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_bpp;

	PointCloudXYZRGB::Ptr pointcloud;
	PointCloudXYZRGB::Ptr nofilter_pointcloud;
	vector<PointCloudXYZRGB::Ptr> array_pointcloud;//input of packing
	vector<PointCloudXYZRGB::Ptr> array_cluster_cloud;
	vector<PointCloudXYZRGB::Ptr> array_resultbpp_cloud;
	vector<PointCloudXYZRGB::Ptr> array_orderedbpp_cloud;
	vector<string> array_pcd_filename;

	int timerId;
	int nframes; // used to calculate actual frame rate
	QTime time; // used to calculate actual frame rate	
	output_mode show_output_mode;

	int last_select_item_index;
	int current_order;

	int count_timer;

	bool is_axis_on;

	KinectV2Interface *kinect;
	CloudFilter *filter;
	Segmentation *segment;
	CloudTransformation *plane_transform;
	CloudTransformation *transform;
	ReadWriteFile *fileio;
	Miscellaneous *misc;
	BinpackingCalculate *binpack;

	QString cloudsize;
	pcl::PointIndices::Ptr inliers;

	bool has_plane_param;
	int total_boxes;


	//array of struct
	//struct(point cloud, bounding box)
	PointTypeXYZRGB min_OBB;
	PointTypeXYZRGB max_OBB;
	PointTypeXYZRGB pos_OBB;
	Eigen::Matrix3f rot_OBB;

	double passthrough_xmin, passthrough_xmax;
	double passthrough_ymin, passthrough_ymax;
	double passthrough_zmin, passthrough_zmax;
	double voxel_leafsize;
	double plane_threshold;
	int meank_percentage, meank_size;
	int std_dev;

	double cluster_tolerance;
	int cluster_min_percentage, cluster_min_size;
	int cluster_max_percentage, cluster_max_size;


	// for bin packing
	int *boxes_w, *boxes_h, *boxes_d;
	int *boxes_x_pos, *boxes_y_pos, *boxes_z_pos;

	int *boxes_bin_num;

	int bin_w, bin_h, bin_d;
	float bin_f_w;// = misc->int_scale_to_float(binW);
	float bin_f_h;// = misc->int_scale_to_float(binH);
	float bin_f_d;// = misc->int_scale_to_float(binD);

	int *neworder;

	///////////////////////////////////////////////

	void InitialValueToUi();
	void InitialQVTKWindow();
	void ConnectFunctionToUi();
	void StartTimerEvent();
	void StopTimerEvent();
	void timerEvent(QTimerEvent *event);

	void GetColorAndDepthImage();
	void DisplayRawCameraInput();
	void DisplayPlaneSegmentCameraInput();
	void DisplayTransformedCameraInput();
	void ExtractObjectsFromPlane();
	void AddObjectsToList();

	void ApplyPassthroughFilterToRawInput(PointCloudXYZRGB::Ptr pt);
	void ApplyPassthroughFilterToPlane(PointCloudXYZRGB::Ptr pt);
	void ApplyVoxelGridFilter(PointCloudXYZRGB::Ptr pt);
	void SetCameraPosition();
	void ClearViewer();

	void CalculatePlaneSegmentation();
	void CalculatePlaneOrientation();

	void CalculateItemSizeUpdateList(int index);

	void CopyEachItemSizeToArray(int total_boxes);

	void ShowPackingByOrder();
	void DisplayBinpackingLoop();
	
	int GetTotalBoxFromList();

	void AddNewItemToList(int _w, int _h, int _d, int _size);
	void ReadPcdFileToList(string filename, int w, int h, int d);
	void ReadTextFileToList(string filename);
	/*
	GET TEXT INPUT FROM UI
	*/
	double GetDouble(QLineEdit *item);
	int GetInt(QLineEdit *item);

	void SetCloudSize1(int size);
	void SetCloudSize2(int size);
	void SetCloudSize3(int size);

	double GetPassthroughXmin();
	double GetPassthroughXmax();
	double GetPassthroughYmin();
	double GetPassthroughYmax();
	double GetPassthroughZmin();
	double GetPassthroughZmax();

	double GetVoxelLeafsize();
	double GetPlaneThreshold();

	int GetOutlierMeanK();
	int GetOutlierStdDev();

	double GetClusterExtractMin();
	double GetClusterExtractMax();
	double GetClusterExtractTolerance();
	
	int GetBinWidth();
	int GetBinHeight();
	int GetBinDepth();

	void SetBinWidth(int size);
	void SetBinHeight(int size);
	void SetBinDepth(int size);

	int GetRotation();
	double GetTransltion();

};

#endif // PCLVIEWER_H
