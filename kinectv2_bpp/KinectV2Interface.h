#ifndef KinectV2Interface_H
#define KinectV2Interface_H

#include "AllHeader.h"
#define NOMINMAX


//using namespace cv;

class KinectV2Interface
{
public:
    KinectV2Interface();
    ~KinectV2Interface();
    void initialize_kinect();
    void release_kinect();
    int openKinect();
    IColorFrameReader* get_colorframereader();
    IDepthFrameReader* get_depthframereader();
    ICoordinateMapper* get_coordinatemapper();
    cv::Mat get_colorframe(float COLORSCALE);
    void get_depthframe();
    void mapping_pointcloud(pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud);

	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL){
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

private:
    HRESULT hResult;

    int colorWidth;
    int colorHeight;
    int depthWidth;
    int depthHeight;

    unsigned int colorBufferSize;
    unsigned int depthBufferSize;

    std::vector<RGBQUAD> colorBuffer;
    std::vector<UINT16> depthBuffer;


    IKinectSensor* pSensor;
    IColorFrameSource* pColorSource;
    IFrameDescription* pColorDescription;
    IDepthFrameSource* pDepthSource;
    IFrameDescription* pDepthDescription;

    IColorFrameReader* pColorReader;
    IDepthFrameReader* pDepthReader;
    ICoordinateMapper* pCoordinateMapper;

};

#endif // KinectV2Interface_H
