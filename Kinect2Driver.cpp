#include "Driver\OniDriverAPI.h"
#include "XnLib.h"
#include "XnHash.h"
#include "XnEvent.h"

#include <atlbase.h>
#include <Kinect.h>
#include <vector>
#include <iostream>

//#define _COLOR_DEPTH_SAME_DIM 1

static const int        COLOR_WIDTH = 1920;
static const int        COLOR_HEIGHT = 1080;
#ifdef _COLOR_DEPTH_SAME_DIM
static const int        DEPTH_OUTPUT_WIDTH = 1920;
static const int        DEPTH_OUTPUT_HEIGHT = 1080;
#else
static const int        DEPTH_OUTPUT_WIDTH = 512;
static const int        DEPTH_OUTPUT_HEIGHT = 424;
#endif

static const int        DEPTH_NATIVE_WIDTH = 512;
static const int        DEPTH_NATIVE_HEIGHT = 424;

typedef struct
{
	int refCount;
} KinectV2StreamFrameCookie;

class KinectV2Stream : public oni::driver::StreamBase
{
public:
	~KinectV2Stream()
	{
		stop();
	}

	OniStatus start()
	{
		xnOSCreateThread(threadFunc, this, &m_threadHandle);

		return ONI_STATUS_OK;
	}

	void stop()
	{
		m_running = false;
		xnOSWaitForThreadExit(m_threadHandle, 1000);
	}

	virtual OniStatus SetVideoMode(OniVideoMode*) = 0;
	virtual OniStatus GetVideoMode(OniVideoMode* pVideoMode) = 0;

	OniStatus getProperty(int propertyId, void* data, int* pDataSize)
	{
		if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
		{
			if (*pDataSize != sizeof(OniVideoMode))
			{
				printf("Unexpected size: %d != %d\n", *pDataSize, (int)sizeof(OniVideoMode));
				return ONI_STATUS_ERROR;
			}
			return GetVideoMode((OniVideoMode*)data);
		}

		return ONI_STATUS_NOT_IMPLEMENTED;
	}

	OniStatus setProperty(int propertyId, const void* data, int dataSize)
	{
		if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
		{
			if (dataSize != sizeof(OniVideoMode))
			{
				printf("Unexpected size: %d != %d\n", dataSize, (int)sizeof(OniVideoMode));
				return ONI_STATUS_ERROR;
			}
			return SetVideoMode((OniVideoMode*)data);
		}

		return ONI_STATUS_NOT_IMPLEMENTED;
	}

	//OniDriverFrame* AcquireFrame()
	//{
	//	OniDriverFrame* pFrame = (OniDriverFrame*)xnOSCalloc(1, sizeof(OniDriverFrame));
	//	if (pFrame == NULL)
	//	{
	//		XN_ASSERT(FALSE);
	//		return NULL;
	//	}

	//	OniVideoMode mode;
	//	GetVideoMode( &mode );

	//	int dataSize = mode.resolutionX * mode.resolutionY * GetBytesPerPixel();
	//	pFrame->frame.data = xnOSMallocAligned(dataSize, XN_DEFAULT_MEM_ALIGN);
	//	if (pFrame->frame.data == NULL)
	//	{
	//		XN_ASSERT(FALSE);
	//		return NULL;
	//	}

	//    xnOSMemSet( pFrame->frame.data, 0,  dataSize );

	//	pFrame->pDriverCookie = xnOSMalloc(sizeof(KinectV2StreamFrameCookie));
	//	((KinectV2StreamFrameCookie*)pFrame->pDriverCookie)->refCount = 1;

	//	pFrame->frame.dataSize = dataSize;
	//	return pFrame;
	//}

	virtual void Mainloop() = 0;
	virtual int GetBytesPerPixel() = 0;

protected:
	// Thread
	static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM pThreadParam)
	{
		KinectV2Stream* pStream = (KinectV2Stream*)pThreadParam;

		if (pStream != NULL)
		{
			pStream->m_running = true;
			pStream->Mainloop();
		}
		XN_THREAD_PROC_RETURN(XN_STATUS_OK);
	}

	volatile bool m_running;

	XN_THREAD_HANDLE m_threadHandle;

};

class KinectV2ColorStream : public KinectV2Stream
{
public:
	KinectV2ColorStream(CComPtr<IKinectSensor>& kinect)
		: KinectV2Stream()
		, m_pIKinectSensor(kinect)
	{
		configureColorNode();

		m_frameId = 1;
	}

	OniStatus SetVideoMode(OniVideoMode *pNewVideoMode) 
	{ 
		if (pNewVideoMode != NULL)
		{
			if (pNewVideoMode->resolutionX != COLOR_WIDTH ||
				pNewVideoMode->resolutionY != COLOR_HEIGHT ||
				pNewVideoMode->fps != 30 ||
				pNewVideoMode->pixelFormat != ONI_PIXEL_FORMAT_RGB888)
			{
				return ONI_STATUS_NOT_IMPLEMENTED;
			}
		}
		else
			return ONI_STATUS_BAD_PARAMETER;
		return ONI_STATUS_OK;
	}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		if (pVideoMode != NULL)
		{
			pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_RGB888;
			pVideoMode->fps = 30;
			pVideoMode->resolutionX = COLOR_WIDTH;
			pVideoMode->resolutionY = COLOR_HEIGHT;
		}
		else
			return ONI_STATUS_BAD_PARAMETER;
		return ONI_STATUS_OK;
	}

	virtual int GetBytesPerPixel() 
	{ 
		return sizeof(OniRGB888Pixel); 
	}

	void Mainloop()
	{
		m_running = true;

		while (m_running)
		{
			OniFrame* pFrame = getServices().acquireFrame();

			BuildFrame(pFrame);

			raiseNewFrame(pFrame);

			getServices().releaseFrame(pFrame);
		}
	}

private:

	void configureColorNode()
	{
		CComPtr<IColorFrameSource> colorFrameSource;

		auto hr = m_pIKinectSensor->Open();
		if (FAILED(hr))
		{
			std::cerr << "IKinectSensor::Open() failed." << std::endl;
		}

		hr = m_pIKinectSensor->get_ColorFrameSource(&colorFrameSource);
		if (FAILED(hr))
		{
			std::cerr << "IKinectSensor::get_ColorFrameSource() failed." << std::endl;
			return;
		}

		hr = colorFrameSource->OpenReader(&m_pIColorFrameReader);
		if (FAILED(hr))
		{
			std::cerr << "IColorFrameSource::OpenReader() failed." << std::endl;
			return;
		}
	}

	virtual int BuildFrame(OniFrame* pFrame)
	{
		if (!m_pIColorFrameReader)
		{
			return 0;
		}
		CComPtr<IColorFrame> pIColorFrame;
		HRESULT hr = m_pIColorFrameReader->AcquireLatestFrame(&pIColorFrame);
		if (FAILED(hr))
		{
			return 0;
		}

		ColorImageFormat rawcolorformat = ColorImageFormat_None;

		hr = pIColorFrame->get_RawColorImageFormat(&rawcolorformat);
		if (FAILED(hr))
		{
			return 0;
		}
		CComPtr<IFrameDescription> pIFrameDescription;

		hr = pIColorFrame->get_FrameDescription(&pIFrameDescription);
		if (FAILED(hr))
		{
			return 0;
		}

		pFrame->frameIndex = m_frameId;

		pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		pFrame->videoMode.resolutionX = COLOR_WIDTH;
		pFrame->videoMode.resolutionY = COLOR_HEIGHT;
		pFrame->videoMode.fps = 30;

		pFrame->width = COLOR_WIDTH;
		pFrame->height = COLOR_HEIGHT;

		OniRGB888Pixel* pixel = (OniRGB888Pixel*)(pFrame->data);

		if (rawcolorformat == ColorImageFormat_Bgra)
		{
			UINT nCapacity = 0;
			BYTE *pOutBuf = NULL;

			hr = pIColorFrame->AccessRawUnderlyingBuffer(&nCapacity, &pOutBuf);
			if (FAILED(hr))
			{
				return 0;
			}

			const int nPixelCount = nCapacity / 4;
			for (int p = 0; p < nPixelCount; p++)
			{
				pixel->b = *pOutBuf; pOutBuf++;
				pixel->g = *pOutBuf; pOutBuf++;
				pixel->r = *pOutBuf; pOutBuf++;
				pOutBuf++;
				pixel++;
			}
		}
		else if (rawcolorformat == ColorImageFormat_Rgba)
		{
			UINT nCapacity = 0;
			BYTE *pOutBuf = NULL;

			hr = pIColorFrame->AccessRawUnderlyingBuffer(&nCapacity, &pOutBuf);
			if (FAILED(hr))
			{
				return 0;
			}

			const int nPixelCount = nCapacity / 4;
			for (int p = 0; p < nPixelCount; p++)
			{
				pixel->r = *pOutBuf; pOutBuf++;
				pixel->g = *pOutBuf; pOutBuf++;
				pixel->b = *pOutBuf; pOutBuf++;
				pOutBuf++;
				pixel++;
			}
		}
		else
		{
			std::vector<RGBQUAD> colorRGBX;

			colorRGBX.resize(COLOR_WIDTH * COLOR_HEIGHT);

			hr = pIColorFrame->CopyConvertedFrameDataToArray((UINT)(colorRGBX.size() * sizeof(RGBQUAD)),
														   reinterpret_cast<BYTE*>(&colorRGBX[0]), ColorImageFormat_Bgra);
			if (FAILED(hr))
			{
				return 0;
			}
			const int nPixelCount = COLOR_WIDTH * COLOR_HEIGHT;
			for (int p = 0; p < nPixelCount; p++)
			{
				pixel->b = colorRGBX[p].rgbBlue;
				pixel->g = colorRGBX[p].rgbGreen;
				pixel->r = colorRGBX[p].rgbRed;
				pixel++;
			}
		}


		pFrame->cropOriginX = pFrame->cropOriginY = 0;
		pFrame->croppingEnabled = FALSE;

		pFrame->sensorType = ONI_SENSOR_COLOR;
		pFrame->stride = COLOR_WIDTH * sizeof(OniRGB888Pixel);
		pFrame->timestamp = m_frameId * 33000;

		++m_frameId;

		return 1;
	}


	int m_frameId;


	CComPtr<IKinectSensor>          m_pIKinectSensor;
	CComPtr<IColorFrameReader>      m_pIColorFrameReader;

};


class KinectV2DepthStream : public KinectV2Stream
{
public:
	KinectV2DepthStream(CComPtr<IKinectSensor>& kinect)
		: KinectV2Stream()
		, m_pIKinectSensor(kinect)
	{
		configureDepthNode();

		m_frameId = 1;
	}

	OniStatus SetVideoMode(OniVideoMode *pNewVideoMode)
	{ 
		if (pNewVideoMode != NULL)
		{
			if (pNewVideoMode->resolutionX != DEPTH_OUTPUT_WIDTH ||
				pNewVideoMode->resolutionY != DEPTH_OUTPUT_HEIGHT ||
				pNewVideoMode->fps != 30 ||
				pNewVideoMode->pixelFormat != ONI_PIXEL_FORMAT_DEPTH_1_MM)
			{
				return ONI_STATUS_NOT_IMPLEMENTED;
			}
		}
		else
			return ONI_STATUS_BAD_PARAMETER;
		return ONI_STATUS_OK;
	}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		if (pVideoMode != NULL)
		{
			pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
			pVideoMode->fps = 30;
			pVideoMode->resolutionX = DEPTH_OUTPUT_WIDTH;
			pVideoMode->resolutionY = DEPTH_OUTPUT_HEIGHT;
		}
		else
			return ONI_STATUS_BAD_PARAMETER;
		return ONI_STATUS_OK;
	}

	virtual int GetBytesPerPixel() 
	{ 
		return sizeof(OniDepthPixel); 
	}

	void Mainloop()
	{
		m_running = true;

		while (m_running)
		{
			OniFrame* pFrame = getServices().acquireFrame();
			BuildFrame(pFrame);
			raiseNewFrame(pFrame);
			getServices().releaseFrame(pFrame);
		}
	}

	virtual OniStatus convertDepthToColorCoordinates(StreamBase* colorStream, int depthX, int depthY, OniDepthPixel depthZ, int* pColorX, int* pColorY) 
	{ 
		CComPtr<ICoordinateMapper> pICoordMapper;
		OniStatus oRes = ONI_STATUS_OK;
		HRESULT hr = m_pIKinectSensor->get_CoordinateMapper(&pICoordMapper);

		if (FAILED(hr))
		{
			std::cerr << "IKinectSensor::get_CoordinateMapper() failed." << std::endl;
			return ONI_STATUS_NOT_SUPPORTED;
		}
		DepthSpacePoint dp = { (float)depthX, (float)depthY };
		ColorSpacePoint cp;

		hr = pICoordMapper->MapDepthPointToColorSpace(dp, depthZ, &cp);
		if (FAILED(hr))
		{
			std::cerr << "IKinectSensor::MapDepthPointToColorSpace() failed." << std::endl;
			return ONI_STATUS_NOT_SUPPORTED;
		}
		if (pColorX)
			*pColorX = (int)cp.X;
		if (pColorY)
			*pColorY = (int)cp.Y;

		return ONI_STATUS_OK; 
	}
private:

	void configureDepthNode()
	{
		CComPtr<IDepthFrameSource> depthFrameSource;

		auto hr = m_pIKinectSensor->get_DepthFrameSource(&depthFrameSource);
		if (FAILED(hr))
		{
			std::cerr << "IKinectSensor::get_DepthFrameSource() failed." << std::endl;
			return;
		}

		hr = depthFrameSource->OpenReader(&m_pIDepthFrameReader);
		if (FAILED(hr))
		{
			std::cerr << "IDepthFrameSource::OpenReader() failed." << std::endl;
			return;
		}
	}


	virtual int BuildFrame(OniFrame* pFrame)
	{
		//update();
		HRESULT hr = S_OK;
		CComPtr<IDepthFrame> depthFrame;
		UINT nNativeBufferSize = 0;
		UINT16 *pNativeBuffer = NULL;

		CComPtr<ICoordinateMapper> ICoordMapper;

		hr = m_pIKinectSensor->get_CoordinateMapper(&ICoordMapper);
		if (FAILED(hr))
		{
			std::cerr << "IKinectSensor::get_DepthFrameSource() failed." << std::endl;
			return 0;
		}

		pFrame->frameIndex = m_frameId;

		pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		pFrame->videoMode.resolutionX = DEPTH_OUTPUT_WIDTH;
		pFrame->videoMode.resolutionY = DEPTH_OUTPUT_HEIGHT;
		pFrame->videoMode.fps = 30;

		pFrame->width = DEPTH_OUTPUT_WIDTH;
		pFrame->height = DEPTH_OUTPUT_HEIGHT;

		if (!m_pIDepthFrameReader)
		{
			return 0;
		}


		hr = m_pIDepthFrameReader->AcquireLatestFrame(&depthFrame);
		if (FAILED(hr))
		{
			return 0;
		}

		// here, KinectV2 native depth size is different from color, 
		// it needs to be mapped to use.
		// for now, we force output depth as the same dimension as color,
		// then map native depth to this new large depth image.
		hr = depthFrame->AccessUnderlyingBuffer(&nNativeBufferSize, &pNativeBuffer);
		if (FAILED(hr))
		{
			return 0;
		}
		pFrame->cropOriginX = pFrame->cropOriginY = 0;
		pFrame->croppingEnabled = FALSE;

		pFrame->sensorType = ONI_SENSOR_DEPTH;
		pFrame->stride = DEPTH_OUTPUT_WIDTH * GetBytesPerPixel();
		pFrame->timestamp = m_frameId * 33000;

#ifdef _COLOR_DEPTH_SAME_DIM
		m_pOutputDepthBuffer.resize(DEPTH_OUTPUT_WIDTH * DEPTH_OUTPUT_HEIGHT);
		/*
		auto count = bufferSize_ * GetBytesPerPixel();

		if ( pFrame->data != 0 && depthBuffer_.size() != 0 && pFrame->dataSize == count ) {
		fprintf( stderr, "update()%p, %p, %d, %d\n", pFrame->data, &depthBuffer_[0], pFrame->dataSize,  count );
		xnOSMemCopy( pFrame->data, &depthBuffer_[0],  count );
		}
		*/
		// here, we assume pFrame->data has enough data space, so we do it inplace
		// first allocate enough storage for color point
		std::vector<DepthSpacePoint> pColorToDepthInfoOut;

		pColorToDepthInfoOut.resize(COLOR_WIDTH * COLOR_HEIGHT);

		hr = ICoordMapper->MapColorFrameToDepthSpace(nNativeBufferSize, pNativeBuffer, COLOR_WIDTH * COLOR_HEIGHT, &(pColorToDepthInfoOut[0]));
		if (FAILED(hr))
		{
			return 0;
		}

		// copy everything back to pFrame->data
		std::vector<DepthSpacePoint>::iterator itColorToDepth;
		UINT16 *pOutFrame = (UINT16*)(pFrame->data);
		UINT16 *pSrcDepth = (UINT16*)(pNativeBuffer);

		for (itColorToDepth = pColorToDepthInfoOut.begin(); itColorToDepth != pColorToDepthInfoOut.end(); itColorToDepth++)
		{
			if (itColorToDepth->X != -std::numeric_limits<float>::infinity() && itColorToDepth->Y != -std::numeric_limits<float>::infinity())
			{
				int depthX = static_cast<int>(itColorToDepth->X + 0.5f);
				int depthY = static_cast<int>(itColorToDepth->Y + 0.5f);

				if ((depthX >= 0 && depthX < DEPTH_NATIVE_WIDTH) && (depthY >= 0 && depthY < DEPTH_NATIVE_HEIGHT))
				{
					*pOutFrame = *(pNativeBuffer + DEPTH_NATIVE_WIDTH * depthY + depthX);
				}

				pOutFrame++;

			}
		}
#else
		if (pFrame->data != NULL && pNativeBuffer != NULL && pFrame->dataSize >= (int)nNativeBufferSize)
		{
			xnOSMemCopy(pFrame->data, pNativeBuffer, nNativeBufferSize);
		}

#endif
		++m_frameId;

		return 1;
	}

	int m_frameId;

	CComPtr<IKinectSensor>          m_pIKinectSensor;
	CComPtr<IDepthFrameReader>      m_pIDepthFrameReader;

	std::vector<UINT16> m_pOutputDepthBuffer;
};


class KinectV2Device : public oni::driver::DeviceBase
{
public:
	KinectV2Device(oni::driver::DriverServices& driverServices, CComPtr<IKinectSensor>& kinect)
		: m_pIKinectSensor(kinect)
		, m_driverServices(driverServices)
	{
#ifdef _COLOR_DEPTH_SAME_DIM
		m_ImageRegistrationMode = ONI_IMAGE_REGISTRATION_OFF;
#else
		m_ImageRegistrationMode = ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR;
#endif
		m_numSensors = 2;

		m_sensors[0].pSupportedVideoModes = m_VideoModeInfo[0];// XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[0].sensorType = ONI_SENSOR_DEPTH;
		m_sensors[0].numSupportedVideoModes = 1;
		m_sensors[0].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		m_sensors[0].pSupportedVideoModes[0].fps = 30;
		m_sensors[0].pSupportedVideoModes[0].resolutionX = DEPTH_OUTPUT_WIDTH;
		m_sensors[0].pSupportedVideoModes[0].resolutionY = DEPTH_OUTPUT_HEIGHT;

		m_sensors[1].pSupportedVideoModes = m_VideoModeInfo[1];// XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[1].sensorType = ONI_SENSOR_COLOR;
		m_sensors[1].numSupportedVideoModes = 1;
		m_sensors[1].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		m_sensors[1].pSupportedVideoModes[0].fps = 30;
		m_sensors[1].pSupportedVideoModes[0].resolutionX = COLOR_WIDTH;
		m_sensors[1].pSupportedVideoModes[0].resolutionY = COLOR_HEIGHT;
	}

	virtual OniBool isImageRegistrationModeSupported(OniImageRegistrationMode mode) 
	{ 
		// kinect 2 support depth to color 
		return (mode == ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}
	OniDeviceInfo* GetInfo()
	{
		return m_pInfo;
	}

	OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
	{
		*numSensors = m_numSensors;
		*pSensors = m_sensors;

		return ONI_STATUS_OK;
	}

	oni::driver::StreamBase* createStream(OniSensorType sensorType)
	{
		if (sensorType == ONI_SENSOR_COLOR)
		{
			return XN_NEW(KinectV2ColorStream, m_pIKinectSensor);
		}
		else  if (sensorType == ONI_SENSOR_DEPTH)
		{
			return XN_NEW(KinectV2DepthStream, m_pIKinectSensor);
		}

		return NULL;
	}

	void destroyStream(oni::driver::StreamBase* pStream)
	{
		XN_DELETE(pStream);
	}
	virtual OniBool isPropertySupported(int propertyId)
	{
#ifdef _COLOR_DEPTH_SAME_DIM
		if (propertyId == ONI_DEVICE_PROPERTY_DRIVER_VERSION)
#else
		if (propertyId == ONI_DEVICE_PROPERTY_DRIVER_VERSION || propertyId == ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION)
#endif
			return TRUE;
		return FALSE;
	}
	virtual OniStatus setProperty(int propertyId, const void* data, int dataSize) 
	{ 
		switch (propertyId)
		{
#ifndef _COLOR_DEPTH_SAME_DIM
			case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
			{
				m_ImageRegistrationMode = *((OniImageRegistrationMode*)(data));
				return ONI_STATUS_OK;
			}
			break;
#endif
			default:
				break;
		}
		return ONI_STATUS_NOT_IMPLEMENTED; 
	}
	OniStatus  getProperty(int propertyId, void* data, int* pDataSize)
	{
		OniStatus rc = ONI_STATUS_OK;

		switch (propertyId)
		{
			case ONI_DEVICE_PROPERTY_DRIVER_VERSION:
			{
				if (*pDataSize == sizeof(OniVersion))
				{
					OniVersion* version = (OniVersion*)data;
					version->major = version->minor = version->maintenance = version->build = 2;
				}
				else
				{
					m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVersion));
					rc = ONI_STATUS_BAD_PARAMETER;
				}
			}
			break;
#ifndef _COLOR_DEPTH_SAME_DIM
			case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
			{
			   if (*pDataSize == sizeof(OniImageRegistrationMode))
				{
				   OniImageRegistrationMode* pMode = (OniImageRegistrationMode*)data;
				   *pMode = m_ImageRegistrationMode;
				}
				else
				{
					m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVersion));
					rc = ONI_STATUS_BAD_PARAMETER;
				}
			}
				break;
#endif
			default:
			{
				m_driverServices.errorLoggerAppend("Unknown property: %d\n", propertyId);
				rc = ONI_STATUS_NOT_SUPPORTED;
			}
		}
		return rc;
	}
private:
	KinectV2Device(const KinectV2Device&);
	void operator=(const KinectV2Device&);

	CComPtr<IKinectSensor>	m_pIKinectSensor;
	OniImageRegistrationMode m_ImageRegistrationMode;
	OniDeviceInfo* m_pInfo;
	int m_numSensors;
	OniSensorInfo m_sensors[10];
	OniVideoMode m_VideoModeInfo[2][2];
	oni::driver::DriverServices& m_driverServices;
};

class Kinect2Driver : public oni::driver::DriverBase
{
public:
	Kinect2Driver(OniDriverServices* pDriverServices) : DriverBase(pDriverServices)
	{
		m_pDeviceInfo = NULL;
	}
	virtual ~Kinect2Driver()
	{
		if (m_pDeviceInfo)
			XN_DELETE(m_pDeviceInfo);
		m_pDeviceInfo = NULL;
	}
	virtual OniStatus initialize(
		oni::driver::DeviceConnectedCallback connectedCallback,
		oni::driver::DeviceDisconnectedCallback disconnectedCallback,
		oni::driver::DeviceStateChangedCallback deviceStateChangedCallback,
		void* pCookie)
	{
		OniStatus oniStatus = oni::driver::DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie);

		if (oniStatus != ONI_STATUS_OK)
			return oniStatus;

		// Open Kinect v2
		auto hr = ::GetDefaultKinectSensor(&m_pIKinectSensor);
		if (FAILED(hr))
		{
			return ONI_STATUS_NO_DEVICE;
		}

		hr = m_pIKinectSensor->Open();
		if (FAILED(hr))
		{
			std::cerr << "IKinectSensor::Open() failed." << std::endl;
			return ONI_STATUS_ERROR;
		}

		// Create device info
		m_pDeviceInfo = XN_NEW(OniDeviceInfo);
		if (m_pDeviceInfo != NULL)
		{
			xnOSStrCopy(m_pDeviceInfo->vendor, "Microsoft", ONI_MAX_STR);
			xnOSStrCopy(m_pDeviceInfo->name, "Kinect V2 Developer Preview", ONI_MAX_STR);
			xnOSStrCopy(m_pDeviceInfo->uri, "Kinect V2", ONI_MAX_STR);
		}
		// internal connect device
		deviceConnected(m_pDeviceInfo);
		deviceStateChanged(m_pDeviceInfo, 0);

		return ONI_STATUS_OK;
	}

	virtual oni::driver::DeviceBase* deviceOpen(const char* uri, const char* /*mode*/)
	{
		if (!m_pIKinectSensor)
		{
			return 0;
		}

		return XN_NEW(KinectV2Device, getServices(), m_pIKinectSensor);
	}

	virtual void deviceClose(oni::driver::DeviceBase* pDevice)
	{
		XN_DELETE(pDevice);
	}

	virtual OniStatus tryDevice(const char*)
	{
		return ONI_STATUS_ERROR;
	}

	void shutdown()
	{
	}

protected:
	OniDeviceInfo* m_pDeviceInfo;
	CComPtr<IKinectSensor>	m_pIKinectSensor;
};

ONI_EXPORT_DRIVER(Kinect2Driver);
