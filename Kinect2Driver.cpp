#include "Driver\OniDriverAPI.h"
#include "XnLib.h"
#include "XnHash.h"
#include "XnEvent.h"

#include <atlbase.h>
#include <Kinect.h>
#include <vector>
#include <iostream>

//#define _COLOR_DEPTH_SAME_DIM 1

#ifdef _COLOR_DEPTH_SAME_HIGHDEF_DIM
static const int        COLOR_WIDTH = 1920;
static const int        COLOR_HEIGHT = 1080;

static const int        DEPTH_OUTPUT_WIDTH = 1920;
static const int        DEPTH_OUTPUT_HEIGHT = 1080;
#else
static const int        COLOR_WIDTH = 512;
static const int        COLOR_HEIGHT = 424;

static const int        DEPTH_OUTPUT_WIDTH = 512;
static const int        DEPTH_OUTPUT_HEIGHT = 424;
#endif

static const int        DEPTH_NATIVE_WIDTH = 512;
static const int        DEPTH_NATIVE_HEIGHT = 424;
static const int        COLOR_NATIVE_WIDTH = 1920;
static const int        COLOR_NATIVE_HEIGHT = 1080;

typedef struct
{
	int refCount;
} KinectV2StreamFrameCookie;


class CKinectV2DeviceBase : public oni::driver::DeviceBase
{
public:
	virtual HRESULT RequestNewFrame(int nFrameType, int *pFrameIndex, INT64 *ptimestamp, const RGBQUAD **ppColorFrameData, const UINT16 **ppDepthFrameData, int *pPixelWidth, int *pPixelHeight) = 0;

};

class KinectV2Stream : public oni::driver::StreamBase
{
public:
	KinectV2Stream(CKinectV2DeviceBase *pDeviceBase)
		: oni::driver::StreamBase()
	{
		m_pDeviceBase = pDeviceBase;
	}
	~KinectV2Stream()
	{
		m_pDeviceBase = NULL;
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

	CKinectV2DeviceBase *m_pDeviceBase;

};

class KinectV2ColorStream : public KinectV2Stream
{
public:
	KinectV2ColorStream(CKinectV2DeviceBase *pBaseService)
		: KinectV2Stream(pBaseService)
	{
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

			if (BuildFrame(pFrame))
				raiseNewFrame(pFrame);

			getServices().releaseFrame(pFrame);
		}
	}

private:

	virtual int BuildFrame(OniFrame* pFrame)
	{
		HRESULT hRes = S_OK;

		if (m_pDeviceBase == NULL)
			return 0;
		// ask device to request new frame data, timestamp, frameindex

		//pFrame->frameIndex = m_frameId;
		const RGBQUAD *pColorFrameData = NULL;

		pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		pFrame->videoMode.fps = 30;
		INT64 iTimeStamp;

		hRes = m_pDeviceBase->RequestNewFrame(1/*frame type*/, &(pFrame->frameIndex), &(iTimeStamp), &pColorFrameData,
											  NULL, &(pFrame->width), &(pFrame->height));
		if (FAILED(hRes) || pColorFrameData == NULL)
			return 0;
		pFrame->timestamp = iTimeStamp;
		pFrame->videoMode.resolutionX = pFrame->width;
		pFrame->videoMode.resolutionY = pFrame->height;

		OniRGB888Pixel* pixel = (OniRGB888Pixel*)(pFrame->data);
		const int nPixelCount = pFrame->width * pFrame->height;

		for (int p = 0; p < nPixelCount; p++)
		{
			pixel->b = pColorFrameData->rgbBlue;
			pixel->g = pColorFrameData->rgbGreen;
			pixel->r = pColorFrameData->rgbRed;
			pColorFrameData++;
			pixel++;
		}

		pFrame->cropOriginX = pFrame->cropOriginY = 0;
		pFrame->croppingEnabled = FALSE;

		pFrame->sensorType = ONI_SENSOR_COLOR;
		pFrame->stride = pFrame->width * sizeof(OniRGB888Pixel);

		return 1;
	}
};


class KinectV2DepthStream : public KinectV2Stream
{
public:
	KinectV2DepthStream(CKinectV2DeviceBase *pBaseService)
		: KinectV2Stream(pBaseService)
	{
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
			if (BuildFrame(pFrame))
				raiseNewFrame(pFrame);
			getServices().releaseFrame(pFrame);
		}
	}

private:
	virtual int BuildFrame(OniFrame* pFrame)
	{
		HRESULT hRes = S_OK;

		if (m_pDeviceBase == NULL)
			return 0;
		// ask device to request new frame data, timestamp, frameindex

		//pFrame->frameIndex = m_frameId;
		const UINT16 *pDepthFrameData = NULL;

		pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		pFrame->videoMode.fps = 30;
		INT64 iTimeStamp;

		hRes = m_pDeviceBase->RequestNewFrame(2/*frame type*/, &(pFrame->frameIndex), &(iTimeStamp), NULL,
											  &pDepthFrameData, &(pFrame->width), &(pFrame->height));

		if (FAILED(hRes) || pDepthFrameData == NULL)
			return 0;
		pFrame->timestamp = iTimeStamp;

		pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		pFrame->videoMode.fps = 30;

		pFrame->videoMode.resolutionX = pFrame->width;
		pFrame->videoMode.resolutionY = pFrame->height;

		pFrame->cropOriginX = pFrame->cropOriginY = 0;
		pFrame->croppingEnabled = FALSE;

		pFrame->sensorType = ONI_SENSOR_DEPTH;
		pFrame->stride = pFrame->width * GetBytesPerPixel();

		xnOSMemCopy(pFrame->data, pDepthFrameData, pFrame->height * pFrame->stride);

		return 1;
	}
};


class KinectV2Device : public CKinectV2DeviceBase	//public oni::driver::DeviceBase
{
public:
	KinectV2Device(oni::driver::DriverServices& driverServices, CComPtr<IKinectSensor>& kinect)
		: m_pIKinectSensor(kinect)
		, m_driverServices(driverServices)
	{
		m_ImageRegistrationMode = ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR;

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

		m_nActiveFrameType = 0;	// current processing frame type, depth or color
		m_bFetchNewFrame = true;	// need to get new frame or not?
		m_nFrameTime = 0;
		m_nFrameIndex = 0;
		InitializeCriticalSection(&m_SCFrameRequest);
	}
	virtual ~KinectV2Device()
	{
		DeleteCriticalSection(&m_SCFrameRequest);
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
			return XN_NEW(KinectV2ColorStream, this);
		}
		else  if (sensorType == ONI_SENSOR_DEPTH)
		{
			return XN_NEW(KinectV2DepthStream, this);
		}

		return NULL;
	}

	void destroyStream(oni::driver::StreamBase* pStream)
	{
		XN_DELETE(pStream);
	}
	virtual OniBool isPropertySupported(int propertyId)
	{
		if (propertyId == ONI_DEVICE_PROPERTY_DRIVER_VERSION || propertyId == ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION)
			return TRUE;
		return FALSE;
	}
	virtual OniStatus setProperty(int propertyId, const void* data, int dataSize)
	{
		switch (propertyId)
		{
			case ONI_DEVICE_PROPERTY_IMAGE_REGISTRATION:
			{
					m_ImageRegistrationMode = *((OniImageRegistrationMode*)(data));
					return ONI_STATUS_OK;
			}
			break;
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

			default:
			{
					   m_driverServices.errorLoggerAppend("Unknown property: %d\n", propertyId);
					   rc = ONI_STATUS_NOT_SUPPORTED;
			}
		}
		return rc;
	}
	public:
		virtual HRESULT RequestNewFrame(int nFrameType,  int *pFrameIndex, INT64 *ptimestamp, 
										const RGBQUAD **ppColorFrameData, const UINT16 **ppDepthFrameData,
										int *pPixelWidth, int *pPixelHeight)
		{
			HRESULT hRes = S_OK;
			EnterCriticalSection(&m_SCFrameRequest);

			hRes = _RequestFrameType(nFrameType);
			if (FAILED(hRes))
				goto err_out;

			if (pFrameIndex)
				*pFrameIndex = m_nFrameIndex;

			if (ptimestamp)
				*ptimestamp = m_nFrameTime;

			if (ppColorFrameData && m_colorRGBX.size())
				*ppColorFrameData = &(m_colorRGBX[0]);
			if (ppDepthFrameData && m_pOutputDepthBuffer.size())
				*ppDepthFrameData = &(m_pOutputDepthBuffer[0]);
			if (pPixelWidth)
				*pPixelWidth = DEPTH_OUTPUT_WIDTH;
			if (pPixelHeight)
				*pPixelHeight = DEPTH_OUTPUT_HEIGHT;
		err_out:
			LeaveCriticalSection(&m_SCFrameRequest);
			
			return hRes;
		}
protected:
	// nType: 0: unknown, 1: color, 2: depth
	HRESULT _RequestFrameType(int nType)
	{
		HRESULT hRes = S_OK;

		if (nType == 0)
			return E_FAIL;

		if (m_nActiveFrameType == 0)
		{
			// initial state, no frame is fetched
			m_bFetchNewFrame = true;
		}
		else if (m_nActiveFrameType != nType)
		{
			// different type, use existing buffer, 
			m_bFetchNewFrame = false;
			
		}
		else
		{
			// active frame type is the same as previous request, that means,
			// data at the same timestamp is not requested by all data stream,
			// so, we just ignore all those 
			m_bFetchNewFrame = true;
		}
		

		if (m_bFetchNewFrame)
		{
			hRes = _FetchNewFrameData();
			if (!FAILED(hRes))
			{
				m_bFetchNewFrame = false;
				m_nActiveFrameType = nType;
			}
		}

		return hRes;
	}
	HRESULT _FetchNewFrameData(void)
	{
		// here, we get multi-frame source at once
		// color frame is the same size as depth
		CComPtr<IDepthFrame> pDepthFrame = NULL;
		CComPtr<IColorFrame> pColorFrame = NULL;
		CComPtr<IMultiSourceFrame> pMultiSourceFrame = NULL;
		HRESULT hRes = S_OK;

		if (m_pIKinectSensor)
		{
			// Initialize the Kinect and get coordinate mapper and the frame reader
			if (NULL == m_pICoorMapper)
			{
				hRes = m_pIKinectSensor->get_CoordinateMapper(&m_pICoorMapper);
				if (FAILED(hRes))
					return hRes;
			}

			hRes = m_pIKinectSensor->Open();

			if (SUCCEEDED(hRes) && m_pMultiSourceFrameReader == NULL)
			{
				hRes = m_pIKinectSensor->OpenMultiSourceFrameReader(
					FrameSourceTypes::FrameSourceTypes_Depth | 
					FrameSourceTypes::FrameSourceTypes_Color /*| FrameSourceTypes::FrameSourceTypes_BodyIndex*/,
					&m_pMultiSourceFrameReader);
			}
		}

		if (!m_pIKinectSensor || FAILED(hRes) || !m_pMultiSourceFrameReader)
		{
			//SetStatusMessage(L"No ready Kinect found!", 10000, true);
			hRes = E_FAIL;
		}
		// here, we have color-frame and depth data, copy it to internal buffer
		hRes = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

		if (SUCCEEDED(hRes))
		{
			CComPtr<IDepthFrameReference> pDepthFrameReference = NULL;

			hRes = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
			if (SUCCEEDED(hRes))
			{
				hRes = pDepthFrameReference->AcquireFrame(&pDepthFrame);
			}

			//SafeRelease(pDepthFrameReference);
		}

		if (SUCCEEDED(hRes))
		{
			CComPtr<IColorFrameReference> pColorFrameReference = NULL;

			hRes = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
			if (SUCCEEDED(hRes))
			{
				hRes = pColorFrameReference->AcquireFrame(&pColorFrame);
			}

			//SafeRelease(pColorFrameReference);
		}
		if (SUCCEEDED(hRes))
			hRes = pDepthFrame->get_RelativeTime(&m_nFrameTime);

		if (SUCCEEDED(hRes))
		{
			UINT nDepthBufferSize = 0;
			UINT16 *pDepthBuffer = NULL;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nColorBufferSize = 0;
			RGBQUAD *pColorBuffer = NULL;
			std::vector<RGBQUAD> pConvertedBuffer;

			hRes = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
			if (!FAILED(hRes))
			{
				m_pOutputDepthBuffer.resize(nDepthBufferSize);

				xnOSMemCopy(&(m_pOutputDepthBuffer[0]), pDepthBuffer, nDepthBufferSize * sizeof(UINT16));

				if (SUCCEEDED(hRes))
				{
					hRes = pColorFrame->get_RawColorImageFormat(&imageFormat);
				}

				if (SUCCEEDED(hRes))
				{
					if (imageFormat == ColorImageFormat_Bgra)
					{
						hRes = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
					}
					else 
					{
						pConvertedBuffer.resize(COLOR_NATIVE_WIDTH * COLOR_NATIVE_HEIGHT);

						pColorBuffer = &(pConvertedBuffer[0]);
						nColorBufferSize = COLOR_NATIVE_WIDTH * COLOR_NATIVE_HEIGHT * sizeof(RGBQUAD);

						hRes = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
					}
					{
						if (!FAILED(hRes))
						{
							m_colorRGBX.resize(DEPTH_OUTPUT_HEIGHT * DEPTH_OUTPUT_WIDTH);
							std::vector<ColorSpacePoint> pColorPt;
							pColorPt.resize(DEPTH_OUTPUT_HEIGHT * DEPTH_OUTPUT_WIDTH);
							hRes = m_pICoorMapper->MapDepthFrameToColorSpace(DEPTH_OUTPUT_WIDTH * DEPTH_OUTPUT_HEIGHT,
																			(const UINT16 *)pDepthBuffer,
																			DEPTH_OUTPUT_WIDTH * DEPTH_OUTPUT_HEIGHT,
																			&(pColorPt[0]));

							std::vector<RGBQUAD>::iterator itRGBOut = m_colorRGBX.begin();
							std::vector<ColorSpacePoint>::iterator itColorPt = pColorPt.begin();
							RGBQUAD *pSourceRGB = (RGBQUAD*)pColorBuffer;
							
							memset(pSourceRGB, 0, DEPTH_OUTPUT_HEIGHT * DEPTH_OUTPUT_WIDTH * sizeof(RGBQUAD));
							for (; itColorPt != pColorPt.end(); itRGBOut++, itColorPt++)
							{
								if (itColorPt->X != -std::numeric_limits<float>::infinity() && itColorPt->Y != -std::numeric_limits<float>::infinity())
								{
									int nCX = (int)(itColorPt->X + 0.5f);
									int nCY = (int)(itColorPt->Y + 0.5f);

									if (nCX >= 0 && nCX < COLOR_NATIVE_WIDTH && nCY >= 0 && nCY < COLOR_NATIVE_HEIGHT)
									{
										*itRGBOut = pSourceRGB[COLOR_NATIVE_WIDTH * nCY + nCX];
									}
								}
							}
						}
					}
				}
			}
		}
		m_nFrameIndex++;
		err_out:
		return hRes;
	}
private:
	KinectV2Device(const KinectV2Device&);
	void operator=(const KinectV2Device&);
	
	CComPtr<IKinectSensor>	m_pIKinectSensor;
	CComPtr<IMultiSourceFrameReader> m_pMultiSourceFrameReader;
	CComPtr<ICoordinateMapper> m_pICoorMapper;

	OniImageRegistrationMode m_ImageRegistrationMode;
	OniDeviceInfo* m_pInfo;
	int m_numSensors;
	OniSensorInfo m_sensors[10];
	OniVideoMode m_VideoModeInfo[2][2];
	oni::driver::DriverServices& m_driverServices;

	// here, we store the latest frame information
	int m_nActiveFrameType;	// current processing frame type, depth or color
	bool m_bFetchNewFrame;	// need to get new frame or not?
	INT64 m_nFrameTime;
	UINT m_nFrameIndex;
	CRITICAL_SECTION m_SCFrameRequest;

	// whenever a frame request arrives, all frame data are retrieved and stored.
	// if another frame request arrives, and this is the same frame type as m_nActiveFrameType,
	// then new frame data is retrieved.
	std::vector<UINT16> m_pOutputDepthBuffer;
	std::vector<RGBQUAD> m_colorRGBX;
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
