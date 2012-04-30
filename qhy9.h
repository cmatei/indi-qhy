#ifndef __QHY9_H
#define __QHY9_H

class QHYCCD;

class QHY9 : public QHYCCD
{
public:
	static const int SHUTTER_OPEN = 0;
	static const int SHUTTER_CLOSE = 1;
	static const int SHUTTER_FREE = 2;

	static const int QHY9_SENSOR_WIDTH = 3584;
	static const int QHY9_SENSOR_HEIGHT = 2574;

	/* control request types */
	// LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
	static const int QHY9_VENDOR_REQUEST_WRITE = 0x40;

	// LIBUSB_ENDPOINT_IN  | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
	static const int QHY9_VENDOR_REQUEST_READ  = 0xC0;

	/* vendor request commands */
	static const int QHY9_BEGIN_VIDEO_CMD = 0xB3;
	static const int QHY9_REGISTERS_CMD   = 0xB5;
	static const int QHY9_CFW_CMD         = 0xC1;
	static const int QHY9_VERSION_CMD     = 0xC2;
	static const int QHY9_SHUTTER_CMD     = 0xC7;

	static const int QHY9_DATA_BULK_EP       = 0x86;
	static const int QHY9_INTERRUPT_WRITE_EP = 0x01;
	static const int QHY9_INTERRUPT_READ_EP  = 0x81;

	QHY9(libusb_device *usbdev) : QHYCCD(usbdev) {
		initCamera();
		ReadOutSP = new ISwitchVectorProperty;
	}

	~QHY9() { delete ReadOutSP; }

	const char *getDefaultName() { return "QHY9"; }

	void initCamera();

	bool initProperties();
	bool updateProperties();

	int StartExposure(float duration);
	bool ExposureComplete();

	void addFITSKeywords(fitsfile *fptr);

	bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);

	bool SetFilterNames();
	bool SelectFilter(int i);
	int  QueryFilter();

protected:

	void TempControlTimer();

private:

	// readout speed
	ISwitch ReadOutS[3];
	ISwitchVectorProperty *ReadOutSP;


	double mv_to_degrees(double mv);
	double degrees_to_mv(double degrees);

	int  getDC201Interrupt();
	void setDC201Interrupt(uint8_t PWM, uint8_t FAN);

	void setCameraRegisters();

	void beginVideo();
	void abortVideo();

	void setShutter(int mode);

};

#endif
