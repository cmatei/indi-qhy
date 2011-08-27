#ifndef __QHY9_H
#define __QHY9_H

class QHYCCD;

class QHY9 : public QHYCCD
{
	static const int SHUTTER_OPEN = 0;
	static const int SHUTTER_CLOSE = 1;
	static const int SHUTTER_FREE = 2;

	static const int QHY9_SENSOR_WIDTH = 3584;
	static const int QHY9_SENSOR_HEIGHT = 2574;

	/* vendor requests */
	static const int QHY9_SHUTTER_CMD = 0xC7;
	static const int QHY9_BEGIN_VIDEO_CMD = 0xB3;
	static const int QHY9_REGISTERS_CMD = 0xB5;

	static const int QHY9_DATA_BULK_EP = 0x86;
	static const int QHY9_INTERRUPT_WRITE_EP = 0x01;
	static const int QHY9_INTERRUPT_READ_EP  = 0x81;

public:
	QHY9(libusb_device *usbdev)
		: QHYCCD(usbdev) { initDefaults(); setDeviceName("QHY9"); }
	~QHY9() {}

	void initDefaults();

	/* INDI */
	bool initProperties();
	bool updateProperties();

	const char *getDefaultName() { return "QHY9"; }

	int StartExposure(float duration);
	bool ExposureComplete();

protected:

	void TempControlTimer();

private:

	double mv_to_degrees(double mv);
	double degrees_to_mv(double degrees);

	int  getDC201();
	int  getDC201Interrupt();
	void setDC201(uint8_t PWM, uint8_t FAN);
	void setDC201Interrupt(uint8_t PWM, uint8_t FAN);

	void setCameraRegisters();

	void beginVideo();
	void abortVideo();

	void setShutter(int mode);

};

#endif
