#ifndef __QHY5_H
#define __QHY5_H

class QHYCCD;

class QHY5 : public QHYCCD
{
public:
	static const int QHY5_SENSOR_WIDTH = 1280;
	static const int QHY5_SENSOR_HEIGHT = 1024;

	/* vendor requests */
	static const int QHY5_START_EXPOSURE_CMD = 0x12;
	static const int QHY5_REGISTERS_CMD   = 0xB5;
	static const int QHY5_CFW_CMD         = 0xC1;

	static const int QHY5_DATA_BULK_EP = 0x86;
	static const int QHY5_INTERRUPT_WRITE_EP = 0x01;
	static const int QHY5_INTERRUPT_READ_EP  = 0x81;

        QHY5(libusb_device *usbdev) : QHYCCD(usbdev) { initCamera(); }
	~QHY5() {}

	const char *getDefaultName() { return "QHY5"; }

	void initCamera();

	int StartExposure(float duration);

private:
	void setCameraRegisters();

	void beginVideo();
	void abortVideo();

};

#endif
