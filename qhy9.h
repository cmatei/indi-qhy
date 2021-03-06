#ifndef __QHY9_H
#define __QHY9_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>

#include <fitsio.h>

#include <indidevapi.h>
#include <indicom.h>
#include <indiccd.h>
#include <indifilterinterface.h>
#include <base64.h>

#include <libusb-1.0/libusb.h>

enum {
	SHUTTER_OPEN = 0,
	SHUTTER_CLOSE,
	SHUTTER_FREE
};

#define QHY9_SENSOR_WIDTH  3584
#define QHY9_SENSOR_HEIGHT 2574

#define QHY9_MAX_FILTERS 5

#define QHY9_USB_DEVID 0x16188301


class QHY9 : public INDI::CCD, INDI::FilterInterface
{
public:
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

	QHY9();
	~QHY9() {}

	/* Device */
	const char *getDefaultName() { return (char *) "QHY9"; }

	void ISGetProperties(const char *dev);
	bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
	bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
	bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);

	bool initProperties();
	bool updateProperties();
	bool saveConfigItems(FILE *fp);

	/* CCD Interface */
	bool Connect();
	bool Disconnect();

	bool StartExposure(float duration);
	bool AbortExposure();
	bool GrabExposure();

	int  SetTemperature(double temperature);

	bool UpdateCCDFrame(int x, int y, int w, int h);
	bool UpdateCCDBin(int binx, int biny);

	void addFITSKeywords(fitsfile *fptr, CCDChip *chip);

	/* Filter wheel Interface */
	int  QueryFilter();
	bool SelectFilter(int position);
	bool SetFilterNames();
	bool GetFilterNames(const char *groupName);

protected:
	void TimerHit();

	int pollTimer;

	libusb_device_handle *usb_handle;	 /* USB device handle */

	struct timeval exposure_start;	 /* used by the timer to call ExposureComplete() */
	double ExposureRequest;
	double calcTimeLeft();

	// Temperature control
	double TemperatureTarget;		 /* temperature setpoint in degC */

	unsigned char camgain;
	unsigned char camoffset;

#define Temperature TemperatureN[0].value

#define TECValue   TECN[0].value
#define TECPercent TECN[1].value
#define TECLimit   TECN[2].value

	// TEC power value and limit
	INumber TECN[3];
	INumberVectorProperty TECPowerNP;
	INumberVectorProperty TECLimitNP;

#define Gain GainN[0].value
	// gain
	INumber GainN[1];
	INumberVectorProperty GainNP;

#define Offset OffsetN[0].value
	// offset
	INumber OffsetN[1];
	INumberVectorProperty OffsetNP;

	// readout speed
	ISwitch ReadOutS[3];
	ISwitchVectorProperty ReadOutSP;

	// Camera Settings
	//unsigned char Gain;
	//unsigned char Offset;
	unsigned long Exptime;
	unsigned char HBIN;
	unsigned char VBIN;
	unsigned short LineSize;
	unsigned short VerticalSize;
	unsigned short SKIP_TOP;
	unsigned short SKIP_BOTTOM;
	unsigned short LiveVideo_BeginLine;
	unsigned short AnitInterlace;
	unsigned char MultiFieldBIN;
	unsigned char AMPVOLTAGE;
	unsigned char DownloadSpeed;
	unsigned char TgateMode;
	unsigned char ShortExposure;
	unsigned char VSUB;
	unsigned char CLAMP;
	unsigned char TransferBIT;
	unsigned char TopSkipNull;
	unsigned short TopSkipPix;
	unsigned char MechanicalShutterMode;
	unsigned char DownloadCloseTEC;
	unsigned char SDRAM_MAXSIZE;
	unsigned short ClockADJ;
	unsigned char Trig;
	unsigned char MotorHeating;   //0,1,2
	unsigned char WindowHeater;   //0-15

	/* I don't fully understand these */
	unsigned int p_size;
	unsigned int patchnum;
	unsigned int total_p;

	int bulk_transfer_read(int ep, unsigned char *data, int psize, int pnum, int *pos);

	double mv_to_degrees(double mv);
	double degrees_to_mv(double degrees);

	int  getDC201Interrupt();
	void setDC201Interrupt(uint8_t PWM, uint8_t FAN);

	void setCameraRegisters();

	void beginVideo();
	void abortVideo();

	void setShutter(int mode);

	void updateTemperature();
};

/* Utility functions */

#define tv_diff(t1, t2) ((((t1)->tv_sec - (t2)->tv_sec) * 1000) + (((t1)->tv_usec - (t2)->tv_usec) / 1000))

static inline double clamp_double(double val, double min, double max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}

static inline int clamp_int(int val, int min, int max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}

static inline uint8_t MSB(unsigned short val)
{
	return (val / 256);
}

static inline uint8_t LSB(unsigned short val)
{
	return (val & 0xff);
}


#endif
