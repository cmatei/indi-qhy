#ifndef __QHY_CCD_H
#define __QHY_CCD_H

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

#define QHYCCD_QHY9_DEVID 0x16188301
#define QHYCCD_QHY5_DEVID 0x16C0296D

#define QHYCCD_TIMER (1 * 1000)

#define QHYCCD_MAX_FILTERS 5

class QHYCCD : public INDI::CCD, public INDI::FilterInterface
{
public:
	static QHYCCD *detectCamera();

        QHYCCD(libusb_device *usbdev) {
		this->usb_dev = usbdev;

		HasTemperatureControl = false;
		HasGuideHead = false;
		HasSt4Port = false;
		HasFilterWheel = false;

		GainOffsetSetNV = new INumberVectorProperty;
	}

	~QHYCCD() { Disconnect(); }

	/* INDI stuff */
	virtual bool Connect();
	virtual bool Disconnect();

	virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
	virtual bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);

	virtual bool initProperties();
	virtual bool updateProperties();

	virtual const char *deviceName() = 0;

	virtual void addFITSKeywords(fitsfile *fptr, CCDChip *chip);
	virtual bool GrabExposure() = 0;

	/* Not all cameras have CFW interface */
	virtual bool GetFilterNames(const char *groupName);
	virtual bool SetFilterNames() { return false; }
	virtual bool SelectFilter(int i) { return false; }
	virtual int QueryFilter() { return 0; }

protected:

	bool HasFilterWheel;
	std::string filterDesignation[QHYCCD_MAX_FILTERS];

	virtual void TempControlTimer() {}
	void   TimerHit();

	libusb_device        *usb_dev;	         /* USB device address */
	libusb_device_handle *usb_handle;	 /* USB device handle */

	struct timeval        exposure_start;	 /* used by the timer to call ExposureComplete() */

	/* Temperature control */
	bool    HasTemperatureControl;
	double  TemperatureTarget;		 /* temperature setpoint in degC */
	double  Temperature;		         /* current temperature in degC */
	int     TEC_PWMLimit;		         /* 0..100, TEC power limit */
	int     TEC_PWM;		         /* current TEC power */
	INumber TemperatureN[4];

	INumberVectorProperty *TemperatureSetNV;  /* temp setpoint */
	INumberVectorProperty *TempPWMSetNV;      /* PWM limit */
	INumberVectorProperty *TemperatureGetNV;  /* R/O, current temp and PWM */

	INumber GainOffsetN[2];
	INumberVectorProperty *GainOffsetSetNV;         /* gain & offset*/

	/* QHY Camera Settings */
	unsigned char Gain;
	unsigned char Offset;
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
	int p_size;
	int patchnum;
	int total_p;

	int bulk_transfer_read(int ep, unsigned char *data, int psize, int pnum, int *pos);
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
