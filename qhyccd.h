#ifndef __QHY_CCD_H
#define __QHY_CCD_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <string>
#include <sys/time.h>

#include <fitsio.h>

#include <indidevapi.h>
#include <indicom.h>
#include <defaultdriver.h>
#include <indiccd.h>
#include <base64.h>

#include <libusb-1.0/libusb.h>

#define QHYCCD_QHY9_DEVID 0x16188301
#define QHYCCD_QHY5_DEVID 0x16C0296D

#define QHYCCD_TIMER (1 * 1000)

class QHYCCD : public INDI::CCD
{
public:
	static const int QHYCCD_REQUEST_READ  = 0xC0;
	static const int QHYCCD_REQUEST_WRITE = 0x40;

        QHYCCD(libusb_device *usbdev) : CCD() { initDefaults(); this->usb_dev = usbdev; }
	~QHYCCD() { Disconnect(); }

	static QHYCCD *detectCamera();

	virtual void   initDefaults();

	/* INDI stuff */
	bool Connect();
	bool Disconnect();

	bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
	bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
	bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);

	bool initProperties();
	bool updateProperties();

protected:
	virtual void TempControlTimer() {}
	void   TimerHit();

	virtual void SetCFWSlot(int slot) { }

	int vendor_request_read(uint8_t req, uint8_t *data, uint16_t length);
	int vendor_request_write(uint8_t req, uint8_t *data, uint16_t length);

	int bulk_transfer_read(uint8_t ep, uint8_t *data, int p_size, int p_num, int* pos);

	bool                  usb_connected;
	libusb_device        *usb_dev;	         /* USB device address */
	libusb_device_handle *usb_handle;	 /* USB device handle */

	bool                  exposing;	         /* true if currently exposing */
	struct timeval        exposure_start;	 /* used by the timer to call ExposureComplete() */

	/* Temperature control */
	bool    HasTemperatureControl;
	double  TemperatureTarget;		 /* in degC */
	double  Temperature;		         /* in degC */
	int     TEC_PWMLimit;		         /* 0..100, TEC power limit */
	int     TEC_PWM;		         /* current TEC power */
	INumber TemperatureN[4];

	INumberVectorProperty *TemperatureSetNV;  /* temp setpoint */
	INumberVectorProperty *TempPWMSetNV;      /* PWM limit */
	INumberVectorProperty *TemperatureGetNV;  /* R/O, current temp and PWM */


	/* CFW */
	bool    HasColorFilterWheel;
	int     CFWSlot;		         /* CFW slot */
	INumber CFWSlotN[1];
	INumberVectorProperty *CFWSlotNV;

	IText   CFWFilterT[5];
	ITextVectorProperty *CFWFilterTV;

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
