#include "qhyccd.h"
#include "qhy9.h"

using namespace std;

void QHYCCD::initDefaults()
{
	HasTemperatureControl = false;
	HasColorFilterWheel = false;

	TemperatureTarget = 50.0;
	Temperature = 50.0;
	TEC_PWM = 0;
	TEC_PWMLimit = 80;
}

QHYCCD *QHYCCD::detectCamera()
{
	static bool libusb_initialized = false;
	libusb_device **devices;
	struct libusb_device_descriptor desc;
	unsigned int devID;
	int i, n;
	QHYCCD *camera = NULL;

	if (!libusb_initialized) {
		if (libusb_init(NULL))
			return NULL;

		libusb_initialized = true;
	}

	n = libusb_get_device_list(NULL, &devices);

	for (i = 0; i < n && !camera; i++) {
		libusb_device *dev = devices[i];

		if (libusb_get_device_descriptor(dev, &desc) < 0)
			continue;

		devID = (desc.idVendor << 16) + desc.idProduct;

		switch (devID) {

		case QHYCCD_QHY9_DEVID:
			camera = new QHY9(dev);
			break;
		}
	}

	if (camera)
		libusb_ref_device(camera->usb_dev);

	libusb_free_device_list(devices, 1);

	return camera;
}

bool QHYCCD::Connect()
{
	if (usb_connected)
		return true;

	usb_connected = libusb_open(usb_dev, &usb_handle) ? false : true;

	if (usb_connected) {
		SetTimer(QHYCCD_TIMER);
	}

	return usb_connected;
}


bool QHYCCD::Disconnect()
{
	if (usb_connected) {
		libusb_close(usb_handle);
		usb_connected = false;
	}

	return true;
}


int QHYCCD::vendor_request_read(uint8_t req, uint8_t *data, uint16_t length)
{
	return libusb_control_transfer(usb_handle, QHYCCD_REQUEST_READ, req, 0, 0, data, length, 0);
}

int QHYCCD::vendor_request_write(uint8_t req, uint8_t *data, uint16_t length)
{
	return libusb_control_transfer(usb_handle, QHYCCD_REQUEST_WRITE, req, 0, 0, data, length, 0);
}

int QHYCCD::bulk_transfer_read(uint8_t ep, uint8_t *data, int psize, int pnum, int* pos)
{
        int ret, length_transfered;
        int i;

        for (i = 0; i < pnum; ++i) {
		length_transfered = 0;

                ret = libusb_bulk_transfer(usb_handle ,ep, data + i * psize, psize, &length_transfered, 0);
                if (ret < 0 || length_transfered != psize) {
			fprintf(stderr, "bulk_transfer %d, %d\n", ret, length_transfered);
			return -1;
                }
                *pos = i;
        }

        return 0;
}

void QHYCCD::TimerHit()
{
	struct timeval now;
	static int counter = 0;

	fprintf(stderr, "TIMER !!\n\n");

	if (isConnected()) {

		gettimeofday(&now, NULL);
		if (exposing && (tv_diff(&now, &exposure_start) >= Exptime)) {
			exposing = false;
			ExposureComplete();
		}

		if (HasTemperatureControl) {
			/* make this slower, at 2 sec */

			counter++;
			if (counter >= 2) {
				counter = 0;
				TempControlTimer();
			}
		}

		SetTimer(QHYCCD_TIMER);
	}
}

bool QHYCCD::initProperties()
{
	CCD::initProperties();

	/* Temperature control */

	IDMessage(deviceName(), "initProperties()");

	/* FIXME: this is ugly, with the 3 groups, but e.g. xephem sends all values in a group when setting, so
	          trying to avoid overwriting temp setpoint with current temp for instance */

	TemperatureSetNV = new INumberVectorProperty();
	IUFillNumber(&TemperatureN[0], "CCD_TEMPERATURE_VALUE", "Temp. Setpoint (degC)", "%4.2f", -50, 50, 0, TemperatureTarget);
	IUFillNumberVector(TemperatureSetNV, &TemperatureN[0], 1, deviceName(), "CCD_TEMPERATURE", "Temperature", "Main Control", IP_RW, 60, IPS_IDLE);

	TempPWMSetNV = new INumberVectorProperty();
	IUFillNumber(&TemperatureN[1], "CCD_TEC_PWM_LIMIT_VALUE", "TEC Power limit (%)", "%3.0f", 0, 100, 0, TEC_PWMLimit);
	IUFillNumberVector(TempPWMSetNV, &TemperatureN[1], 1, deviceName(), "CCD_TEC_PWM_LIMIT", "TEC Power", "Main Control", IP_RW, 60, IPS_IDLE);

	TemperatureGetNV = new INumberVectorProperty();
	IUFillNumber(&TemperatureN[2], "CCD_TEMPERATURE_CURRENT_VALUE", "Temperature (degC)", "%4.2f", -50, 50, 0, Temperature);
	IUFillNumber(&TemperatureN[3], "CCD_TEC_PWM_CURRENT_VALUE", "TEC Power (%)", "%3.0f", 0, 100, 0, TEC_PWM);
	IUFillNumberVector(TemperatureGetNV, &TemperatureN[2], 2, deviceName(), "CCD_TEMPERATURE_INFO", "Temperature", "Temperature Info", IP_RO, 60, IPS_IDLE);

	/* CFW */
	CFWSlotNV = new INumberVectorProperty();
	IUFillNumber(&CFWSlotN[0], "FILTER_SLOT_VALUE", "CFW slot", "%2.0f", 1, 5, 0, CFWSlot);
	IUFillNumberVector(CFWSlotNV, CFWSlotN, 1, deviceName(), "FILTER_SLOT", "Filter", "Main Control", IP_RW, 60, IPS_IDLE);

	CFWFilterTV = new ITextVectorProperty();
	IUFillText(&CFWFilterT[0], "FILTER1", "1", "");
	IUFillText(&CFWFilterT[1], "FILTER2", "2", "");
	IUFillText(&CFWFilterT[2], "FILTER3", "3", "");
	IUFillText(&CFWFilterT[3], "FILTER4", "4", "");
	IUFillText(&CFWFilterT[4], "FILTER5", "5", "");
	IUFillTextVector(CFWFilterTV, CFWFilterT, 5, deviceName(), "FILTER_NAME", "Filter", "Filter Wheel", IP_RW, 60, IPS_IDLE);

	return true;
}

bool QHYCCD::updateProperties()
{
	CCD::updateProperties();

	if (isConnected()) {

		if (HasTemperatureControl) {
			defineNumber(TemperatureSetNV);
			defineNumber(TempPWMSetNV);
			defineNumber(TemperatureGetNV);
		}

		if (HasColorFilterWheel) {
			defineNumber(CFWSlotNV);
			defineText(CFWFilterTV);
		}

	} else {
		if (HasTemperatureControl) {
			deleteProperty(TemperatureSetNV->name);
			deleteProperty(TempPWMSetNV->name);
			deleteProperty(TemperatureGetNV->name);
		}

		if (HasTemperatureControl) {
			deleteProperty(CFWSlotNV->name);
			deleteProperty(CFWFilterTV->name);
		}
	}

	return true;
}


bool QHYCCD::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
	double v;

	fprintf(stderr, "ISNEWNUMBER dev %s, device %s, name %s\n\n", dev, deviceName(), name);
	if (dev && !strcmp(dev, deviceName())) {
		if (HasTemperatureControl && !strcmp(name, "CCD_TEMPERATURE")) {
			if (n < 1) return false;

			v = clamp_double(values[0], -50, 50);

			TemperatureTarget = v;
			TemperatureN[0].value = v;
			TemperatureSetNV->s = IPS_OK;
			IDSetNumber(TemperatureSetNV, NULL);

			return true;
		}

		if (HasTemperatureControl && !strcmp(name, "CCD_TEC_PWM_LIMIT")) {
			if (n < 1) return false;

			v = clamp_double(values[0], 0, 90);

			TEC_PWMLimit = v;
			TemperatureN[1].value = v;
			TempPWMSetNV->s = IPS_OK;
			IDSetNumber(TempPWMSetNV, NULL);

			return true;
		}

		if (HasColorFilterWheel && !strcmp(name, "FILTER_SLOT")) {
			if (n < 1) return false;

			v = clamp_double(values[0], 1, 5);

			CFWSlot = v;
			CFWSlotN[0].value = v;
			CFWSlotNV->s = IPS_OK;

			setCFWSlot(CFWSlot);

			IDSetNumber(CFWSlotNV, NULL);

			return true;
		}
	}

	return CCD::ISNewNumber(dev, name, values, names, n);
}

bool QHYCCD::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	if (dev && !strcmp(dev, deviceName())) {
	}

	return CCD::ISNewSwitch(dev, name, states, names, n);
}

bool QHYCCD::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
#if 0
	IUFillText(&CFWFilterT[0], "FILTER1", "1", "");
	IUFillText(&CFWFilterT[1], "FILTER2", "2", "");
	IUFillText(&CFWFilterT[2], "FILTER3", "3", "");
	IUFillText(&CFWFilterT[3], "FILTER4", "4", "");
	IUFillText(&CFWFilterT[4], "FILTER5", "5", "");
#endif

	if (dev && !strcmp(dev, deviceName())) {
	}

	return CCD::ISNewText(dev, name, texts, names, n);
}

