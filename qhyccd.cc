#include "qhyccd.h"
#include "qhy9.h"
#include "qhy5.h"

using namespace std;

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

		case QHYCCD_QHY5_DEVID:
			camera = new QHY5(dev);
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



void QHYCCD::TimerHit()
{
	struct timeval now;
	unsigned long elapsed, read_wait;
	int usb_disabled = 0;

	if (!isConnected())
		return;

	/* delay readout until image is written to RAM */
	read_wait = Exptime;
	if (DownloadSpeed == 2 && PrimaryCCD.getBinX() == 1)
		read_wait += 16 * 1000;


	gettimeofday(&now, NULL);
	elapsed = tv_diff(&now, &exposure_start);

	if (exposing && elapsed >= Exptime)
		usb_disabled = 1;

	if (exposing && elapsed >= read_wait) {
		exposing = false;
		GrabExposure();
	}

	if (HasTemperatureControl && !usb_disabled)
		TempControlTimer();

	SetTimer(QHYCCD_TIMER);
}

bool QHYCCD::GetFilterNames(const char *groupName)
{
	char filterName[MAXINDINAME];
	char filterLabel[MAXINDILABEL];
	int i;

	if (FilterNameT != NULL)
		delete FilterNameT;

	FilterNameT = new IText[MaxFilter];

	for (i = 0; i < MaxFilter; i++) {
		snprintf(filterName, MAXINDINAME, "FILTER_SLOT_NAME_%d", i+1);
		snprintf(filterLabel, MAXINDILABEL, "Filter #%d", i+1);
		IUFillText(&FilterNameT[i], filterName, filterLabel, filterDesignation[i].c_str());
	}

	IUFillTextVector(FilterNameTP, FilterNameT, MaxFilter, deviceName(), "FILTER_NAME", "Filter", groupName, IP_RW, 1, IPS_IDLE);

	return true;
}


bool QHYCCD::initProperties()
{
	INDI::CCD::initProperties();


	if (HasFilterWheel) {
		initFilterProperties(deviceName(), FILTER_TAB);
		GetFilterNames(FILTER_TAB);
	}

	if (HasTemperatureControl) {
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
	}

	return true;
}


bool QHYCCD::updateProperties()
{
	INDI::CCD::updateProperties();

	if (isConnected()) {
		if (HasTemperatureControl) {
			defineNumber(TemperatureSetNV);
			defineNumber(TempPWMSetNV);
			defineNumber(TemperatureGetNV);
		}

		if (HasFilterWheel) {
			defineText(FilterNameTP);
			defineNumber(&FilterSlotNP);
		}
	} else {
		if (HasTemperatureControl) {
			deleteProperty(TemperatureSetNV->name);
			deleteProperty(TempPWMSetNV->name);
			deleteProperty(TemperatureGetNV->name);
		}

		if (HasFilterWheel) {
			deleteProperty(FilterNameTP->name);
			deleteProperty(FilterSlotNP.name);
		}
	}

	return true;
}

void QHYCCD::addFITSKeywords(fitsfile *fptr, CCDChip *chip)
{
	static char obsdata[128];
	float exposure;
	struct tm *dobs;
	int status = 0;

	/* Date of observation, includes time */
	dobs = gmtime(&exposure_start.tv_sec);
	snprintf(obsdata, 32, "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
		 1900 + dobs->tm_year, 1 + dobs->tm_mon, dobs->tm_mday,
		 dobs->tm_hour, dobs->tm_min, dobs->tm_sec,
		 (int) (exposure_start.tv_usec / 1000));

	fits_write_key(fptr, TSTRING, "DATE-OBS", obsdata, "Date of start of observation, UTC", &status);

	/* Time of observation for compatibility */
	snprintf(obsdata, 32, "%02d:%02d:%02d.%03d",
		 dobs->tm_hour, dobs->tm_min, dobs->tm_sec,
		 (int) (exposure_start.tv_usec / 1000));
	fits_write_key(fptr, TSTRING, "TIME-OBS", obsdata, "Time of start of observation, UTC", &status);

	/* Exposure time */
	exposure = Exptime / 1000.0;
	fits_write_key(fptr, TFLOAT, "EXPTIME", &exposure, "Exposure time in seconds", &status);

	/* Binning */
	fits_write_key(fptr, TBYTE, "CCDBIN1", &HBIN, "CCD BIN X", &status);
	fits_write_key(fptr, TBYTE, "CCDBIN2", &VBIN, "CCD BIN Y", &status);

	/* Gain */
	fits_write_key(fptr, TBYTE, "QHYGAIN", &Gain, "CCD Gain, 0..255", &status);

	/* Offset */
	fits_write_key(fptr, TBYTE, "QHYBIAS", &Offset, "CCD Offset", &status);

	/* Readout speed */
	fits_write_key(fptr, TBYTE, "QHYSPEED", &DownloadSpeed, "CCD readout speed", &status);

	/* CLAMP */
	fits_write_key(fptr, TBYTE, "QHYCLAMP", &CLAMP, "CCD clamp, on/off", &status);


	/* CCD Temperature */
	if (HasTemperatureControl) {
		fits_write_key(fptr, TDOUBLE, "CCDTEMP", &Temperature, "CCD temperature, degC", &status);
		fits_write_key(fptr, TDOUBLE, "CCDTSET", &TemperatureTarget, "CCD set temperature, degC", &status);
	}

	/* Filters */
	if (HasFilterWheel) {
		void *fname = (void *) filterDesignation[CurrentFilter - 1].c_str();

		fits_write_key(fptr, TSTRING, "FILTER", fname, "Filter name", &status);
		fits_write_key(fptr, TINT, "FLT-SLOT", &CurrentFilter, "Filter slot", &status);
	}
}


bool QHYCCD::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
	double v;
	INumber *np;

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

		if (HasFilterWheel && !strcmp(name, FilterSlotNP.name)) {
			TargetFilter = values[0];

			np = IUFindNumber(&FilterSlotNP, names[0]);

			if (!np) {
				FilterSlotNP.s = IPS_ALERT;
				IDSetNumber(&FilterSlotNP, "Unknown error. %s is not a member of %s property.", names[0], name);
				return false;
			}

			if (TargetFilter < MinFilter || TargetFilter > MaxFilter) {
				FilterSlotNP.s = IPS_ALERT;
				IDSetNumber(&FilterSlotNP, "Error: valid range of filter is from %d to %d", MinFilter, MaxFilter);
				return false;
			}

			IUUpdateNumber(&FilterSlotNP, values, names, n);

			SelectFilter(TargetFilter);

			FilterSlotN[0].value = TargetFilter;
			FilterSlotNP.s = IPS_OK;
			IDSetNumber(&FilterSlotNP, "Setting current filter to slot %d", TargetFilter);

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
	int i;

	if (dev && !strcmp(dev, deviceName())) {
		if (!strcmp(name, FilterNameTP->name)) {
			if (IUUpdateText(FilterNameTP, texts, names, n) < 0) {
				FilterNameTP->s = IPS_ALERT;
				IDSetText(FilterNameTP, "Error updating names. XML corrupted.");
				return false;
			}

			for (i=0; i < MaxFilter; i++)
				filterDesignation[i] = FilterNameT[i].text;

			if (SetFilterNames() == true) {
				FilterNameTP->s = IPS_OK;
				IDSetText(FilterNameTP, NULL);
				return true;
			} else {
				FilterNameTP->s = IPS_ALERT;
				IDSetText(FilterNameTP, "Error updating filter names.");
				return false;
			}

		}
	}

	return CCD::ISNewText(dev, name, texts, names, n);
}

int QHYCCD::bulk_transfer_read(int ep, unsigned char *data, int psize, int pnum, int *pos)
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
