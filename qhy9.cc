
#include "qhy9.h"

static QHY9 *camera = NULL;

void initialize()
{
	if (camera)
		return;

	camera = new QHY9();
}


void ISGetProperties(const char *dev)
{
	initialize();

	if (!camera || (dev && strcmp(dev, camera->getDeviceName())))
		return;

	camera->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	initialize();

	if (!camera || (dev && strcmp(dev, camera->getDeviceName())))
		return;

	camera->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	initialize();

	if (!camera || (dev && strcmp(dev, camera->getDeviceName())))
		return;

	camera->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	initialize();

	if (!camera || (dev && strcmp(dev, camera->getDeviceName())))
		return;

	camera->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
	INDI_UNUSED(dev);
	INDI_UNUSED(name);
	INDI_UNUSED(sizes);
	INDI_UNUSED(blobsizes);
	INDI_UNUSED(blobs);
	INDI_UNUSED(formats);
	INDI_UNUSED(names);
	INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
	INDI_UNUSED(root);
}


QHY9::QHY9()
	: INDI::CCD()
{
	usb_handle = NULL;

	CCDCapability cap = {
		.hasGuideHead = false,
		.hasST4Port = false,
		.hasShutter = true,
		.hasCooler = true,
		.canBin = true,
		.canSubFrame = true,
		.canAbort = true,
		.hasBayer = false,
	};

	SetCCDCapability(&cap);

	TemperatureTarget = 50.0;
	Temperature = 50.0;
	TEC_PWM = 0;
	TEC_PWMLimit = 80;

	Gain = 20;
	Offset = 120;
	CLAMP = 0;
	MechanicalShutterMode = 0;
	DownloadCloseTEC = 1;
	SDRAM_MAXSIZE = 100;

	// default to slowest readout
	DownloadSpeed = 2;

	// sim = false
}


bool QHY9::Connect()
{
	libusb_device **devices;
	libusb_device *dev;
	struct libusb_device_descriptor desc;
	unsigned int devID;
	int i, n;

	/* already connected ? */
	if (usb_handle)
		return true;

	if (libusb_init(NULL))
		return false;

	n = libusb_get_device_list(NULL, &devices);
	for (i = 0; i < n; i++) {
		dev = devices[i];

		if (libusb_get_device_descriptor(dev, &desc) < 0)
			continue;

		devID = (desc.idVendor << 16) + desc.idProduct;
		if (devID == QHY9_USB_DEVID) {

			libusb_ref_device(dev);
			SetTimer(QHYCCD_TIMER);

			libusb_free_device_list(devices, 1);

			return true;
		}
	}

	libusb_free_device_list(devices, 1);

	return true;
}


bool QHY9::Disconnect()
{
	if (usb_handle) {
		libusb_close(usb_handle);
		usb_handle = NULL;
	}

	libusb_exit(NULL);

	return true;
}


void QHY9::TimerHit()
{
	struct timeval now;

	if (!isConnected())
		return;

	gettimeofday(&now, NULL);
	if (InExposure && tv_diff(&now, &exposure_start) >= (long) Exptime)
		GrabExposure();

	if (HasTemperatureControl)
		TempControlTimer();

	SetTimer(QHYCCD_TIMER);
}

bool QHY9::initProperties()
{
	INDI::CCD::initProperties();
	initFilterProperties(getDeviceName(), FILTER_TAB);

	FilterSlotN[0].min = 1;
	FilterSlotN[0].max = 5;

	/* Readout speed */
	IUFillSwitch(&ReadOutS[0], "READOUT_FAST",   "Fast",   (DownloadSpeed == 0) ? ISS_ON : ISS_OFF);
	IUFillSwitch(&ReadOutS[1], "READOUT_NORMAL", "Normal", (DownloadSpeed == 1) ? ISS_ON : ISS_OFF);
	IUFillSwitch(&ReadOutS[2], "READOUT_SLOW",   "Slow",   (DownloadSpeed == 2) ? ISS_ON : ISS_OFF);
	IUFillSwitchVector(&ReadOutSP, ReadOutS, 3, getDeviceName(), "READOUT_SPEED", "Readout Speed",
			   IMAGE_SETTINGS_TAB, IP_WO, ISR_1OFMANY, 0, IPS_IDLE);

	/* Gain & Offset */
	IUFillNumber(&GainOffsetN[0], "QHY_GAIN",   "Gain",   "%3.0f", 0, 255, 0, Gain);
	IUFillNumber(&GainOffsetN[1], "QHY_OFFSET", "Offset", "%3.0f", 0, 255, 0, Offset);
	IUFillNumberVector(&GainOffsetSetNV, &GainOffsetN[0], 2, getDeviceName(), "QHY_SETTINGS", "QHY Settings",
			   IMAGE_SETTINGS_TAB, IP_RW, 60, IPS_IDLE);

	/* Temperature */
	IUFillNumber(&TemperatureN[0], "CCD_TEMPERATURE_VALUE", "Temp. Setpoint (degC)", "%4.2f", -50, 50, 0, TemperatureTarget);
	IUFillNumberVector(&TemperatureSetNV, &TemperatureN[0], 1, getDeviceName(), "CCD_TEMPERATURE", "Temperature",
			   MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

	IUFillNumber(&TemperatureN[1], "CCD_TEC_PWM_LIMIT_VALUE", "TEC Power limit (%)", "%3.0f", 0, 100, 0, TEC_PWMLimit);
	IUFillNumberVector(&TempPWMSetNV, &TemperatureN[1], 1, getDeviceName(), "CCD_TEC_PWM_LIMIT", "TEC Power",
			   MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

	IUFillNumber(&TemperatureN[2], "CCD_TEMPERATURE_CURRENT_VALUE", "Temperature (degC)", "%4.2f", -50, 50, 0, Temperature);
	IUFillNumber(&TemperatureN[3], "CCD_TEC_PWM_CURRENT_VALUE", "TEC Power (%)", "%3.0f", 0, 100, 0, TEC_PWM);
	IUFillNumberVector(&TemperatureGetNV, &TemperatureN[2], 2, getDeviceName(), "CCD_TEMPERATURE_INFO", "Current Temperature",
			   MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);


	PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", 0.001, 3600, 1, false);

	addAuxControls();

	setInterfaceDescriptor(getInterfaceDescriptor() | FILTER_INTERFACE);

	return true;
}

void QHY9::ISGetProperties(const char *dev)
{
	INDI::CCD::ISGetProperties(dev);

	if (isConnected()) {
		defineSwitch(&ReadOutSP);
		defineNumber(&GainOffsetSetNV);
		defineNumber(&TemperatureSetNV);
		defineNumber(&TempPWMSetNV);
		defineNumber(&TemperatureGetNV);
		defineText(FilterNameTP);
	}
}

bool QHY9::updateProperties()
{
	INDI::CCD::updateProperties();

	if (isConnected()) {
		defineSwitch(&ReadOutSP);

		defineNumber(&GainOffsetSetNV);
		defineNumber(&TemperatureSetNV);
		defineNumber(&TempPWMSetNV);
		defineNumber(&TemperatureGetNV);

		defineNumber(&FilterSlotNP);
		GetFilterNames(FILTER_TAB);
		defineText(FilterNameTP);

		SetCCDParams(QHY9_SENSOR_WIDTH, QHY9_SENSOR_HEIGHT, 16, 5.4, 5.4);
	} else {
		deleteProperty(ReadOutSP.name);
		deleteProperty(GainOffsetSetNV.name);
		deleteProperty(TemperatureSetNV.name);
		deleteProperty(TempPWMSetNV.name);
		deleteProperty(TemperatureGetNV.name);
	}

	return true;
}

bool QHY9::StartExposure(float duration)
{
	CCDChip::CCD_FRAME type;

//	if (InExposure)
//		return false;

	type = PrimaryCCD.getFrameType();

	/* 0 sec exposures are not done for anything but BIAS.
	   This is useful for the load/save config feature of INDI drivers,
	   so they won't trigger an exposure when loading config */
	if (type != CCDChip::BIAS_FRAME && duration == 0.0)
		return false;

	Exptime = duration * 1000;

	setCameraRegisters();
	usleep(100000);


	if (type == CCDChip::DARK_FRAME || type == CCDChip::BIAS_FRAME) {
		//fprintf(stderr, "SHOOTING A DARK, CLOSING SHUTTER\n");
		setShutter(SHUTTER_CLOSE);
		usleep(500*1000);		     // shutter speed is 1/10 to 1/2 sec
	}

	InExposure = true;
	gettimeofday(&exposure_start, NULL);

	beginVideo();

	return true;
}

bool QHY9::AbortExposure()
{
// FIXME: if camera still locks on exposure transfer, check if we can still abort
// or the camera is dead
//	if (!InExposure)
//		return true;

	abortVideo();
	InExposure = false;

	return true;
}


bool QHY9::GrabExposure()
{
	static uint16_t *buffer = NULL;
	static size_t bufsize = 0;
	struct timeval tv1, tv2;
	int pos = 0;
	int x, y, w, h, bx, by, ix, iy;
	uint16_t *dst;

	gettimeofday(&tv1, NULL);
	fprintf(stderr, "GrabExposure enter: %ld msec from exposure_start\n", tv_diff(&tv1, &exposure_start));

	/* grab to local buffer first */
	if (bufsize != p_size * total_p) {
		free(buffer);
		bufsize = p_size * total_p;
		buffer = (uint16_t *) malloc(bufsize);
	}

	fprintf(stderr, "expecting: p_size %d, total_p %d, bufsize %zd\n",
		p_size, total_p, bufsize);

	if (bulk_transfer_read(QHY9_DATA_BULK_EP, (uint8_t *) buffer, p_size, total_p, &pos))
		return false;

	fprintf(stderr, "transferred\n");

	x  = PrimaryCCD.getSubX();
	y  = PrimaryCCD.getSubY();
	w  = PrimaryCCD.getSubW();
	h  = PrimaryCCD.getSubH();
	bx = PrimaryCCD.getBinX();
	by = PrimaryCCD.getBinY();

	fprintf(stderr, "x %d, y %d, w %d, h %d, bx %d, by %d\n",
		x, y, w, h, bx, by);

	PrimaryCCD.setFrameBufferSize(w / bx * h / by * 2);
	dst = (uint16_t *) PrimaryCCD.getFrameBuffer();
	for (iy = 0; iy < h / by; iy++) {
		for (ix = x / bx; ix < (x + w) / bx; ix++) {
			*dst++ = buffer[iy * LineSize + ix];
		}
	}

	gettimeofday(&tv2, NULL);
	fprintf(stderr, "GrabExposure: readout took %ld msec\n", tv_diff(&tv2, &tv1));

	setShutter(SHUTTER_FREE);

	ExposureComplete(&PrimaryCCD);
	InExposure = false;

	return true;
}


double QHY9::mv_to_degrees(double mv)
{
	double V = 1.024 * mv;
	double R, T, LNR;

	R = 33 / (V/1000 + 1.625) - 10;
	R = clamp_double(R, 1, 400);

	LNR = log(R);

	T= 1 / ( 0.002679+0.000291*LNR + LNR*LNR*LNR*4.28e-7  );

        T -= 273.15;

	return T;
}

double QHY9::degrees_to_mv(double degrees)
{
	double V, R, T;
	double x, y;
	double A=0.002679;
	double B=0.000291;
	double C=4.28e-7;

#define SQR3(x) ((x)*(x)*(x))
#define SQRT3(x) (exp(log(x)/3))

	T = 273.15 + clamp_double(degrees, -50, 50);

	y = (A - 1/T) / C;
	x = sqrt( SQR3(B/(3*C)) + (y*y)/4 );
	R = exp(SQRT3(x-y/2) - SQRT3(x+y/2));

	V = 33000/(R+10) - 1625;

	return V;
}


int QHY9::getDC201Interrupt()
{
	unsigned char buffer[4] = { 0, 0, 0, 0 };
	int transferred;

	if (!usb_handle)
		return 0;

	/* FIXME: A bulk transfer works, an interrupt_transfer doesn't ?!?! */
	libusb_bulk_transfer(usb_handle, QHY9_INTERRUPT_READ_EP, buffer, 4, &transferred, 0);

	//fprintf(stderr, "inte: %02x %02x %02x %02x\n", buffer[0], buffer[1], buffer[2], buffer[3]);

	return ((int16_t) (buffer[1] * 256 + buffer[2]));
}


void QHY9::setDC201Interrupt(uint8_t PWM, uint8_t FAN)
{
	uint8_t buffer[3];
	int transferred;

	buffer[0] = 0x01;
	buffer[1] = PWM;
	buffer[2] = FAN;

	if (!usb_handle)
		return;

	/* FIXME: A bulk transfer works, an interrupt_transfer doesn't ?!?! */
	libusb_bulk_transfer(usb_handle, QHY9_INTERRUPT_WRITE_EP, buffer, 3, &transferred, 0);
}

void QHY9::setCameraRegisters()
{
	uint8_t REG[64];
	unsigned long T;
	uint8_t time_L, time_M, time_H;
	int binx;

	/* Compute frame sizes, skips, number of patches, etc according to binning. wth is a "patch" ? */
	binx = PrimaryCCD.getBinX();
	switch (binx) {

	case 0:
	case 1:
		HBIN = 1;
		VBIN = 1;
		//LineSize = PrimaryCCD.getSubW();
		//VerticalSize = PrimaryCCD.getSubH();
		LineSize = 3584;
		VerticalSize = 2574;
		p_size = 3584 * 2; // must be multiple of 512
		break;

	case 2:
		HBIN = 2;
		VBIN = 2;
		//LineSize = PrimaryCCD.getSubW() / binx;
		//VerticalSize = PrimaryCCD.getSubH() / binx;
		LineSize = 1792;
		VerticalSize = 1287;
		p_size = 3584 * 2; // multiple of 512
		break;

	case 3:
		HBIN = 3;
		VBIN = 3;
		//LineSize = PrimaryCCD.getSubW() / binx;
		//VerticalSize = PrimaryCCD.getSubH() / binx;
		LineSize = 1194;               // was 1196, bad
		VerticalSize = 858;
		p_size = 1024;
		break;

	case 4:
		HBIN = 4;
		VBIN = 4;
		//LineSize = PrimaryCCD.getSubW() / binx;
		//VerticalSize = PrimaryCCD.getSubH() / binx;
		LineSize = 896;
		VerticalSize = 644;
		p_size = 1024;
		break;
	}

	SKIP_TOP = PrimaryCCD.getSubY() / PrimaryCCD.getBinY();
	SKIP_BOTTOM = VerticalSize - SKIP_TOP - PrimaryCCD.getSubH() / PrimaryCCD.getBinY();
	VerticalSize = VerticalSize - SKIP_TOP - SKIP_BOTTOM;

	fprintf(stderr, "SKIP_TOP %d, SKIP_BOTTOM %d, VerticalSize %d\n",
		SKIP_TOP, SKIP_BOTTOM, VerticalSize);

	T = (LineSize * VerticalSize + TopSkipPix) * 2;

	if (T % p_size) {
		total_p = T / p_size + 1;
		patchnum = (total_p * p_size - T) / 2 + 16;
	} else {
		total_p = T / p_size;
		patchnum = 16;
	}

	fprintf(stderr, "linesize=%d, vertsize=%d, T=%lu, p_size=%d, total_p=%d, patchnum=%d\n",
		LineSize, VerticalSize, T, p_size, total_p, patchnum);

	/* 1 = disable AMP during exposure */
	AMPVOLTAGE = 1;

	/* manual shutter for darks and biases */
	CCDChip::CCD_FRAME ft = PrimaryCCD.getFrameType();
	MechanicalShutterMode = (ft == CCDChip::DARK_FRAME || ft == CCDChip::BIAS_FRAME) ? 1 : 0;

	TopSkipNull = 30; // ???

	SDRAM_MAXSIZE = 100;

	/* fill in register buffer */
	memset(REG, 0, 64);

	time_L = Exptime % 256;
	time_M = (Exptime - time_L)/256;
	time_H = (Exptime - time_L - time_M * 256) / 65536;

	REG[0]=Gain;
	REG[1]=Offset;

	REG[2]=time_H;
	REG[3]=time_M;
	REG[4]=time_L;

	REG[5]=HBIN;
	REG[6]=VBIN;

	REG[7]=MSB(LineSize);
	REG[8]=LSB(LineSize);

	REG[9]= MSB(VerticalSize);
	REG[10]=LSB(VerticalSize);

	REG[11]=MSB(SKIP_TOP);
	REG[12]=LSB(SKIP_TOP);

	REG[13]=MSB(SKIP_BOTTOM);
	REG[14]=LSB(SKIP_BOTTOM);

	REG[15]=MSB(LiveVideo_BeginLine);
	REG[16]=LSB(LiveVideo_BeginLine);

	REG[17]=MSB(patchnum);
	REG[18]=LSB(patchnum);

	REG[19]=MSB(AnitInterlace);
	REG[20]=LSB(AnitInterlace);

	REG[22]=MultiFieldBIN;

	REG[29]=MSB(ClockADJ);
	REG[30]=LSB(ClockADJ);

	REG[32]=AMPVOLTAGE;

	REG[33]=DownloadSpeed;

	REG[35]=TgateMode;
	REG[36]=ShortExposure;
	REG[37]=VSUB;
	REG[38]=CLAMP;

	REG[42]=TransferBIT;

	REG[46]=TopSkipNull;

	REG[47]=MSB(TopSkipPix);
	REG[48]=LSB(TopSkipPix);

	REG[51]=MechanicalShutterMode ;
	REG[52]=DownloadCloseTEC ;

	REG[53]=(WindowHeater&~0xf0)*16+(MotorHeating&~0xf0);

	REG[58]=SDRAM_MAXSIZE ;
	REG[63]=Trig ;

	int i;
	fprintf(stderr, "Sending REGS...\n");
	for (i = 0; i < 64; i++) {
		if (i % 16 == 0) {
			fprintf(stderr, "\n%02d: ", i);
		}

		fprintf(stderr, "%02x ", REG[i]);
	}
	fprintf(stderr, "\n");

	if (!usb_handle)
		return;

	libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
				QHY9_REGISTERS_CMD, 0, 0, REG, 64, 0);
}

bool QHY9::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
#if 0
	double v;
	INumber *np;

	fprintf(stderr, "ISNEWNUMBER dev %s, device %s, name %s\n\n", dev, getDeviceName(), name);
	if (dev && !strcmp(dev, getDeviceName())) {
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

		if (!strcmp(name, GainOffsetSetNV->name)) {
			if (n < 2)
				return false;

			Gain   = GainOffsetN[0].value = clamp_int(values[0], 0, 255);
			Offset = GainOffsetN[1].value = clamp_int(values[1], 0, 255);

			GainOffsetSetNV->s = IPS_OK;
			IDSetNumber(GainOffsetSetNV, NULL);

			return true;
		}
	}
#endif

	if (dev && !strcmp(dev, getDeviceName())) {
		if (!strcmp(name, FilterSlotNP.name)) {
			processFilterSlot(dev, values, names);
			return true;
		}
	}

	return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

bool QHY9::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	if (dev && !strcmp(dev, getDeviceName())) {
		if (!strcmp(name, ReadOutSP.name)) {
			if (IUUpdateSwitch(&ReadOutSP, states, names, n) < 0)
				return false;

			if (ReadOutS[0].s == ISS_ON)
				DownloadSpeed = 0;

			if (ReadOutS[1].s == ISS_ON)
				DownloadSpeed = 1;

			if (ReadOutS[2].s == ISS_ON)
				DownloadSpeed = 2;

                        ReadOutSP.s = IPS_OK;
			IDSetSwitch(&ReadOutSP, NULL);

			IDMessage(dev, "readout speed %d", DownloadSpeed);

			return true;
		}
        }

	return CCD::ISNewSwitch(dev, name, states, names, n);
}

bool QHY9::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	if (dev && !strcmp(dev, getDeviceName())) {
		if (!strcmp(name, FilterNameTP->name)) {
			processFilterName(dev, texts, names, n);
			return true;
		}
	}

	return INDI::CCD::ISNewText(dev, name, texts, names, n);
}

bool QHY9::saveConfigItems(FILE *fp)
{
	INDI::CCD::saveConfigItems(fp);

	IUSaveConfigNumber(fp, &FilterSlotNP);
	IUSaveConfigText(fp, FilterNameTP);

	IUSaveConfigNumber(fp, &GainOffsetSetNV);
	IUSaveConfigSwitch(fp, &ReadOutSP);

	return true;
}


void QHY9::beginVideo()
{
	uint8_t buffer[1] = { 100 };

	if (!usb_handle)
		return;

	libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
				QHY9_BEGIN_VIDEO_CMD, 0, 0, buffer, 1, 0);
}

void QHY9::abortVideo()
{
	uint8_t buffer[1] = { 0xff };
	int transferred;

	if (!usb_handle)
		return;

	libusb_bulk_transfer(usb_handle, QHY9_INTERRUPT_WRITE_EP, buffer, 1, &transferred, 0);
}

void QHY9::setShutter(int mode)
{
	uint8_t buffer[1] = { (uint8_t) mode };

	if (!usb_handle)
		return;

	libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
				QHY9_SHUTTER_CMD, 0, 0, buffer, 1, 0);
}

bool QHY9::SelectFilter(int slot)
{
	uint8_t buffer[2];

	slot = clamp_int(slot - 1, 0, 4);

	buffer[0] = 0x5A;
	buffer[1] = slot;

	if (usb_handle) {
		libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
					QHY9_CFW_CMD, 0, 0, buffer, 2, 0);
	}

	CurrentFilter = slot + 1;

	SelectFilterDone(CurrentFilter);

	return true;
}


int QHY9::QueryFilter()
{
	return CurrentFilter;
}

/* hw can't save filter names */
bool QHY9::SetFilterNames()
{
	saveConfig();
	return true;
}

bool QHY9::GetFilterNames(const char *groupName)
{
	char filterName[MAXINDINAME];
	char filterLabel[MAXINDILABEL];
	char filterBand[MAXINDILABEL];
	int MaxFilter = FilterSlotN[0].max;
	int i;

	if (FilterNameT != NULL)
		delete FilterNameT;

	FilterNameT = new IText[MaxFilter];

	for (i = 0; i < MaxFilter; i++) {
		snprintf(filterName, MAXINDINAME, "FILTER_SLOT_NAME_%d", i+1);
		snprintf(filterLabel, MAXINDILABEL, "Filter #%d", i+1);
		snprintf(filterBand, MAXINDILABEL, "%d", i+1);
		IUFillText(&FilterNameT[i], filterName, filterLabel, filterBand);
	}

	IUFillTextVector(FilterNameTP, FilterNameT, MaxFilter, getDeviceName(), "FILTER_NAME", "Filter",
			 groupName, IP_RW, 0, IPS_IDLE);

	return true;
}

/* FIXME: This needs some TLC, this "regulator" oscillates and swings +/- 1.5 deg */
void QHY9::TempControlTimer()
{
	static bool alternate = false;	     // first time, read
	static double voltage = 0.0;
	static int counter = 0;
	static int divider = 0;

	if (++divider > 1) {
		divider = 0;
		return;
	}


	alternate = !alternate;

	if (alternate) {
		voltage = getDC201Interrupt();
		Temperature = mv_to_degrees(1.024 * voltage);
	} else {
		if (Temperature > TemperatureTarget + 5)
			TEC_PWM += 5;
		else if (Temperature < TemperatureTarget - 5)
			TEC_PWM -= 5;
		else if (Temperature > TemperatureTarget + 0.7)
			TEC_PWM += 1;
		else if (Temperature < TemperatureTarget - 0.7)
			TEC_PWM -= 1;

		TEC_PWM = clamp_int(TEC_PWM, 0, TEC_PWMLimit * 256 / 100);

		setDC201Interrupt(TEC_PWM, 255);

		TemperatureN[2].value = Temperature;
		TemperatureN[3].value = TEC_PWM * 100 / 256;

		TemperatureGetNV.s = IPS_OK;

		counter++;
		if ((counter >= 2) && isConnected()) {
			IDSetNumber(&TemperatureGetNV, NULL);
			counter = 0;
		}

		//fprintf(stderr, "volt %.2f, temp %.2f, target %.2f, PWM %d\n",
		//	voltage, Temperature, TemperatureTarget, TEC_PWM);
	}
}

int QHY9::bulk_transfer_read(int ep, unsigned char *data, int psize, int pnum, int *pos)
{
	int ret, length_transfered;
        int i;

	if (!usb_handle)
		return -1;

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



void QHY9::addFITSKeywords(fitsfile *fptr, CCDChip *chip)
{
#if 0
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
#endif
}
