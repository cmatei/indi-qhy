
#include "qhyccd.h"
#include "qhy9.h"

using namespace std;

void QHY9::initCamera()
{
	HasTemperatureControl = true;
	HasFilterWheel = true;
	HasGuideHead = false;

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

	MinFilter = 1;
	MaxFilter = 5;
	CurrentFilter = 1;
	TargetFilter = 1;

	// default to slowest readout
	DownloadSpeed = 2;
}

bool QHY9::initProperties()
{
	QHYCCD::initProperties();

	/* Readout speed */
	IUFillSwitch(&ReadOutS[0], "READOUT_FAST",   "Fast",   (DownloadSpeed == 0) ? ISS_ON : ISS_OFF);
	IUFillSwitch(&ReadOutS[1], "READOUT_NORMAL", "Normal", (DownloadSpeed == 1) ? ISS_ON : ISS_OFF);
	IUFillSwitch(&ReadOutS[2], "READOUT_SLOW",   "Slow",   (DownloadSpeed == 2) ? ISS_ON : ISS_OFF);

	IUFillSwitchVector(ReadOutSP, ReadOutS, 3, getDeviceName(),
			   "READOUT_SPEED", "Readout Speed",
			   IMAGE_SETTINGS_TAB, IP_WO, ISR_1OFMANY, 0, IPS_IDLE);

	GetFilterNames(FILTER_TAB);

	return true;
}

bool QHY9::updateProperties()
{
	QHYCCD::updateProperties();

	SetCCDParams(QHY9_SENSOR_WIDTH, QHY9_SENSOR_HEIGHT, 16, 5.4, 5.4);

	if (isConnected()) {
		defineSwitch(ReadOutSP);

		if (FilterNameT != NULL) {
			defineText(FilterNameTP);
			defineNumber(&FilterSlotNP);
		}

	}

	return true;
}

int QHY9::StartExposure(float duration)
{
	CCDChip::CCD_FRAME type;

	if (InExposure)
		return -1;

	type = PrimaryCCD.getFrameType();

	/* 0 sec exposures are not done for anything but BIAS.
	   This is useful for the load/save config feature of INDI drivers,
	   so they won't trigger an exposure when loading config */
	if (type != CCDChip::BIAS_FRAME && duration == 0.0)
		return 1;

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

	// 0 - exp. running on timers, 1 short exposures already done,  -1 error
	return 0;
}

bool QHY9::AbortExposure()
{
	if (!InExposure)
		return true;

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
#if 0
	PrimaryCCD.setFrameBufferSize(p_size * total_p);
	if (bulk_transfer_read(QHY9_DATA_BULK_EP, (uint8_t *) PrimaryCCD.getFrameBuffer(), p_size, total_p, &pos))
		return false;
#else
	if (bulk_transfer_read(QHY9_DATA_BULK_EP, (uint8_t *) buffer, p_size, total_p, &pos))
		return false;

	fprintf(stderr, "transferred\n");

	x  = PrimaryCCD.getSubX();
	y  = PrimaryCCD.getSubY();
	w  = PrimaryCCD.getSubW();
	h  = PrimaryCCD.getSubH();
	bx = PrimaryCCD.getBinX();
	by = PrimaryCCD.getBinY();

	PrimaryCCD.setFrameBufferSize(w / bx * h / by * 2);
	dst = (uint16_t *) PrimaryCCD.getFrameBuffer();
	for (iy = 0; iy < h / by; iy++) {
		for (ix = x / bx; ix < (x + w) / bx; ix++) {
			*dst++ = buffer[iy * LineSize + ix];
		}
	}
#endif

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

	libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
				QHY9_REGISTERS_CMD, 0, 0, REG, 64, 0);
}

bool QHY9::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	if (dev && !strcmp(dev, getDeviceName())) {
		if (!strcmp(name, ReadOutSP->name)) {
			if (IUUpdateSwitch(ReadOutSP, states, names, n) < 0)
				return false;

			if (ReadOutS[0].s == ISS_ON)
				DownloadSpeed = 0;

			if (ReadOutS[1].s == ISS_ON)
				DownloadSpeed = 1;

			if (ReadOutS[2].s == ISS_ON)
				DownloadSpeed = 2;

                        ReadOutSP->s = IPS_OK;
			IDSetSwitch(ReadOutSP, NULL);

			IDMessage(dev, "readout speed %d", DownloadSpeed);

			return true;
		}
        }

	return QHYCCD::ISNewSwitch(dev, name, states, names, n);
}

void QHY9::beginVideo()
{
	uint8_t buffer[1] = { 100 };

	libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
				QHY9_BEGIN_VIDEO_CMD, 0, 0, buffer, 1, 0);
}

void QHY9::abortVideo()
{
	uint8_t buffer[1] = { 0xff };
	int transferred;

	libusb_bulk_transfer(usb_handle, QHY9_INTERRUPT_WRITE_EP, buffer, 1, &transferred, 0);
}

void QHY9::setShutter(int mode)
{
	uint8_t buffer[1] = { (uint8_t) mode };

	libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
				QHY9_SHUTTER_CMD, 0, 0, buffer, 1, 0);
}

bool QHY9::SetFilterNames()
{
	return true;
}

bool QHY9::SelectFilter(int slot)
{
	uint8_t buffer[2];

	slot = clamp_int(slot - 1, 0, 4);

	buffer[0] = 0x5A;
	buffer[1] = slot;

	libusb_control_transfer(usb_handle, QHY9_VENDOR_REQUEST_WRITE,
				QHY9_CFW_CMD, 0, 0, buffer, 2, 0);

	CurrentFilter = slot + 1;

	SelectFilterDone(CurrentFilter);

	return true;
}

int QHY9::QueryFilter()
{
	return CurrentFilter;
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

		TemperatureGetNV->s = IPS_OK;

		counter++;
		if ((counter >= 2) && isConnected()) {
			IDSetNumber(TemperatureGetNV, NULL);
			counter = 0;
		}

		//fprintf(stderr, "volt %.2f, temp %.2f, target %.2f, PWM %d\n",
		//	voltage, Temperature, TemperatureTarget, TEC_PWM);
	}
}
