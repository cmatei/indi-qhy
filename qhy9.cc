
#include "qhyccd.h"
#include "qhy9.h"

using namespace std;

void QHY9::initDefaults()
{
	QHYCCD::initDefaults();

	HasGuideHead = false;
	HasTemperatureControl = true;
	HasColorFilterWheel = true;

	XRes = QHY9_SENSOR_WIDTH;
	YRes = QHY9_SENSOR_HEIGHT;

	SubX = 0;
	SubY = 0;
	SubW = XRes;
	SubH = YRes;

	BinX = 1;
	BinY = 1;

	PixelSizex = 5.4;
	PixelSizey = 5.4;

	TemperatureTarget = 50.0;
	CFWSlot = 1;

	/* QHY9 specific */
	Gain = 20;
	Offset = 120;
	MechanicalShutterMode = 0;
	DownloadCloseTEC = 0;
	SDRAM_MAXSIZE = 100;
}

bool QHY9::initProperties()
{
	QHYCCD::initProperties();

	return true;
}

bool QHY9::updateProperties()
{
	QHYCCD::updateProperties();
	return true;
}


#if 0

bool QHY9::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	return QHYCCD::ISNewSwitch(dev, name, states, names, n);
}


bool QHY9::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
	return QHYCCD::ISNewNumber(dev, name, values, names, n);
}

bool QHY9::ISNewText  (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	return QHYCCD::ISNewText(dev, name, texts, names, n);
}

#endif

int QHY9::StartExposure(float duration)
{
	if (exposing)
		return -1;

	Exptime = duration * 1000;

	setCameraRegisters();
	usleep(10000);

	if (FrameType == FRAME_TYPE_DARK || FrameType == FRAME_TYPE_BIAS) {
		fprintf(stderr, "SHOOTING A DARK, CLOSING SHUTTER\n");
		setShutter(SHUTTER_CLOSE);
		usleep(500*1000);		     // shutter speed is 1/10 to 1/2 sec
	}

	exposing = true;
	gettimeofday(&exposure_start, NULL);

	beginVideo();

	// 0 - exp. running on timers, 1 short exposures already done,  -1 error
	return 0;
}


bool QHY9::ExposureComplete()
{
	static uint8_t data[QHY9_SENSOR_HEIGHT * QHY9_SENSOR_WIDTH * 2];
	int pos;
	void *memptr;
	size_t memsize;
	int status=0;
	long naxes[2];
	long naxis=2;

	fitsfile *fptr=NULL;

	if (bulk_transfer_read(QHY9_DATA_BULK_EP, data, p_size, total_p, &pos))
		return false;


	setShutter(SHUTTER_FREE);

	fprintf(stderr, "GOT DATA!\n");

	naxes[0]=LineSize;
	naxes[1]=VerticalSize;

	//  Now we have to send fits format data to the client
	memsize=2880;
	memptr=malloc(memsize);
	fits_create_memfile(&fptr,&memptr,&memsize,2880,realloc,&status);
	if(status) {
                IDLog("Error: Failed to create FITS image\n");
                fits_report_error(stderr, status);  /* print out any error messages */
                return false;
	}
        fits_create_img(fptr, USHORT_IMG , naxis, naxes, &status);
        if (status)
        {
                IDLog("Error: Failed to create FITS image\n");
                fits_report_error(stderr, status);  /* print out any error messages */
                return false;
        }

	fits_write_img(fptr, TUSHORT, 1, LineSize * VerticalSize, data, &status);
	if (status)
        {
                IDLog("Error: Failed to write FITS image\n");
                fits_report_error(stderr, status);  /* print out any error messages */
                return false;
        }
        fits_close_file(fptr,&status);
        //IDLog("Built the fits file\n");

	ImageExposureNV->s=IPS_OK;
	IDSetNumber(ImageExposureNV, NULL);

	uploadfile(memptr,memsize);
	free(memptr);
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

#if 0
int QHY9::getDC201()
{
	unsigned char buffer[4] = { 0, 0, 0, 0 };

	libusb_control_transfer(usb_handle, 0xC0, 0xC5, 0, 0, buffer, 4, 0);
	//fprintf(stderr, "vend: %02x %02x %02x %02x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
	return ((int16_t) (buffer[1] * 256 + buffer[2]));
}

void QHY9::setDC201(uint8_t PWM, uint8_t FAN)
{
	uint8_t buffer[2];

	buffer[0] = PWM;
	buffer[1] = 0xFF;

	libusb_control_transfer(usb_handle, 0x40, 0xc6, 0, 0, buffer, 2, 0);
}
#endif


int QHY9::getDC201Interrupt()
{
	unsigned char buffer[4] = { 0, 0, 0, 0 };
	int transferred;

	/* FIXME: WTF ?! A bulk transfer works, an interrupt_transfer doesn't ?!?! */
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

	/* FIXME: WTF ?! A bulk transfer works, an interrupt_transfer doesn't ?!?! */
	libusb_bulk_transfer(usb_handle, QHY9_INTERRUPT_WRITE_EP, buffer, 3, &transferred, 0);

	//fprintf(stderr, "setdc201: write %d, transferred %d\n", r, transferred);
}

void QHY9::setCameraRegisters()
{
	uint8_t REG[64];
	unsigned long T;
	uint8_t time_L, time_M, time_H;

	/* Compute frame sizes, skips, number of patches, etc according to binning. wth is a "patch" ? */
	switch (BinX) {

	case 0:
	case 1:
		HBIN = 1;
		VBIN = 1;
		LineSize = 3584;
		VerticalSize = 2574;
		p_size = 3584 * 2; // must be multiple of 512 ?
		break;

	case 2:
		HBIN = 2;
		VBIN = 2;
		LineSize = 1792;
		VerticalSize = 1287;
		p_size = 3584 * 2; // multiple of 512
		break;

	case 3:
		HBIN = 3;
		VBIN = 3;
		LineSize = 1196;
		VerticalSize = 858;
		p_size = 1024;
		break;

	case 4:
		HBIN = 4;
		VBIN = 4;
		LineSize = 896;
		VerticalSize = 644;
		p_size = 1024;
		break;
	}

	T = (LineSize * VerticalSize + TopSkipPix) * 2;

	if (T % p_size) {
		total_p = T / p_size + 1;
		patchnum = (total_p * p_size - T) / 2 + 16;
	} else {
		total_p = T / p_size;
		patchnum = 16;
	}

	/* FIXME */
	SKIP_TOP = 0;
	SKIP_BOTTOM = 0;

	AMPVOLTAGE = 1;

	// slowest. 0 - normal and 1 - fast
	DownloadSpeed = 2;

	/* manual shutter for darks and biases */
	MechanicalShutterMode = (FrameType == FRAME_TYPE_DARK || FrameType == FRAME_TYPE_BIAS) ? 1 : 0;

	TopSkipNull = 30; // ???

	SDRAM_MAXSIZE = 100;

	// CLAMP ?!
	CLAMP = 0; // 1 also

	/* fill in register buffer */
	memset(REG, 0, 64);

	time_L = Exptime % 256;
	time_M = (Exptime - time_L)/256;
	time_H = (Exptime - time_L - time_M * 256) / 65536;

	REG[0]=Gain ;
	REG[1]=Offset ;

	REG[2]=time_H;
	REG[3]=time_M;
	REG[4]=time_L;

	REG[5]=HBIN ;
	REG[6]=VBIN ;

	REG[7]=MSB(LineSize );
	REG[8]=LSB(LineSize );

	REG[9]= MSB(VerticalSize );
	REG[10]=LSB(VerticalSize );

	REG[11]=MSB(SKIP_TOP );
	REG[12]=LSB(SKIP_TOP );

	REG[13]=MSB(SKIP_BOTTOM );
	REG[14]=LSB(SKIP_BOTTOM );

	REG[15]=MSB(LiveVideo_BeginLine );
	REG[16]=LSB(LiveVideo_BeginLine );

	REG[17]=MSB(patchnum);
	REG[18]=LSB(patchnum);

	REG[19]=MSB(AnitInterlace );
	REG[20]=LSB(AnitInterlace );

	REG[22]=MultiFieldBIN ;

	REG[29]=MSB(ClockADJ );
	REG[30]=LSB(ClockADJ );

	REG[32]=AMPVOLTAGE ;

	REG[33]=DownloadSpeed ;

	REG[35]=TgateMode ;
	REG[36]=ShortExposure ;
	REG[37]=VSUB ;
	REG[38]=CLAMP;

	REG[42]=TransferBIT ;

	REG[46]=TopSkipNull ;

	REG[47]=MSB(TopSkipPix );
	REG[48]=LSB(TopSkipPix );

	REG[51]=MechanicalShutterMode ;
	REG[52]=DownloadCloseTEC ;

	REG[53]=(WindowHeater&~0xf0)*16+(MotorHeating&~0xf0);

	REG[58]=SDRAM_MAXSIZE ;
	REG[63]=Trig ;

	vendor_request_write(QHY9_REGISTERS_CMD, REG, 64);
}

void QHY9::beginVideo()
{
	uint8_t buffer[1] = { 100 };
	vendor_request_write(QHY9_BEGIN_VIDEO_CMD, buffer, 1);
}

void QHY9::abortVideo()
{
}

void QHY9::setShutter(int mode)
{
	uint8_t buffer[1] = { mode };
	vendor_request_write(QHY9_SHUTTER_CMD, buffer, 1);
}

void QHY9::SetCFWSlot(int slot)
{
	uint8_t buffer[2];

	buffer[0] = 0x5A;
	buffer[1] = clamp_int(slot, 0, 4);

	vendor_request_write(QHY9_CFW_CMD, buffer, 2);

	fprintf(stderr, "FILTER: slot %d\n\n\n\n", buffer[1]);
}

/* FIXME: This needs some TLC, this "regulator" oscillates and swings +/- 1.5 deg */
void QHY9::TempControlTimer()
{
	static bool alternate = false;	     // first time, read
	static double voltage = 0.0;
	static int counter = 0;

	alternate = !alternate;

	if (alternate) {
		voltage = getDC201Interrupt();
		Temperature = mv_to_degrees(1.024 * voltage);
	} else {

		if (voltage > degrees_to_mv(TemperatureTarget + 5))
			TEC_PWM = TEC_PWM + 5;
		else if (voltage > degrees_to_mv(TemperatureTarget + 1))
			TEC_PWM = TEC_PWM + 1;

		if (voltage < degrees_to_mv(TemperatureTarget - 5))
			TEC_PWM = TEC_PWM - 5;
		else if (voltage < degrees_to_mv(TemperatureTarget - 1))
			TEC_PWM = TEC_PWM - 1;

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

#if 1
		fprintf(stderr, "volt %.2f, PWM %d, t + 5 %.2f, t + 1 %.2f, t + 0.2 %.2f, t - 5 %.2f, t - 1 %.2f, t - 0.2 %.2f\n",
			voltage, TEC_PWM,
			degrees_to_mv(TemperatureTarget + 5), degrees_to_mv(TemperatureTarget + 1), degrees_to_mv(TemperatureTarget + 0.2),
			degrees_to_mv(TemperatureTarget - 5), degrees_to_mv(TemperatureTarget - 1), degrees_to_mv(TemperatureTarget - 0.2));
#endif

	}
}



