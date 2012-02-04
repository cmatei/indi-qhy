
#include "qhyccd.h"
#include "qhy5.h"

using namespace std;

static const int gain_map[] = {
    0x000,0x004,0x005,0x006,0x007,0x008,0x009,0x00A,0x00B,0x00C,
    0x00D,0x00E,0x00F,0x010,0x011,0x012,0x013,0x014,0x015,0x016,
    0x017,0x018,0x019,0x01A,0x01B,0x01C,0x01D,0x01E,0x01F,0x051,
    0x052,0x053,0x054,0x055,0x056,0x057,0x058,0x059,0x05A,0x05B,
    0x05C,0x05D,0x05E,0x05F,0x6CE,0x6CF,0x6D0,0x6D1,0x6D2,0x6D3,
    0x6D4,0x6D5,0x6D6,0x6D7,0x6D8,0x6D9,0x6DA,0x6DB,0x6DC,0x6DD,
    0x6DE,0x6DF,0x6E0,0x6E1,0x6E2,0x6E3,0x6E4,0x6E5,0x6E6,0x6E7,
    0x6FC,0x6FD,0x6FE,0x6FF
};

static const int gain_map_size = (sizeof(gain_map) / sizeof(int));


void QHY5::initDefaults()
{
	QHYCCD::initDefaults();

	HasGuideHead = false;
	HasTemperatureControl = false;
	HasColorFilterWheel = false;
	HasSt4Port = true;

	PrimaryCCD.setResolution(QHY5_SENSOR_WIDTH, QHY5_SENSOR_HEIGHT);
	PrimaryCCD.setFrame(0, 0, QHY5_SENSOR_WIDTH, QHY5_SENSOR_HEIGHT);
	PrimaryCCD.setBin(1, 1);
	PrimaryCCD.setPixelSize(5.2, 5.2);
	PrimaryCCD.setFrameType(CCDChip::LIGHT_FRAME);

	/* QHY5 specific */
//	Gain = 20;
//	Offset = 120;
//	MechanicalShutterMode = 0;
//	DownloadCloseTEC = 1;
//	SDRAM_MAXSIZE = 100;
}


int QHY5::StartExposure(float duration)
{
	if (exposing)
		return -1;

	Exptime = duration * 1000;

#if 0

	setCameraRegisters();
	usleep(100000);

	gettimeofday(&exposure_start, NULL);
	exposing = true;

	beginVideo();
#endif
	// 0 - exp. running on timers, 1 short exposures already done,  -1 error
	return 0;
}

void QHY5::beginVideo()
{

}

void QHY5::abortVideo()
{
}


void QHY5::setCameraRegisters()
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

	/* 1 = disable AMP during exposure */
	AMPVOLTAGE = 1;

	// slowest. 0 - normal and 1 - fast
	DownloadSpeed = 2;

	/* manual shutter for darks and biases */
	CCDChip::CCD_FRAME ft = PrimaryCCD.getFrameType();
	MechanicalShutterMode = (ft == CCDChip::DARK_FRAME || ft == CCDChip::BIAS_FRAME) ? 1 : 0;

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

	//	vendor_request_write(QHY5_REGISTERS_CMD, REG, 64);
}

