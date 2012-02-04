
#include "qhyccd.h"

using namespace std;

QHYCCD *camera = NULL;

class ErrInitialize { };

void initialize()
{
	static bool initialized = false;

	if (initialized)
		return;

	if ((camera = QHYCCD::detectCamera()) == NULL)
		throw ErrInitialize();

	initialized = true;
}


void ISGetProperties(const char *dev)
{
	try {
		initialize();

		if (dev && strcmp(dev, camera->deviceName()))
			return;

		camera->ISGetProperties(dev);
	}
	catch (ErrInitialize) {
	}
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	try {
		initialize();

		if (dev && strcmp(dev, camera->deviceName()))
			return;

		camera->ISNewSwitch(dev, name, states, names, n);
	} catch (ErrInitialize) {
	}
}

void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	try {
		initialize();

		if (dev && strcmp(dev, camera->deviceName()))
			return;

		camera->ISNewText(dev, name, texts, names, n);
	} catch (ErrInitialize) {
	}
}

void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	try {
		initialize();

		if (dev && strcmp(dev, camera->deviceName()))
			return;

		camera->ISNewNumber(dev, name, values, names, n);
	} catch (ErrInitialize) {
	}
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
