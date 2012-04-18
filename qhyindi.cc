
#include "qhyccd.h"

using namespace std;

static QHYCCD *camera = NULL;

bool initialize()
{
	static bool initialized = false;

	if (initialized)
		return initialized;

	if ((camera = QHYCCD::detectCamera()) == NULL)
		return initialized;

	initialized = true;
	return initialized;
}


void ISGetProperties(const char *dev)
{
	if (!initialize())
		return;

	if (dev && strcmp(dev, camera->deviceName()))
		return;

	camera->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
	if (!initialize())
		return;

	if (dev && strcmp(dev, camera->deviceName()))
		return;

	camera->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	if (!initialize())
		return;

	if (dev && strcmp(dev, camera->deviceName()))
		return;

	camera->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	if (!initialize())
		return;

	if (dev && strcmp(dev, camera->deviceName()))
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
