#include "gui.h"
#include "utils.h"


// global variables
IplImage* mwFLIRImage = 0;	// stores the image read from the FLIR sensor
pthread_mutex_t mutex_FLIRImage; // controls the concurrent access to FLIR data
IplImage* mwImage = 0;		// image used to show/scale FLIR image on the main window
int mwHeight = 100;			// main window heigth
int mwWidth = 100;			// main window width
int app_exit = 0;			// controls when application is terminated


void buildGUI() {
}

void mySetRGB(IplImage* img, int x, int y, char red, char green, char blue) {
	int height = img->height;
	int width = img->width;
	int step = img->widthStep/sizeof(uchar);
	int channels = img->nChannels;
	uchar* data = (uchar *)mwFLIRImage->imageData;

	data[y*step+x*channels] = blue;
	data[y*step+x*channels+1] = green;
	data[y*step+x*channels+2] = red;
}



void createFLIRImage() {
	mwWidth = 80;
	mwHeight = 60;
	for(int y=mwHeight-1; y >= 0; y--)
		for(int x=mwWidth-1; x >= 0; x--) {
		}
}
