#ifndef OCV_GUI
#define OCV_GUI

//#include <cv.h>
#include "/opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <highgui.h>
#include <pthread.h>


#include "RGBImageTemplate.h"

// global variables
extern IplImage* mwFLIRImage;	// stores the image read from the FLIR sensor
extern pthread_mutex_t mutex_FLIRImage; // controls the concurrent access to FLIR data
extern IplImage* mwImage;		// image used to show/scale FLIR image on the main window
extern int mwHeight;			// main window heigth
extern int mwWidth;				// main window width
extern int app_exit;			// controls when application is terminated

// windows names
#define W_MAIN "mainWin"

extern void mySetRGB(IplImage* img, int x, int y, char red, char green, char blue);
extern void loadImage(char * file_name);
extern void createFLIRImage();


#endif
