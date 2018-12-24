/*
 Copyright (c) 2014, Pure Engineering LLC
 All rights reserved.
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


/*
	The software from this repository was modified by mkisho.
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
//#define DEBUG

#include "gui.h"
#include "lepton-threads.h"

/*
 * IMPORTANT THINGS:
 * 
 * A Comprehensive Guide to Installing and Configuring OpenCV 2.4.2 on Ubuntu
 * http://www.ozbotz.org/opencv-installation/
 */


int main( int argc, char **argv )
{
	ros::init(argc, argv, "lepton_node");

	// initialize SPI
	int spi_status = SpiOpenPort(0);
	if (spi_status != 0) {
		printf("Problems with SPI ! [%d]", spi_status);
		exit(-1);
	}

	// creating the mutex to control the concurrent access to FLIRImage
	pthread_mutex_init(&mutex_FLIRImage, NULL);
	pthread_mutex_init(&mutex_flirData, NULL); 

	// creating the FLIR image
	//loadImage("maw.png");
	createFLIRImage(); 

	// creating GUI
//	buildGUI();
//
/*
	printf("\nCreating video writer...\n");
	printf("File Name=%s\nFPS=%.1f\nW=%d\nH=%d\nColor=%d\n", VIDEO_FILE_NAME,VIDEO_FPS,VIDEO_WIDTH,VIDEO_HEIGHT,IS_VIDEO_COLOR);
	videoWriter = cvCreateVideoWriter("maw_test0000.avi",//VIDEO_FILE_NAME, 
									  CV_FOURCC('H', '2', '6', '4'),
									  VIDEO_FPS,
									  cvSize(VIDEO_WIDTH, VIDEO_HEIGHT),
									  IS_VIDEO_COLOR);
	if (videoWriter)
		printf("\nVideo writer successfuly created.\n");
	else
		printf("\nPROBLEMS !!!\n");
*/


	//pthread_t t1;
	//pthread_create(&t1, NULL, readImageFromFLIR_andDisplay, NULL);

	// creating application threads
	pthread_t thread_readFLIR;
	pthread_create(&thread_readFLIR, NULL, readDataFromFLIR, NULL);

//	pthread_t thread_displayFLIRImage;
//	pthread_create(&thread_displayFLIRImage, NULL, displayFLIRDataAsImage, NULL);
	
//	pthread_t thread_writeFLIRVideo;
//	pthread_create(&thread_writeFLIRVideo, NULL, writeFLIRVideo, NULL);

	// infinite loop to avoid the program to finish
	app_exit = 0;
	while(!app_exit){
		char key = 1;
		// keyboard commands
		switch(key){
		case 27:
			app_exit = 1;
			continue;
		}
//		updateWindow();
	}
	
	// wait until all threads finish
#ifndef DEBUG
	printf("\n\nWaiting for thread 1 to complete...\n");
#endif
	pthread_join(thread_readFLIR, NULL);

	// Closing SPI port
	SpiClosePort(0);
	
	// Releasing allocated resources
	pthread_mutex_destroy(&mutex_FLIRImage);
	pthread_mutex_destroy(&mutex_flirData);
		
	printf("\nEnd !!\n\n\n");
	return 0;
}

