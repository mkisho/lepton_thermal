 #ifndef LEPTON_THREAD
 #define LEPTON_THREAD
 
 #include <pthread.h>
 
 #include "Palettes.h"
 #include "SPI.h"
 #include "Lepton_I2C.h"
 
 #define PACKET_SIZE 164
 #define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
 #define PACKETS_PER_FRAME 60
 #define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)
 #define FPS 27;
 
 #define FLIR_MAX_VALUE 65535
 #define FLIR_MIN_VALUE 0
 
 // compilation options

 // this indicates whether the max/min values should be unique
 // for the whole viode
 #define __SINGLE_MAX_MIN_VALUE 1	
 // this indicates whether the display thread should acquire
 // locks on both flir data and flir image on the same time, 
 // or these lock should be individual when each of these resources
 // is needed
 #define DISPLAY_MUTEX_FULL_LOCK 1
 
 // video-related variables
 #define VIDEO_WIDTH    (PACKET_SIZE_UINT16 - 2)
 #define VIDEO_HEIGHT   PACKETS_PER_FRAME
 #define VIDEO_FPS      27.0
 #define IS_VIDEO_COLOR 1
 #define VIDEO_FILE_NAME "./videos/flirVideo-00001.avi"
 
 
 extern uint8_t flirData[PACKET_SIZE*PACKETS_PER_FRAME];
 extern pthread_mutex_t mutex_flirData;
 extern uint16_t *flirFrameBuffer;
 extern uint16_t minValue;
 extern uint16_t maxValue;
 
 
 extern void *readImageFromFLIR_andDisplay(void *ptr);
 extern void *readDataFromFLIR(void *ptr);
 extern void *displayFLIRDataAsImage(void *ptr);
 extern void *writeFLIRVideo(void *ptr);
 
 #endif
