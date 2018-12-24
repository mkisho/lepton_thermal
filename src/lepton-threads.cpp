#include "lepton-threads.h"
#include "utils.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "lepton_msgs/Num.h"

#include <sstream>


// global variables
uint8_t flirData[PACKET_SIZE*PACKETS_PER_FRAME];
pthread_mutex_t mutex_flirData; // controls the concurrent access to FLIR data
uint16_t *flirFrameBuffer = (uint16_t *)flirData;
uint16_t minValue = FLIR_MAX_VALUE;
uint16_t maxValue = FLIR_MIN_VALUE;
//extern int app_exit;






void *readDataFromFLIR(void *ptr) {
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<lepton_msgs::Num>("thermic_image", 1000);

//	ros::Rate loop_rate(10);
	int count = 0;
	while (ros::ok()){
		lepton_msgs::Num msg;
#ifdef DEBUG
		printf("FLIR sensor is being read...\n");
#endif
#ifdef DEBUG
		printf("Reading SPI...");
#endif
		// entering critical region
		pthread_mutex_lock(&mutex_flirData); 

		//read data packets from lepton over SPI
		int resets = 0;
		for(int j=0;j<PACKETS_PER_FRAME;j++) {
			//if it's a drop packet, reset j to 0, set to -1 so it'll be at 0 again loop
#ifdef DEBUG
			printf("\nReading packet # %d", j);
#endif
			// reading the next packet
			read(spi_cs0_fd, flirData+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
			int packetNumber = flirData[j*PACKET_SIZE+1];
			// checking if it is the correct packet from the current frame
			if(packetNumber != j) {
				j = -1;
				resets += 1;
				usleep(1000);
				//Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
				//By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
				if(resets == 750) {
					SpiClosePort(0);
					usleep(750000);
					SpiOpenPort(0);
				}
			}
		}
		// leaving critical region
		//pthread_mutex_unlock(&mutex_flirData); 
#ifdef DEBUG
		printf("done.\n");
#endif
		if(resets >= 30) {
			printf("Number of resets: %d\n", resets);
		}

		uint16_t value;
		for(int i=0;i<FRAME_SIZE_UINT16;i++) {
			//skip the first 2 uint16_t's of every packet, they're 4 header bytes
			if(i % PACKET_SIZE_UINT16 < 2) {
				continue;
			}

			// entering critical region
			//pthread_mutex_lock(&mutex_flirData); 
			//flip the MSB and LSB at the last second
			int temp = flirData[i*2];
			flirData[i*2] = flirData[i*2+1];
			flirData[i*2+1] = temp;
			
			value = flirFrameBuffer[i];
			if(value > maxValue) {
#ifdef SINGLE_MAX_MIN_VALUE
				// this implementation has a fixed range of value for all images built from
				// the sensor data, in order to have a unique color scale for the whole video
				// the range of value increases smoothly and remains stable during the whole video
				
				if (maxValue == FLIR_MIN_VALUE) {
					// plus 5% to decrease the chance of abrupt variance
					maxValue = value + (value/20);
					printf("MIN = %d\t\t*MAX* = %d\n", minValue, maxValue);
				}
				else {

					//maxValue = (maxValue + value + (value/10))/2; 
					int actualRange = maxValue - minValue;
					int newRange = value - minValue;
					int diff = newRange - actualRange;
					printf("V=%d\tA=%d\tN=%d\tDIFF = %d\n", value, actualRange, newRange, diff);
					if (diff < (actualRange/20)) {
						maxValue = value; //+ (value/10);
						printf("MIN = %d\t\t*MAX* = %d\n", minValue, maxValue);
					}
				}
#else
				// this implementation variable range value for all images built from
				// the sensor data. In other words, the range is stablished for each frame,
				// and hence, the color scale varies during the video
				// the range of values may change at each frame
				maxValue = value;
				printf("MIN = %d\t\t*MAX* = %d\n", minValue, maxValue);
#endif
			}
			if(value < minValue) {
#ifdef SINGLE_MAX_MIN_VALUE
				// this implementation has a fixed range of value for all images built from
				// the sensor data, in order to have a unique color scale for the whole video
				// the range of value increases smoothly and remains stable during the whole video
								
				if (minValue == FLIR_MAX_VALUE) {
					// minus 5% to decrease the chance of abrupt variance
					minValue = value - (value/20);
					printf("*MIN* = %d\t\tMAX = %d\n", minValue, maxValue);
				}
				else {
					//minValue = (minValue - value - (value/10))/2;
					int actualRange = maxValue - minValue;
					int newRange = maxValue - value;
					int diff = newRange - actualRange;
					if (diff < (actualRange/20)) {
						minValue = value; //- (value/10);
						printf("*MIN* = %d\t\tMAX = %d\n", minValue, maxValue);
					}
				}
#else
				// this implementation variable range value for all images built from
				// the sensor data. In other words, the range is stablished for each frame,
				// and hence, the color scale varies during the video
				// the range of values may change at each frame
				
				minValue = value;
				printf("MIN = %d\t\t*MAX* = %d\n", minValue, maxValue);
#endif
			}
			// leaving critical region
			//pthread_mutex_unlock(&mutex_flirData); 
		}
		// leaving critical region
		pthread_mutex_unlock(&mutex_flirData); 
		// wait 1ms to alleviate the CPU processing
		SLEEP_1_MS;
#ifdef DEBUG
		printf("FLIR data has been completely read.\n");
#endif
//		
		int c;
		for(c=0; c<PACKET_SIZE*PACKETS_PER_FRAME; c++)
			msg.dados[c] = flirData[c];
		chatter_pub.publish(msg);
		ros::spinOnce();
//		loop_rate.sleep();
		++count;



	}
	// assuring the mutex will be released before exiting
	pthread_mutex_unlock(&mutex_flirData); 
	
	pthread_exit(NULL);
}
