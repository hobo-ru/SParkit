/*
 * mag_sensor.h
 *
 *  Created on: 06 рту. 2014 у.
 *      Author: vdubikhin
 */


#ifndef MAG_SENSOR_H_
#define MAG_SENSOR_H_

//#include <stdint.h>

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}magnetdata_t;
int initSensor();
int getData(magnetdata_t *data);
int getSelfTest(magnetdata_t *data, uint8_t direction); 
void SaveTempCorr(void);
void UpdateTempCorr(void);
void ApplyTempCorr(magnetdata_t * data_in);
#endif /* MAG_SENSOR_H_ */
