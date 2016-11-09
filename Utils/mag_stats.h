/*
 * mag_stats.h
 *
 *  Created on: 21 мая 2014 г.
 *      Author: vdubikhin
 */

#ifndef MAG_STATS_H_
#define MAG_STATS_H_

#define DATA_POINTS 32
#define DATA_AVG_POINTS 1
#define AVG_POINTS_TO_MEASURE 4096


//store one data point in internal buffer
uint8_t logData(int16_t dataX, int16_t dataY, int16_t dataZ);

//return current constant values
int8_t calcCurConst();

int8_t detectObject();

//reset internal buffers
void resetMagnetData();

// measure current magnetic field constants
uint8_t MeasureAverage(int16_t dataX, int16_t dataY, int16_t dataZ);

int8_t getAvgConsts(int* buffer);

int8_t getAvgPoints(int* buffer, uint8_t part);

int8_t getAvgSave(int* buffer);

int getLinRegConstant();

int8_t getAvgConstantsSave(int* buffer);

#endif /* MAG_STATS_H_ */
