/*
 * mag_stats.c
 *
 *  Created on: 21 мая 2014 г.
 *      Author: vdubikhin
 */

#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

#include "mag_stats.h"

//general storage structure
typedef struct DataPoint {
    int  data[3]; //3 axis
    uint8_t valid;
} DataPoint;

typedef struct DataPointF {
    // 0 - Y(X)
    // 1 - Y(Z)
    float  corr[2];
    float  coef[2];
    int  constant[2];
    int linRegConstant[2];
    uint8_t valid;
} DataPointF;

typedef struct DataPointDeriv {
    // 0 - Y(X)
    // 1 - Y(Z)
    float  coef[DATA_POINTS][2];
    float  corr[2];
    int  constant[2];
    uint8_t valid;
} DataPointDeriv;

typedef struct Deviation {
    float deviation;
    int average;
} Deviation;

static int dataArr[DATA_POINTS][3];
static DataPointDeriv derivData;
static int avgCurConstant[DATA_AVG_POINTS][2];
static int avgCurrentSave[DATA_AVG_POINTS];
static int avgavgCurConstantSave[DATA_AVG_POINTS];
static uint8_t avgCurConstantPointer = 0;

DataPoint  baseVal;

static DataPoint avgConstants;
static int avgConstantsCur[3];
static int avgConstantsSave[3];
static int avgConstantsCurPointer = 0;

static DataPoint avgCurrent;
// 0 - Y-X
// 1 - Z-X
// 2 - Y-Z
static DataPointF corrCoeff;

static uint16_t dataPointer = 0;
static uint32_t numPoints = 0;

void resetMagnetData() {
    avgConstants.data[0] = 0;
    avgConstants.data[1] = 0;
    avgConstants.data[2] = 0;
    avgConstants.valid = 0;

    dataPointer = 0;
    avgCurrent.data[0] = 0;
    avgCurrent.data[1] = 0;
    avgCurrent.data[2] = 0;
    avgCurConstantPointer = 0;

    avgConstantsCur[0] = 0;
    avgConstantsCur[1] = 0;
    avgConstantsCur[2] = 0;


    corrCoeff.coef[0] = 0;
    corrCoeff.corr[0] = 0;
    corrCoeff.constant[0] = 0;

    corrCoeff.coef[1] = 0;
    corrCoeff.corr[1] = 0;
    corrCoeff.constant[1] = 0;

    corrCoeff.valid = 0;

    numPoints = 0;
    avgCurConstantPointer = 0;
}


int8_t calcCurConst() {

    //check that average and data are present
    if (derivData.valid != 1 || corrCoeff.valid != 1 || avgConstants.valid != 1)
        return -1;

    corrCoeff.constant[0] = 0;
    corrCoeff.constant[1] = 0;

    derivData.constant[0] = 0;
    derivData.constant[1] = 0;

    for (int i = 0; i < DATA_POINTS; i++) {
        //y_constant_x = y - b(x,y)*(x-x_constant)
        corrCoeff.constant[0] += dataArr[i][1] - (int)(corrCoeff.coef[0]*(float)(dataArr[i][0] -
                avgConstants.data[0]));

        //y_constant_z = y - b(z,y)*(z-z_constant)
        corrCoeff.constant[1] += dataArr[i][1] - (int)(corrCoeff.coef[1]*(float)(dataArr[i][2] -
                avgConstants.data[2]));

        //deriv(x)
        derivData.constant[0] += dataArr[i][1] - (int)(derivData.coef[i][0]*(float)(dataArr[i][0] -
                avgConstants.data[0]));

        //deriv(z)
        derivData.constant[1] += dataArr[i][1] - (int)(derivData.coef[i][1]*(float)(dataArr[i][2] -
                avgConstants.data[2]));
    }
    corrCoeff.constant[0] /= DATA_POINTS;
    corrCoeff.constant[1] /= DATA_POINTS;

    derivData.constant[0] /= DATA_POINTS;
    derivData.constant[1] /= DATA_POINTS;

    if (avgCurConstantPointer < DATA_AVG_POINTS) {
        //choose best axis with correlation
        if (fabs(corrCoeff.corr[0]) > fabs(corrCoeff.corr[1])) {
            if (fabs(corrCoeff.corr[0]) > 0.2)
                avgCurConstant[avgCurConstantPointer][0] = corrCoeff.constant[0];
            else
                avgCurConstant[avgCurConstantPointer][0] = avgCurrent.data[1];
        } else {
            if (fabs(corrCoeff.corr[1]) > 0.2)
                avgCurConstant[avgCurConstantPointer][0] = corrCoeff.constant[1];
            else
                avgCurConstant[avgCurConstantPointer][0] = avgCurrent.data[1];
        }

        if (fabs(derivData.corr[0]) > fabs(derivData.corr[1]))
            avgCurConstant[avgCurConstantPointer][1] = derivData.constant[0];
        else
            avgCurConstant[avgCurConstantPointer][1] = derivData.constant[1];

        avgavgCurConstantSave[avgCurConstantPointer] = avgCurConstant[avgCurConstantPointer][0];

        avgCurConstantPointer += 1;
    }

    dataPointer = 0;

    avgCurrentSave[avgCurConstantPointer] = avgCurrent.data[1];

    avgCurrent.data[0] = 0;
    avgCurrent.data[1] = 0;
    avgCurrent.data[2] = 0;
    corrCoeff.valid = 0;

    return 1;
}

static Deviation standardDeviation(int* buffer, int size) {
    Deviation deviation;

    deviation.average = 0;
    for (int i = 0; i < size; i++)
        deviation.average += *(buffer+i);

    deviation.average /= size;

    int diffSq = 0;
    for (int i = 0; i < size; i++)
        diffSq += *(buffer+i) - deviation.average;

    deviation.deviation = sqrt((float)diffSq/(float)(size-1));

    return deviation;
}

int8_t detectObject() {
    if (avgCurConstantPointer >= DATA_AVG_POINTS) {
        avgCurConstantPointer = 0;

        //calculate average and standard deviation for avgCurrent
        Deviation deviationAvg = standardDeviation(avgCurrentSave, DATA_AVG_POINTS);

        //calculate average and standard deviation for currentConstant
        Deviation deviationCurConstant = standardDeviation(avgavgCurConstantSave, DATA_AVG_POINTS);

        return 1;
    }

    return 0;
}

uint8_t logData(int16_t dataX, int16_t dataY, int16_t dataZ) {
    // fill data array
    if (dataPointer < DATA_POINTS) {
        dataArr[dataPointer][0] = dataX;
        dataArr[dataPointer][1] = dataY;
        dataArr[dataPointer][2] = dataZ;

        avgCurrent.data[0] += dataX;
        avgCurrent.data[1] += dataY;
        avgCurrent.data[2] += dataZ;
        dataPointer += 1;

        avgConstantsCur[0] += dataX;
        avgConstantsCur[1] += dataY;
        avgConstantsCur[2] += dataZ;
        avgConstantsCurPointer += 1;

        if (avgConstantsCurPointer >= AVG_POINTS_TO_MEASURE) {
            avgConstantsSave[0] = avgConstantsCur[0]/AVG_POINTS_TO_MEASURE;
            avgConstantsSave[1] = avgConstantsCur[1]/AVG_POINTS_TO_MEASURE;
            avgConstantsSave[2] = avgConstantsCur[2]/AVG_POINTS_TO_MEASURE;

            avgConstantsCur[0] = 0;
            avgConstantsCur[1] = 0;
            avgConstantsCur[2] = 0;

            avgConstantsCurPointer = 0;
        }

        //average calculation is here so that it wont be recalculated every function call
        if (dataPointer >= DATA_POINTS) {
            avgCurrent.data[0] /= DATA_POINTS;
            avgCurrent.data[1] /= DATA_POINTS;
            avgCurrent.data[2] /= DATA_POINTS;
        }
    } else {
        // calculate correlation coefficient
        float covYX = 0, covYZ = 0;
        float meanSqX  = 0, meanSqY  = 0, meanSqZ  = 0;
        float diffX, diffY, diffZ;

        float derivX, derivY, derivZ;
        float derivXAvg = 0, derivYAvg = 0, derivZAvg = 0;
        int16_t derivXArr[DATA_POINTS], derivYArr[DATA_POINTS], derivZArr[DATA_POINTS];

        // calculate needed parts
        for (int i = 0; i < DATA_POINTS; i++) {
            //1st method - linear regression
            diffX = dataArr[i][0] - avgCurrent.data[0];
            diffY = dataArr[i][1] - avgCurrent.data[1];
            diffZ = dataArr[i][2] - avgCurrent.data[2];

            covYX += diffX*diffY;
            covYZ += diffY*diffZ;

            meanSqX += diffX*diffX;
            meanSqY += diffY*diffY;
            meanSqZ += diffZ*diffZ;

            //2nd method - derivative
            //left border
            if (i == 0) {
                derivX = dataArr[1][0] - dataArr[0][0];
                derivY = dataArr[1][1] - dataArr[0][1];
                derivZ = dataArr[1][2] - dataArr[0][2];
            }

            //right border
            if (i == DATA_POINTS - 1) {
                derivX = dataArr[DATA_POINTS - 1][0] - dataArr[DATA_POINTS - 2][0];
                derivY = dataArr[DATA_POINTS - 1][1] - dataArr[DATA_POINTS - 2][1];
                derivZ = dataArr[DATA_POINTS - 1][2] - dataArr[DATA_POINTS - 2][2];
            }

            //3 points pattern in between
            if (i > 0 && i < DATA_POINTS - 1) {
                derivX = (dataArr[i+1][0] - dataArr[i-1][0])/2;
                derivY = (dataArr[i+1][1] - dataArr[i-1][1])/2;
                derivZ = (dataArr[i+1][2] - dataArr[i-1][2])/2;
            }

            if (derivX == 0)
                derivData.coef[i][0] = 0;
            else
                derivData.coef[i][0] = derivY/derivX;

            if (derivZ == 0)
                derivData.coef[i][1] = 0;
            else
                derivData.coef[i][1] = derivY/derivZ;

            derivXAvg += derivX;
            derivYAvg += derivY;
            derivZAvg += derivZ;

            derivXArr[i] = derivX;
            derivYArr[i] = derivY;
            derivZArr[i] = derivZ;
        }

        derivXAvg /= DATA_POINTS;
        derivYAvg /= DATA_POINTS;
        derivZAvg /= DATA_POINTS;

        float covDerivYX = 0, covDerivYZ = 0;
        float meanSqDerivX  = 0, meanSqDerivY  = 0, meanSqDerivZ  = 0;
        float diffDerivX, diffDerivY, diffDerivZ;

        for (int i = 0; i < DATA_POINTS; i++) {
            diffDerivX = derivXArr[i] - derivXAvg;
            diffDerivY = derivYArr[i] - derivYAvg;
            diffDerivZ = derivZArr[i] - derivZAvg;

            covDerivYX += diffDerivY*diffDerivX;
            covDerivYZ += diffDerivY*diffDerivZ;

            meanSqDerivX += diffDerivX*diffDerivX;
            meanSqDerivY += diffDerivY*diffDerivY;
            meanSqDerivZ += diffDerivZ*diffDerivZ;
        }


        //calculate the rest
        corrCoeff.coef[0] = covYX/meanSqX;
        corrCoeff.corr[0] = covYX/sqrt(meanSqX*meanSqY);
        corrCoeff.linRegConstant[0] = avgCurrent.data[1] -
                (int)(corrCoeff.coef[0]*avgCurrent.data[0]);

        corrCoeff.coef[1] = covYZ/meanSqZ;
        corrCoeff.corr[1] = covYZ/sqrt(meanSqZ*meanSqY);
        corrCoeff.linRegConstant[1] = avgCurrent.data[1] -
                (int)(corrCoeff.coef[1]*avgCurrent.data[2]);

        corrCoeff.valid = 1;

        derivData.corr[0] = covDerivYX/sqrt(meanSqDerivX*meanSqDerivY);
        derivData.corr[1] = covDerivYZ/sqrt(meanSqDerivZ*meanSqDerivY);
        derivData.valid = 1;

        return 1;
    }

    return 0;
}

uint8_t MeasureAverage(int16_t dataX, int16_t dataY, int16_t dataZ) {
    // if less than number of points to measure
    if (numPoints < AVG_POINTS_TO_MEASURE) {
        // log data
        avgConstants.data[0] += dataX;
        avgConstants.data[1] += dataY;
        avgConstants.data[2] += dataZ;
        avgConstants.valid = 0;

        numPoints += 1;

        if (numPoints >= AVG_POINTS_TO_MEASURE) {
            // calculate average
            avgConstants.data[0] /= AVG_POINTS_TO_MEASURE;
            avgConstants.data[1] /= AVG_POINTS_TO_MEASURE;
            avgConstants.data[2] /= AVG_POINTS_TO_MEASURE;
            avgConstants.valid = 1;

            avgConstantsSave[0] = avgConstants.data[0];
            avgConstantsSave[1] = avgConstants.data[1];
            avgConstantsSave[2] = avgConstants.data[2];
        }

    } else
        return 1;

    return 0;
}

int8_t getAvgSave(int* buffer) {
    for (int i = 0; i < DATA_AVG_POINTS; i++) {
        buffer[i] = avgCurrentSave[i];
    }
    return 1;
}

int8_t getAvgConstantsSave(int* buffer) {
    buffer[0] = avgConstantsSave[0];
    buffer[1] = avgConstantsSave[1];
    buffer[2] = avgConstantsSave[2];
    return 1;
}

int8_t getAvgConsts(int* buffer) {
    buffer[0] = avgConstants.data[0];
    buffer[1] = avgConstants.data[1];
    buffer[2] = avgConstants.data[2];
    return avgConstants.valid;
}

int getLinRegConstant() {
    if (fabs(corrCoeff.corr[0]) > fabs(corrCoeff.corr[1]))
        return corrCoeff.linRegConstant[0];
    else
        return corrCoeff.linRegConstant[1];
}

int8_t getAvgPoints(int* buffer, uint8_t part) {
    for (int i = 0; i < DATA_AVG_POINTS; i++) {
        buffer[0] = avgCurConstant[0][part];
    }
    return 0;
}
