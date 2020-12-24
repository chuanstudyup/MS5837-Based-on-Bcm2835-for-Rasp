#ifndef MS5837_H
#define MS5837_H

#include <bcm2835.h>
#include <cstdio>
#include <iostream>
#include <cstdint>
#include <cmath>

using namespace std;

class MS5837 {
public:
	static const float Pa;
	static const float bar;
	static const float mbar;

	static const uint8_t MS5837_30BA;
	static const uint8_t MS5837_02BA;

	MS5837();
	~MS5837();
	bool init();

	/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
	 * and MS5837::MS5837_02BA.
	 */
	void setModel(uint8_t model);

	/** Provide the density of the working fluid in kg/m^3. Default is for 
	 * seawater. Should be 997 for freshwater.
	 */
	void setFluidDensity(float density);

	/** The read from I2C takes up to 40 ms, so use sparingly is possible.
	 *  When OSR = 2048, The read from I2C takes up to 10 ms
	 */
	void read();

	/** Pressure returned in mbar or mbar*conversion rate.
	 */
	float pressure(float conversion = 1.0f);

	/** Temperature returned in deg C.
	 */
	float temperature();

	/** Depth returned in meters (valid for operation in incompressible
	 *  liquids only. Uses density that is set for fresh or seawater.
	 */
	float depth();

	/** Altitude returned in meters (valid for operation in air only).
	 */
	float altitude();
	
	void autoCalibration();
	
private:
	uint16_t C[8];
	uint32_t D1, D2;
	int32_t TEMP;
	int32_t P;
	uint8_t _model;
	
	/**
	 * Expose the sensor to the air and calculate the bias parameter based on
	 * the original depth data,the purpose is to make the depth 0
	 **/
	int32_t offsetP;  
	
	char sendBuf[5],recvBuf[5];  //IIC send of recv buffer

	/**
	 * bcm2835 iic error code
	 * 0x00  BCM2835_I2C_REASON_OK 	Success
	 * 0x01 BCM2835_I2C_REASON_ERROR_NACK 	Received a NACK
	 * 0x02 BCM2835_I2C_REASON_ERROR_CLKT 	Received Clock Stretch Timeout
	 * 0x03 BCM2835_I2C_REASON_ERROR_DATA 	Not all data is sent / received
	 **/ 
	uint8_t errCode;  
	
	float fluidDensity;

	/** Performs calculations per the sensor data sheet for conversion and
	 *  second order compensation.
	 */
	void calculate();

	uint8_t crc4(uint16_t n_prom[]);
};

#endif  //MS5837_H
