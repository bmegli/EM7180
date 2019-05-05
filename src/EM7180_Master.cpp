/* 

   EM7180_Master.cpp: Class implementation for EM7180 SENtral Sensor in master mode

   Copyright (C) 2018 Simon D. Levy

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/MPU9250_BMP280

   This file is part of EM7180.

   EM7180 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   EM7180 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "EM7180_Master.h"

#include <CrossPlatformI2C_Core.h>

#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP

// WARM START
static const uint8_t M24512DFM_DATA_ADDRESS   = 0x50;   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
static const uint8_t SEN_WS_DATA_ADDRESS   = 0x80;   //spans two pages:
// - 128 bytes (from offset 0)
// - 12 bytes (from offset 0x80 = 128)
// - 12 bytes (from offset 0x8c = 140) of ACCEL calibration data


//temp
#define MAG_SCALE                          0x3E8  // +/-1000uT


/**************************              Sensor Scales                ****************************/
 /* MPU9250 Acc Output Scale. Uncomment only one option */
 //#define ACC_SCALE                          0x02   // +/-2g
 //#define ACC_SCALE                          0x04   // +/-4g
 #define ACC_SCALE                          0x08   // +/-8g
 //#define ACC_SCALE                          0x10   // +/-16g

 
/* MPU9250 Gyro Output Scale. Uncomment only one option */
 //#define GYRO_SCALE                         0xFA   // +/-250DPS
 //#define GYRO_SCALE                         0x1F4  // +/-500DPS
 //#define GYRO_SCALE                         0x3E8  // +/-1000DPS
 #define GYRO_SCALE                         0x7D0  // +/-2000DPS 
 
struct acc_cal
{
  int16_t accZero_max[3];
  int16_t accZero_min[3];
};

struct Sentral_WS_params
{
  uint8_t Sen_param[35][4];
};

static acc_cal            global_conf;
static Sentral_WS_params  WS_params;

// WS

void EM7180_Master::EM7180_set_WS_params()
{
    uint8_t param = 1;
    uint8_t stat;

    // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    param = param | 0x80;
    _em7180.loadParamByte0(WS_params.Sen_param[0][0]);
    _em7180.loadParamByte1(WS_params.Sen_param[0][1]);
    _em7180.loadParamByte2(WS_params.Sen_param[0][2]);
    _em7180.loadParamByte3(WS_params.Sen_param[0][3]);
    _em7180.requestParamRead(param);

    // Request parameter transfer procedure
    _em7180.algorithmControlRequestParameterTransfer();

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    stat = _em7180.getParamAcknowledge();
    while(!(stat==param)) {
		
	//BM - commented out
	// _em7180.loadParamByte0(WS_params.Sen_param[0][0]);
   // _em7180.loadParamByte1(WS_params.Sen_param[0][1]);
   // _em7180.loadParamByte2(WS_params.Sen_param[0][2]);
    //_em7180.loadParamByte3(WS_params.Sen_param[0][3]);
	// _em7180.requestParamRead(param); 
   // _em7180.algorithmControlRequestParameterTransfer();
		 
        stat = _em7180.getParamAcknowledge();
    }
	 	 
    for(uint8_t i=1; i<35; i++) {
        param = (i+1) | 0x80;
        _em7180.loadParamByte0(WS_params.Sen_param[i][0]);
        _em7180.loadParamByte1(WS_params.Sen_param[i][1]);
        _em7180.loadParamByte2(WS_params.Sen_param[i][2]);
        _em7180.loadParamByte3(WS_params.Sen_param[i][3]);
        _em7180.requestParamRead(param);

        // Check the parameter acknowledge register and loop until the result matches parameter request byte
        stat = _em7180.getParamAcknowledge();
        while(!(stat==param))
            stat = _em7180.getParamAcknowledge();
        
    }
	 	 
    // Parameter request = 0 to end parameter transfer process
    _em7180.requestParamRead(0x00);
	 _em7180.algorithmControlReset();
}

void EM7180_Master::M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(device_address);            // Initialize the Tx buffer
    Wire.write(data_address1);                         // Put slave register address in Tx buffer
    Wire.write(data_address2);                         // Put slave register address in Tx buffer
    Wire.endTransmission(NOSTOP);                  // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(device_address, (size_t)count);  // Read bytes from slave register address 
    while (Wire.available())
        dest[i++] = Wire.read();
	// Put read results in the Rx buffer
}


//M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
//M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
void EM7180_Master::readSenParams(uint8_t address) //adjust addresses
{
    uint8_t data[140];
    uint8_t paramnum;

    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, address, 0x80, 12, &data[128]);
	 delay(100);
	 M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, address, 0x00, 128, &data[0]);

    for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
        for (uint8_t i= 0; i < 4; i++)
            WS_params.Sen_param[paramnum][i] = data[(paramnum*4 + i)];
}

// ACC

#define ACC_NORTH 0        // MPU9250 X-axis
#define ACC_EAST  1        // MPU9250 Y-axis
#define ACC_DOWN  2        // MPU9250 Z-axis

void EM7180_Master::EM7180_acc_cal_upload()
{
  int64_t big_cal_num;
  union
  {
    int16_t cal_num;
    unsigned char cal_num_byte[2];
  };
  
  if (!_accelCal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    //NORTH SCALE
    big_cal_num = (4096000000/(global_conf.accZero_max[ACC_NORTH] - global_conf.accZero_min[ACC_NORTH])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  
  _em7180.writeGp36(cal_num_byte[0]);
  _em7180.writeGp37(cal_num_byte[1]);
  
  if (!_accelCal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
   // EAST SCALE
   big_cal_num = (4096000000/(global_conf.accZero_max[ACC_EAST] - global_conf.accZero_min[ACC_EAST])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  
  _em7180.writeGp38(cal_num_byte[0]);
  _em7180.writeGp39(cal_num_byte[1]);

  if (!_accelCal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
   // DOWN SCALE
   big_cal_num = (4096000000/(global_conf.accZero_max[ACC_DOWN] - global_conf.accZero_min[ACC_DOWN])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  _em7180.writeGp40(cal_num_byte[0]);
  _em7180.writeGp50(cal_num_byte[1]);

  if (!_accelCal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    // NORTH OFFSET - is the sign correct?
    big_cal_num = (((global_conf.accZero_max[ACC_NORTH] - 2048) + (global_conf.accZero_min[ACC_NORTH] + 2048))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  
	_em7180.writeGp51(cal_num_byte[0]);
  _em7180.writeGp52(cal_num_byte[1]);

  if (!_accelCal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    // EAST OFFSET - is the sign correct?
    big_cal_num = (((global_conf.accZero_max[ACC_EAST] - 2048) + (global_conf.accZero_min[ACC_EAST] + 2048))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  
	_em7180.writeGp53(cal_num_byte[0]);
  _em7180.writeGp54(cal_num_byte[1]);

  if (!_accelCal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    // DOWN OFFSET - is the sign correct?
    big_cal_num = (((global_conf.accZero_max[ACC_DOWN] - 2048) + (global_conf.accZero_min[ACC_DOWN] + 2048))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  
  _em7180.writeGp55(cal_num_byte[0]);
  _em7180.writeGp56(cal_num_byte[1]);
}

void EM7180_Master::readAccelCal(uint8_t address1, uint8_t address2) 
{
    uint8_t data[12];
    uint8_t axis;

    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, address1, address2, 12, data); // Page 255
    for (axis = 0; axis < 3; axis++)
    {
        global_conf.accZero_max[axis] = ((int16_t)(data[(2*axis + 1)]<<8) | data[2*axis]);
        global_conf.accZero_min[axis] = ((int16_t)(data[(2*axis + 7)]<<8) | data[(2*axis + 6)]);
    }
}


// END OF WARM START

EM7180_Master::EM7180_Master(uint8_t magRate, uint16_t accelRate, uint16_t gyroRate, uint8_t baroRate, uint8_t qRateDivisor, bool accelCal, bool warmStart)
{
    _magRate = magRate;
    _accelRate = accelRate;
    _gyroRate = gyroRate; 
    _baroRate = baroRate;
    _qRateDivisor = qRateDivisor;
	 _accelCal = accelCal;
	 _warmStart = warmStart;
}

const char * EM7180_Master::getErrorString(void)
{
    return _em7180.getErrorString();
}


void EM7180_Master::loadAccelAndWSFromEEPROM()
{
	if(!_accelCal && !_warmStart)
		return;
		
	_em7180.setPassThroughMode();

	if(_accelCal)
		readAccelCal(SEN_WS_DATA_ADDRESS, 0x8c); //WS uses 128 + 12, ACCEL starts at next byte (0x8c)
		
	if(_warmStart)
		readSenParams(SEN_WS_DATA_ADDRESS);

	_em7180.setMasterMode();
}

bool EM7180_Master::begin(uint8_t bus)
{  
  // Fail immediately if unable to upload EEPROM
  if (!_em7180.begin(bus)) return false;

  delay(100);

  loadAccelAndWSFromEEPROM();
    
  // Be sure Sentral is in "Idle" state
  _em7180.setRunDisable();

	 if(_accelCal)
	 {
		EM7180_acc_cal_upload();
		delay(500);
	 }

	_em7180.setRunEnable();

  delay(20);

// koniec wczytania kalibradcji akcelerometru

  // Apply Warm Start Parameters

	if(_warmStart)
	{
		EM7180_set_WS_params();
		delay(1000);
	}
  // Set Sensor LPF bandwidth. MUST BE DONE BEFORE SETTING ODR's
    _em7180.setAccelLpfBandwidth(0x03); // 41Hz
    _em7180.setGyroLpfBandwidth(0x03);  // 41Hz


    // Set accel/gyro/mage desired ODR rates
    _em7180.setAccelRate(_accelRate/10);
    _em7180.setGyroRate(_gyroRate/10);
    _em7180.setMagRate(_magRate);
    _em7180.setQRateDivisor(_qRateDivisor-1);

  // ODR + 10000000b to activate the eventStatus bit for the barometer...
    _em7180.setBaroRate(0x80 | _baroRate); // 0x80 = enable bit


  // Configure operating mode
  // Output scaled sensor data (Quaternion convention NED)  
  _em7180.algorithmControlReset();
  
  // Enable interrupt to host upon certain events:
    // quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    _em7180.enableEvents(0x07);

  // Start the Sentral
  _em7180.algorithmControlReset();
  
  
  // Perform final Sentral alogorithm parameter modifications
	_em7180.setIntegerParam (0x49, 0x00); // Disable "Stillness" mode
	 _em7180.setIntegerParam (0x48, 0x01);   // Set Gbias_mode to 1
	 _em7180.setMagAccFs(MAG_SCALE, ACC_SCALE);  // Set magnetometer/accelerometer full-scale ranges
	 _em7180.setGyroFs(GYRO_SCALE); // Set gyroscope full-scale range
	 _em7180.setFloatParam (0x3B, 0.0f);         			                                                                         // Param 59 Mag Transient Protect off (0.0)
  ////EM7180::EM7180_set_float_param (0x34, 4.0f);                                                                                  // Param 52 Mag merging rate (0.7 default)
  ////EM7180::EM7180_set_float_param (0x35, 0.3f);                                                                                  // Param 53 Accel merging rate (0.6 default)
 
  
  //I2C->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x02);                                                                // Diagnostic; reports unscaled sensor data

  // Choose interrupt events: Gyros updated (0x20), Sentral error (0x02) or Sentral reset (0x01)
  _em7180.enableEvents(0x07);

  // Read event status register
  return _em7180.getSensorStatus() ? false : true;	
}

 
void EM7180_Master::checkEventStatus(void)
{
    // Check event status register, way to check data ready by checkEventStatusing rather than interrupt
    _eventStatus = _em7180.getEventStatus(); // reading clears the register

}

bool EM7180_Master::checkEventStatusAsync(void)
{
    return _em7180.getEventStatusAsync(_eventStatus);
}

bool EM7180_Master::gotError(void)
{
    if (_eventStatus & 0x02) {

        return true;
    }

    return false;
}

bool EM7180_Master::gotQuaternion(void)
{
    return _eventStatus & 0x04;
}

bool EM7180_Master::gotMagnetometer(void)
{
    return _eventStatus & 0x08;
}

bool EM7180_Master::gotAccelerometer(void)
{
    return _eventStatus & 0x10;
}

bool EM7180_Master::gotGyrometer(void)
{
    return _eventStatus & 0x20;
}

bool EM7180_Master::gotBarometer(void)
{
    return _eventStatus & 0x40;
}

void EM7180_Master::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    _em7180.readQuaternion(qw, qx, qy, qz);
}

bool EM7180_Master::readQuaternionAsync(float & qw, float & qx, float & qy, float & qz)
{
    return _em7180.readQuaternionAsync(qw, qx, qy, qz);
}

void EM7180_Master::readThreeAxis(uint8_t regx, float & x, float & y, float & z, float scale)
{
    int16_t xx=0, yy=0, zz=0;

    _em7180.readThreeAxis(regx, xx, yy, zz);

    x = xx * scale;
    y = yy * scale;
    z = zz * scale;
}

void EM7180_Master::readAccelerometer(float & ax, float & ay, float & az)
{
    readThreeAxis(EM7180::AX, ax, ay, az, 0.0004882813f);
}

void EM7180_Master::readGyrometer(float & gx, float & gy, float & gz)
{
    readThreeAxis(EM7180::GX, gx, gy, gz, 0.153);
}

void EM7180_Master::readMagnetometer(float & mx, float & my, float & mz)
{
    readThreeAxis(EM7180::MX, mx, my, mz, 0.305176);
}

void EM7180_Master::readBarometer(float & pressure, float & temperature)
{
    _em7180.readBarometer(pressure, temperature);
}
