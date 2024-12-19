// I2Cdev library collection - LSM303DLHC I2C device class
// Based on ST LSM303DLHC datasheet, REV 2, 11/2013
// [current release date] by Nate Costello <natecostello at gmail dot com>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2015-03-10 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 [Author Name], Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "LSM303DLHC.h"

uint8_t devAddrA = LSM303DLHC_DEFAULT_ADDRESS_A;
uint8_t devAddrM = LSM303DLHC_DEFAULT_ADDRESS_M;
uint8_t buffer[6];
bool endianMode = LSM303DLHC_LITTLE_ENDIAN;

#if 0
/** Default constructor, uses default I2C address.
 * @see LSM303DLHC_DEFAULT_ADDRESS_A
 * @see LSM303DLHC_DEFAULT_ADDRESS_M
 */
LSM303DLHC_LSM303DLHC() {
    devAddrA = LSM303DLHC_DEFAULT_ADDRESS_A;
    devAddrM = LSM303DLHC_DEFAULT_ADDRESS_M;
    endianMode = LSM303DLHC_LITTLE_ENDIAN;
}
#endif

esp_err_t writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t val)
{
  static uint8_t txbuffer[2];
  txbuffer[0] = regAddr;
  txbuffer[1] = val;
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_A)
    return i2c_master_transmit(lsm303A_handle, txbuffer, 2, 10);
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_M)
    return i2c_master_transmit(lsm303M_handle, txbuffer, 2, 10);
  return ESP_ERR_NOT_FOUND;
}

esp_err_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *result)
{
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_A)
    i2c_master_transmit(lsm303A_handle, &regAddr, 1, 10);
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_M)
    i2c_master_transmit(lsm303M_handle, &regAddr, 1, 10);

  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_A)
    return i2c_master_receive(lsm303A_handle, result, 1, 10);
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_M)
    return i2c_master_receive(lsm303M_handle, result, 1, 10);
  return ESP_ERR_NOT_FOUND;
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
esp_err_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_A)
    i2c_master_transmit(lsm303A_handle, &regAddr, 1, 10);
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_M)
    i2c_master_transmit(lsm303M_handle, &regAddr, 1, 10);

  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_A)
    return i2c_master_receive(lsm303A_handle, data, length, 10);
  if (devAddr == LSM303DLHC_DEFAULT_ADDRESS_M)
    return i2c_master_receive(lsm303M_handle, data, length, 10);
  return ESP_ERR_NOT_FOUND;
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
esp_err_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
  uint8_t b;
  esp_err_t err = readByte(devAddr, regAddr, &b);
  *data = b & (1 << bitNum);
  return err;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 * @return Status of read operation (true = success)
 */
esp_err_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t b;
  esp_err_t err;
  if ((err = readByte(devAddr, regAddr, &b)) == ESP_OK)
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }
  return err;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
esp_err_t writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
  uint8_t b;
  readByte(devAddr, regAddr, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  return writeByte(devAddr, regAddr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
esp_err_t writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t b = 0;
  if (readByte(devAddr, regAddr, &b) == ESP_OK)
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask;                     // zero all non-important bits in data
    b &= ~(mask);                     // zero all important bits in existing byte
    b |= data;                        // combine data with existing byte
    return writeByte(devAddr, regAddr, b);
  }
  else
  {
    return ESP_ERR_NOT_FOUND;
  }
}

/** Specific address constructor.
@param addressA I2C accelerometer address
@param addressM I2C magnetometer address
@see LSM303DLHC_DEFAULT_ADDRESS_A
@see LSM303DLHC_DEFAULT_ADDRESS_M
@see LSM303DLHC_ADDRESS_A
@see LSM303DLHC_ADDRESS_M

LSM303DLHC_LSM303DLHC(uint8_t addressA, uint8_t addressM)
{
  devAddrA = addressA;
  devAddrM = addressM;
  endianMode = LSM303DLHC_LITTLE_ENDIAN;
}
*/
/** Power on and prepare for general usage.
All values are defaults except for the data rates for the accelerometer and
magnetometer data rates (200hz and 220hz respectively).
@see LSM303DLHC_RA_CTRL_REG1_A
@see LSM303DLHC_RA_CRA_REG_M
*/
void LSM303DLHC_initialize()
{
  writeByte(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, 0b01100111);
  writeByte(devAddrM, LSM303DLHC_RA_CRA_REG_M, 0b00011100);
  // ----------------------------------------------------------------------------
  // STUB TODO:
  // Perform any important initialization here. Maybe nothing is required, but
  // the method should exist anyway.
  // ----------------------------------------------------------------------------
}

/** Verify the I2C connection.
Make sure the device is connected and responds as expected.  This device has no
device ID or WHO_AM_I register.  To test the connection, bits are written to control
register, checked, and then the original contents are written back.
 * @return True if connection is valid, false otherwise
 */
bool LSM303DLHC_testConnection()
{
  readByte(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, buffer);
  uint8_t origValA = buffer[0];
  readByte(devAddrM, LSM303DLHC_RA_CRA_REG_M, buffer);
  uint8_t origValM = buffer[0];

  uint8_t zeros = 0b00000000;
  writeByte(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, zeros);
  writeByte(devAddrM, LSM303DLHC_RA_CRA_REG_M, zeros);

  readByte(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, buffer);
  uint8_t newValA = buffer[0];
  readByte(devAddrM, LSM303DLHC_RA_CRA_REG_M, buffer);
  uint8_t newValM = buffer[0];

  writeByte(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, origValA);
  writeByte(devAddrM, LSM303DLHC_RA_CRA_REG_M, origValM);

  if ((newValM == zeros) && (newValA == zeros))
  {
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------
// STUB TODO:
// Define methods to fully cover all available functionality provided by the
// device, according to the datasheet and/or register map. Unless there is very
// clear reason not to, try to follow the get/set naming convention for all
// values, for instance:
//   - uint8_t getThreshold()
//   - void setThreshold(uint8_t threshold)
//   - uint8_t getRate()
//   - void setRate(uint8_t rate)
//
// Some methods may be named differently if it makes sense. As long as all
// functionality is covered, that's the important part. The methods here are
// only examples and should not be kept for your real device.
// ----------------------------------------------------------------------------

// CTRL_REG1_A, r/w

/** Set the output data rate
 * @param rate The new data output rate (can be 1, 10, 25, 50, 100, 200, 400, 1620, 1344, or 5376)
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_ODR_BIT
 * @see LSM303DLHC_ODR_LENGTH
 * @see LSM303DLHC_RATE_1
 * @see LSM303DLHC_RATE_10
 * @see LSM303DLHC_RATE_25
 * @see LSM303DLHC_RATE_50
 * @see LSM303DLHC_RATE_100
 * @see LSM303DLHC_RATE_200
 * @see LSM303DLHC_RATE_400
 * @see LSM303DLHC_RATE_1620_LP
 * @see LSM303DLHC_RATE_1344_N_5376_LP
 */
void LSM303DLHC_setAccelOutputDataRate(uint16_t rate)
{
  uint8_t writeVal = 0x00;

  if (rate == 0)
  {
    writeVal = LSM303DLHC_ODR_RATE_POWERDOWN;
  }
  else if (rate == 1)
  {
    writeVal = LSM303DLHC_ODR_RATE_1;
  }
  else if (rate == 10)
  {
    writeVal = LSM303DLHC_ODR_RATE_10;
  }
  else if (rate == 25)
  {
    writeVal = LSM303DLHC_ODR_RATE_25;
  }
  else if (rate == 50)
  {
    writeVal = LSM303DLHC_ODR_RATE_50;
  }
  else if (rate == 100)
  {
    writeVal = LSM303DLHC_ODR_RATE_100;
  }
  else if (rate == 200)
  {
    writeVal = LSM303DLHC_ODR_RATE_200;
  }
  else if (rate == 400)
  {
    writeVal = LSM303DLHC_ODR_RATE_400;
  }
  else if (rate == 1620)
  {
    writeVal = LSM303DLHC_ODR_RATE_1620_LP;
  }
  else if (rate == 1344)
  {
    writeVal = LSM303DLHC_ODR_RATE_1344_N_5376_LP;
  }
  else if (rate == 5376)
  {
    writeVal = LSM303DLHC_ODR_RATE_1344_N_5376_LP;
  }

  writeBits(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_ODR_BIT, LSM303DLHC_ODR_LENGTH, writeVal);
}

/** Get the output data rate
 * @return The current data output rate (can be 1, 10, 25, 50, 100, 200, 400, 1620, or 1344 (implies 5376))
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_ODR_BIT
 * @see LSM303DLHC_ODR_LENGTH
 * @see LSM303DLHC_RATE_1
 * @see LSM303DLHC_RATE_10
 * @see LSM303DLHC_RATE_25
 * @see LSM303DLHC_RATE_50
 * @see LSM303DLHC_RATE_100
 * @see LSM303DLHC_RATE_200
 * @see LSM303DLHC_RATE_400
 * @see LSM303DLHC_RATE_1620_LP
 * @see LSM303DLHC_RATE_1344_N_5376_LP
 */
uint16_t LSM303DLHC_getAccelOutputDataRate()
{
  readBits(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_ODR_BIT,
           LSM303DLHC_ODR_LENGTH, buffer);
  uint8_t rate = buffer[0];

  if (rate == LSM303DLHC_ODR_RATE_POWERDOWN)
  {
    return 0;
  }
  else if (rate == LSM303DLHC_ODR_RATE_1)
  {
    return 1;
  }
  else if (rate == LSM303DLHC_ODR_RATE_10)
  {
    return 10;
  }
  else if (rate == LSM303DLHC_ODR_RATE_25)
  {
    return 25;
  }
  else if (rate == LSM303DLHC_ODR_RATE_50)
  {
    return 50;
  }
  else if (rate == LSM303DLHC_ODR_RATE_100)
  {
    return 100;
  }
  else if (rate == LSM303DLHC_ODR_RATE_200)
  {
    return 200;
  }
  else if (rate == LSM303DLHC_ODR_RATE_400)
  {
    return 400;
  }
  else if (rate == LSM303DLHC_ODR_RATE_1620_LP)
  {
    return 1620;
  }
  else if (rate == LSM303DLHC_ODR_RATE_1344_N_5376_LP)
  {
    return 1344;
  }
  else if (rate == LSM303DLHC_ODR_RATE_1344_N_5376_LP)
  {
    return 5376;
  }
  return 0;
}

/*Enables or disables the accelerometer low power mode
@param enabled The new enabled state of the low power mode
@see LSM303DLHC_RA_CTRL_REG1_A
@see LSM303DLHC_LPEN_BIT
*/
void LSM303DLHC_setAccelLowPowerEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_LPEN_BIT, enabled);
}

/*Get whether the accelerometer low power mode is enabled
@return True if the acceleromer low power mode is enabled
@see LSM303DLHC_RA_CTRL_REG1_A
@see LSM303DLHC_LPEN_BIT
*/
bool LSM303DLHC_getAccelLowPowerEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_LPEN_BIT, buffer);
  return buffer[0];
}

/** Enables or disables the ability to get Z data
 * @param enabled The new enabled state of the Z axis
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_ZEN_BIT
 */
void LSM303DLHC_setAccelZEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_ZEN_BIT, enabled);
}

/** Get whether Z axis data is enabled
 * @return True if the Z axis is enabled, false otherwise
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_ZEN_BIT
 */
bool LSM303DLHC_getAccelZEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_ZEN_BIT, buffer);
  return buffer[0];
}

/** Enables or disables the ability to get Y data
 * @param enabled The new enabled state of the Y axis
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_YEN_BIT
 */
void LSM303DLHC_setAccelYEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_YEN_BIT, enabled);
}

/** Get whether Y axis data is enabled
 * @return True if the Y axis is enabled, false otherwise
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_YEN_BIT
 */
bool LSM303DLHC_getAccelYEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_YEN_BIT, buffer);
  return buffer[0];
}

/** Enables or disables the ability to get X data
 * @param enabled The new enabled state of the X axis
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_XEN_BIT
 */
void LSM303DLHC_setAccelXEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_XEN_BIT, enabled);
}

/** Get whether X axis data is enabled
 * @return True if the X axis is enabled, false otherwise
 * @see LSM303DLHC_RA_CTRL_REG1_A
 * @see LSM303DLHC_XEN_BIT
 */
bool LSM303DLHC_getAccelXEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG1_A, LSM303DLHC_XEN_BIT, buffer);
  return buffer[0];
}

// CTRL_REG2_A r/w

/** Set the high pass mode
 * @param mode The new high pass mode
 * @see LSM303DLHC_RA_CTRL_REG2
 * @see LSM303DLHC_HPM_BIT
 * @see LSM303DLHC_HPM_LENGTH
 * @see LSM303DLHC_HPM_HRF
 * @see LSM303DLHC_HPM_REFERENCE
 * @see LSM303DLHC_HPM_NORMAL
 * @see LSM303DLHC_HPM_AUTORESET
 */
void LSM303DLHC_setAccelHighPassMode(uint8_t mode)
{
  writeBits(devAddrA, LSM303DLHC_RA_CTRL_REG2_A, LSM303DLHC_HPM_BIT,
            LSM303DLHC_HPM_LENGTH, mode);
}

/** Get the high pass mode
 * @return High pass mode
 * @see LSM303DLHC_RA_CTRL_REG2_A
 * @see LSM303DLHC_HPM_BIT
 * @see LSM303DLHC_HPM_LENGTH
 * @see LSM303DLHC_HPM_HRF
 * @see LSM303DLHC_HPM_REFERENCE
 * @see LSM303DLHC_HPM_NORMAL
 * @see LSM303DLHC_HPM_AUTORESET
 */
uint8_t LSM303DLHC_getAccelHighPassMode()
{
  readBits(devAddrA, LSM303DLHC_RA_CTRL_REG2_A, LSM303DLHC_HPM_BIT,
           LSM303DLHC_HPM_LENGTH, buffer);
  return buffer[0];
}

/** Set the high pass filter cut off frequency level (1 - 10)
 * @param level The new level for the hpcf, using one of the defined levels
 * @see LSM303DLHC_RA_CTRL_REG2_A
 * @see LSM303DLHC_HPCF_BIT
 * @see LSM303DLHC_HPCF_LENGTH
 * @see LSM303DLHC_HPCF1
 * @see LSM303DLHC_HPCF2
 * @see LSM303DLHC_HPCF3
 * @see LSM303DLHC_HPCF4
 */
void LSM303DLHC_setAccelHighPassFilterCutOffFrequencyLevel(uint8_t level)
{
  writeBits(devAddrA, LSM303DLHC_RA_CTRL_REG2_A, LSM303DLHC_HPCF_BIT,
            LSM303DLHC_HPCF_LENGTH, level);
}

/** Get the high pass filter cut off frequency level (1 - 10)
 * @return High pass filter cut off frequency level
 * @see LSM303DLHC_RA_CTRL_REG2_A
 * @see LSM303DLHC_HPCF_BIT
 * @see LSM303DLHC_HPCF_LENGTH
 * @see LSM303DLHC_HPCF1
 * @see LSM303DLHC_HPCF2
 * @see LSM303DLHC_HPCF3
 * @see LSM303DLHC_HPCF4
 */
uint8_t LSM303DLHC_getAccelHighPassFilterCutOffFrequencyLevel()
{
  readBits(devAddrA, LSM303DLHC_RA_CTRL_REG2_A, LSM303DLHC_HPCF_BIT,
           LSM303DLHC_HPCF_LENGTH, buffer);
  return buffer[0];
}

// CTRL_REG3_A r/w

/*Enable the Click interrupt routed to the INT1 pin.
@param enabled The new enabled state of the Click interrupt routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_CLICK_BIT
*/
void LSM303DLHC_setAccelINT1ClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_CLICK_BIT, enabled);
}

/*Get whether the Click interrupt is routed to the INT1 pin.
@return True if the Click interrupt is routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_CLICK_BIT
*/
bool LSM303DLHC_getAccelINT1ClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_CLICK_BIT, buffer);
  return buffer[0];
}

/*Enable the AOR1 interrupt routed to the INT1 pin.
@param enabled The new enabled state of the AOR1 interrupt routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_AOI1_BIT
*/
void LSM303DLHC_setAccelINT1AOI1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_AOI1_BIT, enabled);
}

/*Get whether the AOR1 interrupt is routed to the INT1 pin.
@return True if the AOR1 interrupt is routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_AOI1_BIT
*/
bool LSM303DLHC_getAccelINT1AOI1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_AOI1_BIT, buffer);
  return buffer[0];
}

/*Enable the AOR2 interrupt routed to the INT1 pin.
@param enabled The new enabled state of the AOR2 interrupt routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_AOI2_BIT
*/
void LSM303DLHC_setAccelINT1AOI2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_AOI2_BIT, enabled);
}

/*Get whether the AOR2 interrupt is routed to the INT1 pin.
@return True if the AOR2 interrupt is routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_AOI2_BIT
*/
bool LSM303DLHC_getAccelINT1AOI2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_AOI2_BIT, buffer);
  return buffer[0];
}

/*Enable the Data Ready 1 interrupt routed to the INT1 pin.
@param enabled The new enabled state of the Data Ready 1 interrupt routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_DRDY1_BIT
*/
void LSM303DLHC_setAccelINT1DataReady1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_DRDY1_BIT, enabled);
}

/*Get whether the Data Ready 1 interrupt is routed to the INT1 pin.
@return True if the Data Ready 1 interrupt is routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_DRDY1_BIT
*/
bool LSM303DLHC_getAccelINT1DataReady1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_DRDY1_BIT, buffer);
  return buffer[0];
}

/*Enable the Data Ready 2 interrupt routed to the INT1 pin.
@param enabled The new enabled state of the Data Ready 2 interrupt routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_DRDY2_BIT
*/
void LSM303DLHC_setAccelINT1DataReady2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_DRDY2_BIT, enabled);
}

/*Get whether the Data Ready 2 interrupt is routed to the INT1 pin.
@return True if the Data Ready 2 interrupt is routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_DRDY2_BIT
*/
bool LSM303DLHC_getAccelINT1DataReady2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_DRDY2_BIT, buffer);
  return buffer[0];
}

/*Enable the FIFO watermark interrupt routed to the INT1 pin.
@param enabled The new enabled state of the FIFO watermark interrupt routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_WTM_BIT
*/
void LSM303DLHC_setAccelINT1FIFOWatermarkEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_WTM_BIT, enabled);
}

/*Get whether the FIFO watermark interrupt is routed to the INT1 pin.
@return True if the FIFO watermark interrupt is routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_WTM_BIT
*/
bool LSM303DLHC_getAccelINT1FIFOWatermarkEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_WTM_BIT, buffer);
  return buffer[0];
}

/*Enable the FIFO overrun interrupt routed to the INT1 pin.
@param enabled The new enabled state of the FIFO overrun interrupt routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_OVERRUN_BIT
*/
void LSM303DLHC_setAccelINT1FIFOOverunEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_OVERRUN_BIT, enabled);
}

/*Get whether the FIFO overrun interrupt is routed to the INT1 pin.
@return True if the FIFO overrun interrupt is routed to the INT1 pin
@see LSM303DLHC_RA_CTRL_REG3_A
@see LSM303DLHC_I1_OVERRUN_BIT
*/
bool LSM303DLHC_getAccelINT1FIFOOverunEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG3_A, LSM303DLHC_I1_OVERRUN_BIT, buffer);
  return buffer[0];
}

// CTRL_REG4_A r/w

/** Set the Block Data Update (BDU) enabled state
 * @param enabled New BDU enabled state
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_BDU_BIT
 */
void LSM303DLHC_setAccelBlockDataUpdateEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_BDU_BIT, enabled);
}

/** Get the BDU enabled state
 * @return True if Block Data Update is enabled, false otherwise
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_BDU_BIT
 */
bool LSM303DLHC_getAccelBlockDataUpdateEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_BDU_BIT, buffer);
  return buffer[0];
}

/** Set the data endian modes
 * In Big Endian mode, the Most Significat Byte (MSB) is on the lower address,
 * and the Least Significant Byte (LSB) is on the higher address. Little Endian
 * mode reverses this order. Little Endian is the default mode.
 * @param endianness New endian mode
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_BLE_BIT
 * @see LSM303DLHC_BIG_ENDIAN
 * @see LSM303DLHC_LITTLE_ENDIAN
 */
void LSM303DLHC_setAccelEndianMode(bool endianness)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_BLE_BIT,
           endianness);
  endianMode = LSM303DLHC_getAccelEndianMode();
}

/** Get the data endian mode
 * @return Current endian mode
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_BLE_BIT
 * @see LSM303DLHC_BIG_ENDIAN
 * @see LSM303DLHC_LITTLE_ENDIAN
 */
bool LSM303DLHC_getAccelEndianMode()
{
  return endianMode;
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_BLE_BIT,
          buffer);
  return buffer[0];
}

/** Set the full scale of the data output (in dps)
 * @param scale The new scale of the data output (250, 500, 2000)
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_FS_BIT
 * @see LSM303DLHC_FS_LENGTH
 * @see LSM303DLHC_FS_2
 * @see LSM303DLHC_FS_4
 * @see LSM303DLHC_FS_8
 * @see LSM303DLHC_FS_16
 */
void LSM303DLHC_setAccelFullScale(uint8_t scale)
{
  uint8_t writeBitsv;

  if (scale == 2)
  {
    writeBitsv = LSM303DLHC_FS_2;
  }
  else if (scale == 4)
  {
    writeBitsv = LSM303DLHC_FS_4;
  }
  else if (scale == 8)
  {
    writeBitsv = LSM303DLHC_FS_8;
  }
  else
  {
    writeBitsv = LSM303DLHC_FS_16;
  }

  writeBits(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_FS_BIT,
            LSM303DLHC_FS_LENGTH, writeBitsv);
}

/** Get the current full scale of the output data (in dps)
 * @return Current scale of the output data
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_FS_BIT
 * @see LSM303DLHC_FS_LENGTH
 * @see LSM303DLHC_FS_2
 * @see LSM303DLHC_FS_4
 * @see LSM303DLHC_FS_8
 * @see LSM303DLHC_FS_16
 */
uint8_t LSM303DLHC_getAccelFullScale()
{
  readBits(devAddrA, LSM303DLHC_RA_CTRL_REG4_A,
           LSM303DLHC_FS_BIT, LSM303DLHC_FS_LENGTH, buffer);
  uint8_t readBits = buffer[0];

  if (readBits == LSM303DLHC_FS_2)
  {
    return 2;
  }
  else if (readBits == LSM303DLHC_FS_4)
  {
    return 4;
  }
  else if (readBits == LSM303DLHC_FS_8)
  {
    return 8;
  }
  else
  {
    return 16;
  }
}

/*Enables or disables high resolution output.
@param enabled New enabled state of high resolution output
@see LSM303DLHC_RA_CTRL_REG4_A
@see LSM303DLHC_HR_BIT
*/
void LSM303DLHC_setAccelHighResOutputEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_HR_BIT,
           enabled);
}

/*Gets whether high resolution output is enabled.
@return True if high resolution output is enabled
@see LSM303DLHC_RA_CTRL_REG4_A
@see LSM303DLHC_HR_BIT
*/
bool LSM303DLHC_getAccelHighResOutputEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_HR_BIT,
          buffer);
  return buffer[0];
}

/** Set the SPI mode
 * @param mode New SPI mode
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_SIM_BIT
 * @see LSM303DLHC_SIM_4W
 * @see LSM303DLHC_SIM_3W
 */
void LSM303DLHC_setAccelSPIMode(bool mode)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_SIM_BIT, mode);
}

/** Get the SPI mode
 * @return Current SPI mode
 * @see LSM303DLHC_RA_CTRL_REG4_A
 * @see LSM303DLHC_SIM_BIT
 * @see LSM303DLHC_SIM_4W
 * @see LSM303DLHC_SIM_3W
 */
bool LSM303DLHC_getAccelSPIMode()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG4_A, LSM303DLHC_SIM_BIT,
          buffer);
  return buffer[0];
}

// CTRL_REG5_A r/w

/** Reboots the FIFO memory content
 * @see LSM303DLHC_RA_CTRL_REG5_A
 * @see LSM303DLHC_BOOT_BIT
 */
void LSM303DLHC_rebootAccelMemoryContent()
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_BOOT_BIT, true);
}

/** Set whether the FIFO buffer is enabled
 * @param enabled New enabled state of the FIFO buffer
 * @see LSM303DLHC_RA_CTRL_REG5_A
 * @see LSM303DLHC_FIFO_EN_BIT
 */
void LSM303DLHC_setAccelFIFOEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_FIFO_EN_BIT,
           enabled);
}

/** Get whether the FIFO buffer is enabled
 * @return True if the FIFO buffer is enabled, false otherwise
 * @see LSM303DLHC_RA_CTRL_REG5_A
 * @see LSM303DLHC_FIFO_EN_BIT
 */
bool LSM303DLHC_getAccelFIFOEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_FIFO_EN_BIT,
          buffer);
  return buffer[0];
}

/*Enable latching of interrupt requrest 1
@param latched New enabled state of latching interrupt request 1
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_LIR_INT1_BIT
*/
void LSM303DLHC_setAccelInterrupt1RequestLatched(bool latched)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_LIR_INT1_BIT,
           latched);
}

/*Get whether latching of interrupt request 1 is enabled.
@return True if latching of interrupt request 1 is enabled
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_LIR_INT1_BIT
*/
bool LSM303DLHC_getAccelInterrupt1RequestLatched()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_LIR_INT1_BIT,
          buffer);
  return buffer[0];
}

/*Enable latching of interrupt requrest 2
@param latched New enabled state of latching interrupt request 2
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_LIR_INT2_BIT
*/
void LSM303DLHC_setAccelInterrupt2RequestLatched(bool latched)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_LIR_INT2_BIT,
           latched);
}

/*Get whether latching of interrupt request 2 is enabled.
@return True if latching of interrupt request 2 is enabled
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_LIR_INT2_BIT
*/
bool LSM303DLHC_getAccelInterrupt2RequestLatched()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_LIR_INT2_BIT,
          buffer);
  return buffer[0];
}

/*Enable 4D dectection interrupt 1
@param enabled New enabled state of the 4D detection interrupt 1
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_D4D_INT1_BIT
*/
void LSM303DLHC_setAccelDetect4DInterrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_D4D_INT1_BIT,
           enabled);
}

/*Get whether 4D detection interrupt 1 is enabled.
@return True if 4D detection interrupt 1 is enabled
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_D4D_INT1_BIT
*/
bool LSM303DLHC_getAccelDetect4DInterrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_D4D_INT1_BIT,
          buffer);
  return buffer[0];
}

/*Enable 4D dectection interrupt 2
@param enabled New enabled state of the 4D detection interrupt 2
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_D4D_INT2_BIT
*/
void LSM303DLHC_setAccelDetect4DInterrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_D4D_INT2_BIT,
           enabled);
}

/*Get whether 4D detection interrupt 2 is enabled.
@return True if 4D detection interrupt 2 is enabled
@see LSM303DLHC_RA_CTRL_REG5_A
@see LSM303DLHC_D4D_INT2_BIT
*/
bool LSM303DLHC_getAccelDetect4DInterrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG5_A, LSM303DLHC_D4D_INT2_BIT,
          buffer);
  return buffer[0];
}

// CTRL_REG6_A r/w

// TODO:
/*Enable the Click interrupt routed to the INT2 pin.
@param enabled The new enabled state of the Click interrupt routed to the INT2 pin
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_I2_CLICK_BIT
*/
void LSM303DLHC_setAccelINT2ClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_I2_CLICK_BIT, enabled);
}

/*Get whether the Click interrupt is routed to the INT2 pin.
@return True if the Click interrupt is routed to the INT2 pin
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_I2_CLICK_BIT
*/
bool LSM303DLHC_getAccelINT2ClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_I2_CLICK_BIT, buffer);
  return buffer[0];
}

/*Enable interrupt 1 routed to the INT2 pin.
@param enabled The new enabled state of whether interrupt 1 routed to the INT2 pin
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_I2_INT1_BIT
*/
void LSM303DLHC_setAccelINT2Interrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_I2_INT1_BIT, enabled);
}

/*Get whether interrupt 1 is routed to the INT2 pin.
@return True if interrupt 1 is routed to the INT2 pin
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_I2_INT1_BIT
*/
bool LSM303DLHC_getAccelINT2Interrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_I2_INT1_BIT, buffer);
  return buffer[0];
}

/*Enable interrupt 2 routed to the INT2 pin.
@param enabled The new enabled state of whether interrupt 2 routed to the INT2 pin
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_I2_INT2_BIT
*/
void LSM303DLHC_setAccelINT2Interrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_I2_INT2_BIT, enabled);
}

/*Get whether interrupt 2 is routed to the INT2 pin.
@return True if interrupt 2 is routed to the INT2 pin
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_I2_INT2_BIT
*/
bool LSM303DLHC_getAccelINT2Interrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_I2_INT2_BIT, buffer);
  return buffer[0];
}

/*Enable memory content reboot via INT2 pin
@param enabled The new state of whether memory content is rebooted via INT2 pin
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_BOOT_I1_BIT
*/
void LSM303DLHC_setAccelRebootMemoryContentINT2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_BOOT_I1_BIT, enabled);
}

/*Get whether memory content reboot is enabled via INT2 pin
@return True if memory content reboot is via INT2 pin is enabled
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_BOOT_I1_BIT
*/
bool LSM303DLHC_getAccelRebootMemoryContentINT2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_BOOT_I1_BIT, buffer);
  return buffer[0];
}

// // TODO Documentation
// void setAccelActiveFunctionStatusINT2Enabled(bool enabled);

// // TODO Documentation
// bool getAccelActiveFunctionStatusINT2Enabled();

/*Enable active low interrupts.
@param enable The new state of whether active low interrupts are enabled
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_H_ACTIVE_BIT
*/
void LSM303DLHC_setAccelInterruptActiveLowEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_H_ACTIVE_BIT, enabled);
}

/*Get whether active low interrupts are enabled.
@return True if active low interrupts are enabled.
@see LSM303DLHC_RA_CTRL_REG6_A
@see LSM303DLHC_H_ACTIVE_BIT
*/
bool LSM303DLHC_getAccelInterruptActiveLowEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CTRL_REG6_A, LSM303DLHC_H_ACTIVE_BIT, buffer);
  return buffer[0];
}

// REFERENCE_A, r/w

/** Set the reference value for interrupt generation
 * @param reference New reference value for interrupt generation
 * @see LSM303DLHC_RA_REFERENCE_A
 */
void LSM303DLHC_setAccelInterruptReference(uint8_t reference)
{
  writeByte(devAddrA, LSM303DLHC_RA_REFERENCE_A, reference);
}

/** Get the 8-bit reference value for interrupt generation
 * @return 8-bit reference value for interrupt generation
 * @see LSM303DLHC_RA_REFERENCE
 */
uint8_t LSM303DLHC_getAccelInterruptReference()
{
  readByte(devAddrA, LSM303DLHC_RA_REFERENCE_A, buffer);
  return buffer[0];
}

// STATUS_REG_A r

/** Get whether new data overwrote the last set of data before it was read
 * @return True if the last set of data was overwritten before being read, false
 * otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_ZYXOR_BIT
 */
bool LSM303DLHC_getAccelXYZOverrun()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_ZYXOR_BIT,
          buffer);
  return buffer[0];
}

/** Get whether new Z data overwrote the last set of data before it was read
 * @return True if the last set of Z data was overwritten before being read,
 * false otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_ZOR_BIT
 */
bool LSM303DLHC_getAccelZOverrun()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_ZOR_BIT,
          buffer);
  return buffer[0];
}

/** Get whether new Y data overwrote the last set of data before it was read
 * @return True if the last set of Y data was overwritten before being read,
 * false otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_YOR_BIT
 */
bool LSM303DLHC_getAccelYOverrun()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_YOR_BIT,
          buffer);
  return buffer[0];
}

/** Get whether new X data overwrote the last set of data before it was read
 * @return True if the last set of X data was overwritten before being read,
 * false otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_XOR_BIT
 */
bool LSM303DLHC_getAccelXOverrun()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_XOR_BIT,
          buffer);
  return buffer[0];
}

/** Get whether there is new data avaialable
 * @return True if there is new data available, false otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_ZYXDA_BIT
 */
bool LSM303DLHC_getAccelXYZDataAvailable()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_ZYXDA_BIT,
          buffer);
  return buffer[0];
}

/** Get whether there is new Z data avaialable
 * @return True if there is new Z data available, false otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_ZDA_BIT
 */
bool LSM303DLHC_getAccelZDataAvailable()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_ZDA_BIT,
          buffer);
  return buffer[0];
}

/** Get whether there is new Y data avaialable
 * @return True if there is new Y data available, false otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_YDA_BIT
 */
bool LSM303DLHC_getAccelYDataAvailable()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_YDA_BIT,
          buffer);
  return buffer[0];
}

/** Get whether there is new X data avaialable
 * @return True if there is new X data available, false otherwise
 * @see LSM303DLHC_RA_STATUS_REG_A
 * @see LSM303DLHC_XDA_BIT
 */
bool LSM303DLHC_getAccelXDataAvailable()
{
  readBit(devAddrA, LSM303DLHC_RA_STATUS_REG_A, LSM303DLHC_XDA_BIT,
          buffer);
  return buffer[0];
}

// OUT_*_A, r

// TODO:
// Fix getAccerlation to read all 6 regs, and fix endian internal state

/** Get the acceleration for all 3 axes
 * Due to the fact that this device supports two difference Endian modes, both
 * must be accounted for when reading data. In Little Endian mode, the first
 * byte (lowest address) is the least significant and in Big Endian mode the
 * first byte is the most significant.
 * @param x 16-bit integer container for the X-axis acceleration
 * @param y 16-bit integer container for the Y-axis acceleration
 * @param z 16-bit integer container for the Z-axis acceleration
 */
esp_err_t LSM303DLHC_getAcceleration(int16_t *x, int16_t *y, int16_t *z)
{
  esp_err_t ret = readBytes(devAddrA, LSM303DLHC_RA_OUT_X_L_A | 0x80, 6, buffer);
  if (endianMode == LSM303DLHC_LITTLE_ENDIAN)
  {
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
  }
  else
  {
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
  }
  return ret;
}

/** Get the acceleration along the X-axis
 * @return acceleration along the X-axis
 * @see LSM303DLHC_RA_OUT_X_L_A
 * @see LSM303DLHC_RA_OUT_X_H_A
 */
int16_t LSM303DLHC_getAccelerationX()
{
  readBytes(devAddrA, LSM303DLHC_RA_OUT_X_L_A | 0x80, 2, buffer);
  if (endianMode == LSM303DLHC_LITTLE_ENDIAN)
  {
    return (((int16_t)buffer[1]) << 8) | buffer[0];
  }
  else
  {
    return (((int16_t)buffer[0]) << 8) | buffer[1];
  }
}

/** Get the acceleration along the Y-axis
 * @return acceleration along the Y-axis
 * @see LSM303DLHC_RA_OUT_Y_L_A
 * @see LSM303DLHC_RA_OUT_Y_H_A
 */
int16_t LSM303DLHC_getAccelerationY()
{
  readBytes(devAddrA, LSM303DLHC_RA_OUT_Y_L_A | 0x80, 2, buffer);
  if (endianMode == LSM303DLHC_LITTLE_ENDIAN)
  {
    return (((int16_t)buffer[1]) << 8) | buffer[0];
  }
  else
  {
    return (((int16_t)buffer[0]) << 8) | buffer[1];
  }
}

/** Get the acceleration along the Z-axis
 * @return acceleration along the Z-axis
 * @see LSM303DLHC_RA_OUT_Z_L_A
 * @see LSM303DLHC_RA_OUT_Z_H_A
 */
int16_t LSM303DLHC_getAccelerationZ()
{
  readBytes(devAddrA, LSM303DLHC_RA_OUT_Z_L_A | 0x80, 2, buffer);
  if (endianMode == LSM303DLHC_LITTLE_ENDIAN)
  {
    return (((int16_t)buffer[1]) << 8) | buffer[0];
  }
  else
  {
    return (((int16_t)buffer[0]) << 8) | buffer[1];
  }
}

// FIFO_CTRL_REG_A, rw

/** Set the FIFO mode to one of the defined modes
 * @param mode New FIFO mode
 * @see LSM303DLHC_RA_FIFO_CTRL_REG_A
 * @see LSM303DLHC_FM_BIT
 * @see LSM303DLHC_FM_LENGTH
 * @see LSM303DLHC_FM_BYPASS
 * @see LSM303DLHC_FM_FIFO
 * @see LSM303DLHC_FM_STREAM
 * @see LSM303DLHC_FM_TRIGGER
 */
void LSM303DLHC_setAccelFIFOMode(uint8_t mode)
{
  writeBits(devAddrA, LSM303DLHC_RA_FIFO_CTRL_REG_A, LSM303DLHC_FM_BIT,
            LSM303DLHC_FM_LENGTH, mode);
}

/** Get the FIFO mode to one of the defined modes
 * @return Current FIFO mode
 * @see LSM303DLHC_RA_FIFO_CTRL_REG_A
 * @see LSM303DLHC_FM_BIT
 * @see LSM303DLHC_FM_LENGTH
 * @see LSM303DLHC_FM_BYPASS
 * @see LSM303DLHC_FM_FIFO
 * @see LSM303DLHC_FM_STREAM
 * @see LSM303DLHC_FM_TRIGGER
 */
uint8_t LSM303DLHC_getAccelFIFOMode()
{
  readBits(devAddrA, LSM303DLHC_RA_FIFO_CTRL_REG_A,
           LSM303DLHC_FM_BIT, LSM303DLHC_FM_LENGTH, buffer);
  return buffer[0];
}

/*Set the trigger selection to INT1 or INT2
@param trigger The new trigger selection
@see LSM303DLHC_RA_FIFO_CTRL_REG_A
@see LSM303DLHC_TR_BIT
@see LSM303DLHC_TR_INT1
@see LSM303DLHC_TR_INT2
*/
void LSM303DLHC_setAccelFIFOTriggerINT(bool trigger)
{
  writeBit(devAddrA, LSM303DLHC_RA_FIFO_CTRL_REG_A,
           LSM303DLHC_TR_BIT, trigger);
}

/*Get the trigger selection (INT1 or INT2)
@return trigger The new trigger selection (0 for INT1, 1 for INT2)
@see LSM303DLHC_RA_FIFO_CTRL_REG_A
@see LSM303DLHC_TR_BIT
@see LSM303DLHC_TR_INT1
@see LSM303DLHC_TR_INT2
*/
bool LSM303DLHC_getAccelFIFOTriggerINT()
{
  readBit(devAddrA, LSM303DLHC_RA_FIFO_CTRL_REG_A,
          LSM303DLHC_TR_BIT, buffer);
  return buffer[0];
}

/** Set the FIFO watermark threshold
 * @param wtm New FIFO watermark threshold
 * @see LSM303DLHC_RA_FIFO_CTRL_REG_A
 * @see LSM303DLHC_FTH_BIT
 * @see LSM303DLHC_FTH_LENGTH
 */
void LSM303DLHC_setAccelFIFOThreshold(uint8_t wtm)
{
  writeBits(devAddrA, LSM303DLHC_RA_FIFO_CTRL_REG_A, LSM303DLHC_FTH_BIT,
            LSM303DLHC_FTH_LENGTH, wtm);
}

/** Get the FIFO watermark threshold
 * @return FIFO watermark threshold
 * @see LSM303DLHC_RA_FIFO_CTRL_REG_A
 * @see LSM303DLHC_FTH_BIT
 * @see LSM303DLHC_FTH_LENGTH
 */
uint8_t LSM303DLHC_getAccelFIFOThreshold()
{
  readBits(devAddrA, LSM303DLHC_RA_FIFO_CTRL_REG_A, LSM303DLHC_FTH_BIT,
           LSM303DLHC_FTH_LENGTH, buffer);
  return buffer[0];
}

// FIFO_SRC_REG_A, r

/** Get whether the number of data sets in the FIFO buffer is less than the
 * watermark
 * @return True if the number of data sets in the FIFO buffer is more than or
 * equal to the watermark, false otherwise.
 * @see LSM303DLHC_RA_FIFO_SRC_REG_A
 * @see LSM303DLHC_WTM_BIT
 */
bool LSM303DLHC_getAccelFIFOAtWatermark()
{
  readBit(devAddrA, LSM303DLHC_RA_FIFO_SRC_REG_A, LSM303DLHC_WTM_BIT,
          buffer);
  return buffer[0];
}

/** Get whether the FIFO buffer is full
 * @return True if the FIFO buffer is full, false otherwise
 * @see LSM303DLHC_RA_FIFO_SRC_REG_A
 * @see LSM303DLHC_OVRN_FIFO_BIT
 */
bool LSM303DLHC_getAccelFIFOOverrun()
{
  readBit(devAddrA, LSM303DLHC_RA_FIFO_SRC_REG_A,
          LSM303DLHC_OVRN_FIFO_BIT, buffer);
  return buffer[0];
}

/** Get whether the FIFO buffer is empty
 * @return True if the FIFO buffer is empty, false otherwise
 * @see LSM303DLHC_RA_FIFO_SRC_REG_A
 * @see LSM303DLHC_EMPTY_BIT
 */
bool LSM303DLHC_getAccelFIFOEmpty()
{
  readBit(devAddrA, LSM303DLHC_RA_FIFO_SRC_REG_A,
          LSM303DLHC_EMPTY_BIT, buffer);
  return buffer[0];
}

/** Get the number of filled FIFO buffer slots
 * @return Number of filled slots in the FIFO buffer
 * @see LSM303DLHC_RA_FIFO_SRC_REG_A
 * @see LSM303DLHC_FSS_BIT
 * @see LSM303DLHC_FSS_LENGTH
 */
uint8_t LSM303DLHC_getAccelFIFOStoredSamples()
{
  readBits(devAddrA, LSM303DLHC_RA_FIFO_SRC_REG_A,
           LSM303DLHC_FSS_BIT, LSM303DLHC_FSS_LENGTH, buffer);
  return buffer[0];
}

// INT1_CFG_A, w/r

/** Set the combination mode for interrupt 1events
 * @param combination New combination mode for interrupt 1 events.
 * LSM303DLHC_INT1_OR for OR and LSM303DLHC_INT1_AND for AND
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_AOI_BIT
 * @see LSM303DLHC_INT1_OR
 * @see LSM303DLHC_INT1_AND
 */
void LSM303DLHC_setAccelInterrupt1Combination(bool combination)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_AOI_BIT,
           combination);
}

/** Get the combination mode for interrupt 1 events
 * @return Combination mode for interrupt 1 events. LSM303DLHC_INT1_OR for OR and
 * LSM303DLHC_INT1_AND for AND
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_AOI_BIT
 * @see LSM303DLHC_INT1_OR
 * @see LSM303DLHC_INT1_AND
 */
bool LSM303DLHC_getAccelInterrupt1Combination()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_AOI_BIT,
          buffer);
  return buffer[0];
}

/*Enable 6D dectection interrupt 1.  See datasheet for how 4D detection is affected.
@param enabled New enabled state of the 6D detection interrupt 1
@see LSM303DLHC_RA_INT1_CFG_A
@see LSM303DLHC_INT1_6D_BIT
*/
void LSM303DLHC_setAccelInterrupt16DEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_6D_BIT, enabled);
}

/*Get enable status of 6D dectection interrupt 1.  See datasheet for how 4D detection is affected.
@return True if 6D detection interrupt 1 is enabled
@see LSM303DLHC_RA_INT1_CFG_A
@see LSM303DLHC_INT1_6D_BIT
*/
bool LSM303DLHC_getAccelInterrupt16DEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_6D_BIT, buffer);
  return buffer[0];
}

/** Set whether the interrupt 1 for Z high/up is enabled
 * @param enabled New enabled state for Z high/up interrupt.
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_ZHIE_ZUPE_BIT
 */
void LSM303DLHC_setAccelZHighUpInterrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_ZHIE_ZUPE_BIT, enabled);
}

/** Get whether the interrupt 1 for Z high/up is enabled
 * @return True if the interrupt 1 for Z high/up is enabled, false otherwise
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_ZHIE_ZUPE_BIT
 */
bool LSM303DLHC_getAccelZHighUpInterrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_ZHIE_ZUPE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 1 for Z low/down is enabled
 * @param enabled New enabled state for Z low/down interrupt.
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_ZLIE_ZDOWNE_BIT
 */
void LSM303DLHC_setAccelZLowDownInterrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_ZLIE_ZDOWNE_BIT, enabled);
}

/** Get whether the interrupt 1 for Z low/down is enabled
 * @return True if the interrupt 1 for Z low/down is enabled, false otherwise
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_ZLIE_ZDOWNE_BIT
 */
bool LSM303DLHC_getAccelZLowDownInterrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_ZLIE_ZDOWNE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 1 for Y high/up is enabled
 * @param enabled New enabled state for Y high/up interrupt.
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_YHIE_YUPE_BIT
 */
void LSM303DLHC_setAccelYHighUpInterrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_YHIE_YUPE_BIT, enabled);
}

/** Get whether the interrupt 1 for Y high/up is enabled
 * @return True if the interrupt 1 for Y high/up is enabled, false otherwise
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_YHIE_YUPE_BIT
 */
bool LSM303DLHC_getAccelYHighUpInterrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_YHIE_YUPE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 1 for Y low/down is enabled
 * @param enabled New enabled state for Y low/down interrupt.
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_YLIE_YDOWNE_BIT
 */
void LSM303DLHC_setAccelYLowDownInterrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_YLIE_YDOWNE_BIT, enabled);
}

/** Get whether the interrupt 1 for Y low/down is enabled
 * @return True if the interrupt 1 for Y low/down is enabled, false otherwise
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_YLIE_YDOWNE_BIT
 */
bool LSM303DLHC_getAccelYLowDownInterrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_YLIE_YDOWNE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 1 for X high/up is enabled
 * @param enabled New enabled state for X high/up interrupt.
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_XHIE_XUPE_BIT
 */
void LSM303DLHC_setAccelXHighUpInterrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_XHIE_XUPE_BIT, enabled);
}

/** Get whether the interrupt 1 for X high/up is enabled
 * @return True if the interrupt 1 for X high/up is enabled, false otherwise
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_XHIE_XUPE_BIT
 */
bool LSM303DLHC_getAccelXHighUpInterrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_XHIE_XUPE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 1 for X low/down is enabled
 * @param enabled New enabled state for/down X low interrupt.
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_XLIE_XDOWNE_BIT
 */
void LSM303DLHC_setAccelXLowDownInterrupt1Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_XLIE_XDOWNE_BIT, enabled);
}

/** Get whether the interrupt 1 for X low/down is enabled
 * @return True if the interrupt 1 for X low/down is enabled, false otherwise
 * @see LSM303DLHC_RA_INT1_CFG_A
 * @see LSM303DLHC_INT1_XLIE_XDOWNE_BIT
 */
bool LSM303DLHC_getAccelXLowDownInterrupt1Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT1_CFG_A, LSM303DLHC_INT1_XLIE_XDOWNE_BIT,
          buffer);
  return buffer[0];
}

// INT1_SRC_A, r

/*Get the contents of the INT1_SRC_A register to determine if any and which interrupts have occured.
Reading this register will clear any interrupts that were lateched.
@return the 8 bit contents of the INT1_SRC_a register
@see LSM303DLHC_RA_INT1_SRC_A
@see LSM303DLHC_INT1_IA_BIT
@see LSM303DLHC_INT1_ZH_BIT
@see LSM303DLHC_INT1_ZL_BIT
@see LSM303DLHC_INT1_YH_BIT
@see LSM303DLHC_INT1_YL_BIT
@see LSM303DLHC_INT1_XH_BIT
@see LSM303DLHC_INT1_XL_BIT
*/
uint8_t LSM303DLHC_getAccelInterrupt1Source()
{
  readByte(devAddrA, LSM303DLHC_RA_INT1_SRC_A, buffer);
  return buffer[0];
}

// INT1_THS_A, w/r

/** Set the threshold for interrupt 1
 * @param threshold New threshold for interrupt 1
 * @see LSM303DLHC_RA_INT1_THS_A
 */
void LSM303DLHC_setAccelInterrupt1Threshold(uint8_t value)
{
  writeByte(devAddrA, LSM303DLHC_RA_INT1_THS_A, value);
}

/** Retrieve the threshold interrupt 1
 * @return interrupt 1 threshold
 * @see LSM303DLHC_RA_INT1_THS_A
 */
uint8_t LSM303DLHC_getAccelInterupt1Threshold()
{
  readByte(devAddrA, LSM303DLHC_RA_INT1_THS_A, buffer);
  return buffer[0];
}

// INT1_DURATION_A, r/w

/* Set the minimum event duration for interrupt 1 to be generated
 * This depends on the chosen output data rate
 * @param duration New duration necessary for interrupt 1 to be
 * generated
 * @see LSM303DLHC_RA_INT1_DURATION_A
 * @see LSM303DLHC_INT1_DURATION_BIT
 * @see LSM303DLHC_INT1_DURATION_LENGTH
 */
void LSM303DLHC_setAccelInterrupt1Duration(uint8_t duration)
{
  writeBits(devAddrA, LSM303DLHC_RA_INT1_DURATION_A, LSM303DLHC_INT1_DURATION_BIT,
            LSM303DLHC_INT1_DURATION_LENGTH, duration);
}

/** Get the minimum event duration for interrupt 1 to be generated
 * @return Duration necessary for interrupt 1 to be generated
 * @see LSM303DLHC_RA_INT1_DURATION_A
 * @see LSM303DLHC_INT1_DURATION_BIT
 * @see LSM303DLHC_INT1_DURATION_LENGTH
 */
uint8_t LSM303DLHC_getAccelInterrupt1Duration()
{
  readBits(devAddrA, LSM303DLHC_RA_INT1_DURATION_A,
           LSM303DLHC_INT1_DURATION_BIT, LSM303DLHC_INT1_DURATION_LENGTH, buffer);
  return buffer[0];
}

// INT2_CFG_A, r/w

/** Set the combination mode for interrupt 2 events
 * @param combination New combination mode for interrupt events.
 * LSM303DLHC_INT1_OR for OR and LSM303DLHC_INT1_AND for AND
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_AOI_BIT
 * @see LSM303DLHC_INT1_OR
 * @see LSM303DLHC_INT1_AND
 */
void LSM303DLHC_setAccelInterrupt2Combination(bool combination)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_AOI_BIT,
           combination);
}

/** Get the combination mode for interrupt 2 events
 * @return Combination mode for interrupt events. LSM303DLHC_INT1_OR for OR and
 * LSM303DLHC_INT1_AND for AND
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_AOI_BIT
 * @see LSM303DLHC_INT1_OR
 * @see LSM303DLHC_INT1_AND
 */
bool LSM303DLHC_getAccelInterrupt2Combination()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_AOI_BIT,
          buffer);
  return buffer[0];
}

/*Enable 6D dectection interrupt 2.  See datasheet for how 4D detection is affected.
@param enabled New enabled state of the 6D detection interrupt 1
@see LSM303DLHC_RA_INT2_CFG_A
@see LSM303DLHC_INT2_6D_BIT
*/
void LSM303DLHC_setAccelInterrupt26DEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_6D_BIT, enabled);
}

/*Get enable status of 6D dectection interrupt 2.  See datasheet for how 4D detection is affected.
@return True if 6D detection interrupt 1 is enabled
@see LSM303DLHC_RA_INT2_CFG_A
@see LSM303DLHC_INT2_6D_BIT
*/
bool LSM303DLHC_getAccelInterrupt26DEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_6D_BIT, buffer);
  return buffer[0];
}

/** Set whether the interrupt 2 for Z high is enabled
 * @param enabled New enabled state for Z high interrupt 2.
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_ZHIE_BIT
 */
void LSM303DLHC_setAccelZHighInterrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_ZHIE_BIT, enabled);
}

/** Get whether the interrupt 2 for Z high is enabled
 * @return True if the interrupt 2 for Z high is enabled, false otherwise
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_ZHIE_BIT
 */
bool LSM303DLHC_getAccelZHighInterrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_ZHIE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 2 for Z low is enabled
 * @param enabled New enabled state for Z low interrupt 2.
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_ZLIE_BIT
 */
void LSM303DLHC_setAccelZLowInterrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_ZLIE_BIT, enabled);
}

/** Get whether the interrupt 2 for Z low is enabled
 * @return True if the interrupt 2 for Z low is enabled, false otherwise
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_ZLIE_BIT
 */
bool LSM303DLHC_getAccelZLowInterrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_ZLIE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 2 for Y high is enabled
 * @param enabled New enabled state for Y high interrupt 2.
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_YHIE_BIT
 */
void LSM303DLHC_setAccelYHighInterrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_YHIE_BIT, enabled);
}

/** Get whether the interrupt 2 for Y high is enabled
 * @return True if the interrupt 2 for Y high is enabled, false otherwise
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_YHIE_BIT
 */
bool LSM303DLHC_getAccelYHighInterrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_YHIE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 2 for Y low is enabled
 * @param enabled New enabled state for Y low interrupt 2.
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_YLIE_BIT
 */
void LSM303DLHC_setAccelYLowInterrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_YLIE_BIT, enabled);
}

/** Get whether the interrupt 2 for Y low is enabled
 * @return True if the interrupt 2 for Y low is enabled, false otherwise
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_YLIE_BIT
 */
bool LSM303DLHC_getAccelYLowInterrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_YLIE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 2 for X high is enabled
 * @param enabled New enabled state for X high interrupt 2.
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_XHIE_BIT
 */
void LSM303DLHC_setAccelXHighInterrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_XHIE_BIT, enabled);
}

/** Get whether the interrupt 2 for X high is enabled
 * @return True if the interrupt 2 for X high is enabled, false otherwise
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_XHIE_BIT
 */
bool LSM303DLHC_getAccelXHighInterrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_XHIE_BIT,
          buffer);
  return buffer[0];
}

/** Set whether the interrupt 2 for X low is enabled
 * @param enabled New enabled state for X low interrupt 2.
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_XLIE_BIT
 */
void LSM303DLHC_setAccelXLowInterrupt2Enabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_XLIE_BIT, enabled);
}

/** Get whether the interrupt 2 for X low is enabled
 * @return True if the interrupt 2 for X low is enabled, false otherwise
 * @see LSM303DLHC_RA_INT2_CFG_A
 * @see LSM303DLHC_INT2_XLIE_BIT
 */
bool LSM303DLHC_getAccelXLowInterrupt2Enabled()
{
  readBit(devAddrA, LSM303DLHC_RA_INT2_CFG_A, LSM303DLHC_INT2_XLIE_BIT,
          buffer);
  return buffer[0];
}

// INT2_SRC_A, r

/*Get the contents of the INT2_SRC_A register to determine if any and which interrupts have occured.
Reading this register will clear any interrupts that were lateched.
@return the 8 bit contents of the INT2_SRC_a register
@see LSM303DLHC_RA_INT2_SRC_A
@see LSM303DLHC_INT2_IA_BIT
@see LSM303DLHC_INT2_ZH_BIT
@see LSM303DLHC_INT2_ZL_BIT
@see LSM303DLHC_INT2_YH_BIT
@see LSM303DLHC_INT2_YL_BIT
@see LSM303DLHC_INT2_XH_BIT
@see LSM303DLHC_INT2_XL_BIT
*/
uint8_t LSM303DLHC_getAccelInterrupt2Source()
{
  readByte(devAddrA, LSM303DLHC_RA_INT2_SRC_A, buffer);
  return buffer[0];
}

// INT2_THS_A, r/w

/** Set the threshold for interrupt 2
 * @param threshold New threshold for interrupt 2
 * @see LSM303DLHC_RA_INT2_THS_A
 */
void LSM303DLHC_setAccelInterrupt2Threshold(uint8_t value)
{
  writeByte(devAddrA, LSM303DLHC_RA_INT2_THS_A, value);
}

/** Retrieve the threshold interrupt 2
 * @return interrupt 2 threshold
 * @see LSM303DLHC_RA_INT2_THS_A
 */
uint8_t LSM303DLHC_getAccelInterupt2Threshold()
{
  readByte(devAddrA, LSM303DLHC_RA_INT2_THS_A, buffer);
  return buffer[0];
}

// INT2_DURATION_A, r/w

/* Set the minimum event duration for interrupt 2 to be generated
 * This depends on the chosen output data rate
 * @param duration New duration necessary for interrupt 2 to be
 * generated
 * @see LSM303DLHC_RA_INT2_DURATION_A
 * @see LSM303DLHC_INT2_DURATION_BIT
 * @see LSM303DLHC_INT2_DURATION_LENGTH
 */
void LSM303DLHC_setAccelInterrupt2Duration(uint8_t duration)
{
  writeBits(devAddrA, LSM303DLHC_RA_INT2_DURATION_A, LSM303DLHC_INT2_DURATION_BIT,
            LSM303DLHC_INT2_DURATION_LENGTH, duration);
}

/** Get the minimum event duration for interrupt 2 to be generated
 * @return Duration necessary for interrupt 2 to be generated
 * @see LSM303DLHC_RA_INT2_DURATION_A
 * @see LSM303DLHC_INT2_DURATION_BIT
 * @see LSM303DLHC_INT2_DURATION_LENGTH
 */
uint8_t LSM303DLHC_getAccelInterrupt2Duration()
{
  readBits(devAddrA, LSM303DLHC_RA_INT2_DURATION_A,
           LSM303DLHC_INT2_DURATION_BIT, LSM303DLHC_INT2_DURATION_LENGTH, buffer);
  return buffer[0];
}

// CLICK_CFG_A, r/w

/*Enables interrupt double click on the Z axis.
@param enable The new status of double click interrupt on the Z axis
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_ZD_BIT
*/
void LSM303DLHC_setAccelZDoubleClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_ZD_BIT, enabled);
}

/*Get status of interrupt double click on the Z axis.
@return True if double click interrupt on the Z axis is enabled
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_ZD_BIT
*/
bool LSM303DLHC_getAccelZDoubleClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_ZD_BIT, buffer);
  return buffer[0];
}

/*Enables interrupt single click on the Z axis.
@param enable The new status of single click interrupt on the Z axis
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_ZS_BIT
*/
void LSM303DLHC_setAccelZSingleClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_ZS_BIT, enabled);
}

/*Get status of interrupt single click on the Z axis.
@return True if single click interrupt on the Z axis is enabled
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_ZS_BIT
*/
bool LSM303DLHC_getAccelZSingleClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_ZS_BIT, buffer);
  return buffer[0];
}

/*Enables interrupt double click on the Y axis.
@param enable The new status of double click interrupt on the Y axis
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_YD_BIT
*/
void LSM303DLHC_setAccelYDoubleClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_YD_BIT, enabled);
}

/*Get status of interrupt double click on the Y axis.
@return True if double click interrupt on the Y axis is enabled
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_YD_BIT
*/
bool LSM303DLHC_getAccelYDoubleClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_YD_BIT, buffer);
  return buffer[0];
}

/*Enables interrupt single click on the Y axis.
@param enable The new status of single click interrupt on the Y axis
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_YS_BIT
*/
void LSM303DLHC_setAccelYSingleClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_YS_BIT, enabled);
}

/*Get status of interrupt single click on the Y axis.
@return True if single click interrupt on the Y axis is enabled
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_YS_BIT
*/
bool LSM303DLHC_getAccelYSingleClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_YS_BIT, buffer);
  return buffer[0];
}

/*Enables interrupt double click on the X axis.
@param enable The new status of double click interrupt on the X axis
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_XD_BIT
*/
void LSM303DLHC_setAccelXDoubleClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_XD_BIT, enabled);
}

/*Get status of interrupt double click on the X axis.
@return True if double click interrupt on the X axis is enabled
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_XD_BIT
*/
bool LSM303DLHC_getAccelXDoubleClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_XD_BIT, buffer);
  return buffer[0];
}

/*Enables interrupt single click on the X axis.
@param enable The new status of single click interrupt on the X axis
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_XS_BIT
*/
void LSM303DLHC_setAccelXSingleClickEnabled(bool enabled)
{
  writeBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_XS_BIT, enabled);
}

/*Get status of interrupt single click on the X axis.
@return True if single click interrupt on the X axis is enabled
@see LSM303DLHC_RA_CLICK_CFG_A
@see LSM303DLHC_CLICK_XS_BIT
*/

bool LSM303DLHC_getAccelXSingleClickEnabled()
{
  readBit(devAddrA, LSM303DLHC_RA_CLICK_CFG_A, LSM303DLHC_CLICK_XS_BIT, buffer);
  return buffer[0];
}

// CLICK_SRC_A, r

/*Get the contents of the CLICK_SRC_A register to determine if any and which interrupts have occured.
Reading this register may (not sure) clear any interrupts that were lateched.
@return the 7 bit contents of the CLICK_SRC_A register
@see LSM303DLHC_RA_CLICK_SRC_A
@see LSM303DLHC_CLICK_IA_BIT
@see LSM303DLHC_CLICK_DCLICK_BIT
@see LSM303DLHC_CLICK_SCLICK_BIT
@see LSM303DLHC_CLICK_SIGN_BIT
@see LSM303DLHC_CLICK_Z_BIT
@see LSM303DLHC_CLICK_Y_BIT
@see LSM303DLHC_CLICK_X_BIT
*/
uint8_t LSM303DLHC_getAccelClickSource()
{
  readByte(devAddrA, LSM303DLHC_RA_CLICK_SRC_A, buffer);
  return buffer[0];
}

// CLICK_THS_A, r/w

/*Set the threshold which is used to start the click-detection procedure. The threshold
value is expressed over 7 bits as an unsigned number.
@param value The value of the click detection threshold
@see LSM303DLHC_RA_CLICK_THS_A
*/
void LSM303DLHC_setAcceLClickThreshold(uint8_t value)
{
  writeByte(devAddrA, LSM303DLHC_RA_CLICK_THS_A, value);
}

/*Get the threshold which is used to start the click-detection procedure. The threshold
value is a 7 bit unsigned number.
@return The value of the click detection threshold
@see LSM303DLHC_RA_CLICK_THS_A
*/
uint8_t LSM303DLHC_getAccelClickThreshold()
{
  readByte(devAddrA, LSM303DLHC_RA_CLICK_THS_A, buffer);
  return buffer[0];
}

// TIME_LIMIT_A, r/w

/*Set the maximum time interval that can elapse between the start of the click-detection
procedure (the acceleration on the selected channel exceeds the programmed threshold)
and when the acceleration falls below the threshold.
@param value The value of the maximum time interval for click detection
@see LSM303DLHC_RA_TIME_LIMIT_A
*/
void LSM303DLHC_setAcceLClickTimeLimit(uint8_t value)
{
  writeByte(devAddrA, LSM303DLHC_RA_TIME_LIMIT_A, value);
}

/*Get the maximum time interval that can elapse between the start of the click-detection
procedure (the acceleration on the selected channel exceeds the programmed threshold)
and when the acceleration falls below the threshold.
@return The value of the maximum time interval for click detection
@see LSM303DLHC_RA_TIME_LIMIT_A
*/
uint8_t LSM303DLHC_getAccelClickTimeLimit()
{
  readByte(devAddrA, LSM303DLHC_RA_TIME_LIMIT_A, buffer);
  return buffer[0];
}

// TIME_LATENCY_A, r/w

/*Set the time interval that starts after the first click detection where the
click-detection procedure is disabled, in cases where the device is configured
for double-click detection.
@param value The double click interval
@see LSM303DLHC_RA_TIME_LATENCY_A
*/
void LSM303DLHC_setAcceLClickTimeLatency(uint8_t value)
{
  writeByte(devAddrA, LSM303DLHC_RA_TIME_LATENCY_A, value);
}

/*Get the time interval that starts after the first click detection where the
click-detection procedure is disabled, in cases where the device is configured
for double-click detection.
@return The double click interval
@see LSM303DLHC_RA_TIME_LATENCY_A
*/
uint8_t LSM303DLHC_getAccelClickTimeLatency()
{
  readByte(devAddrA, LSM303DLHC_RA_TIME_LATENCY_A, buffer);
  return buffer[0];
}

// TIME_WINDOW_A, r/w

/*Set the maximum interval of time that can elapse after the end of the latency
interval in which the click detection procedure can start, in cases where the device
is configured for double-click detection.
@param value The time window
@see LSM303DLHC_RA_TIME_WINDOW_A
*/
void LSM303DLHC_setAcceLClickTimeWindow(uint8_t value)
{
  writeByte(devAddrA, LSM303DLHC_RA_TIME_WINDOW_A, value);
}

/*Get the maximum interval of time that can elapse after the end of the latency
interval in which the click detection procedure can start, in cases where the device
is configured for double-click detection.
@return The double click time window
@see LSM303DLHC_RA_TIME_WINDOW_A
*/
uint8_t LSM303DLHC_getAccelClickTimeWindow()
{
  readByte(devAddrA, LSM303DLHC_RA_TIME_WINDOW_A, buffer);
  return buffer[0];
}

// CRA_REG_M_A, rw

/*Set the temperature sensor to be enabled.
@param enabled The new enabled state of the temperature sensor.
@see LSM303DLHC_RA_CRA_REG_M
@see LSM303DLHC_TEMP_EN_BIT
*/
void LSM303DLHC_setMagTemperatureEnabled(bool enabled)
{
  writeBit(devAddrM, LSM303DLHC_RA_CRA_REG_M, LSM303DLHC_TEMP_EN_BIT, enabled);
}

/*Get whether the temperature sensor is enabled.
@return The state of the temperature sensor.
@see LSM303DLHC_RA_CRA_REG_M
@see LSM303DLHC_TEMP_EN_BIT
*/
bool LSM303DLHC_getMagTemperatureEnabled()
{
  readBit(devAddrM, LSM303DLHC_RA_CRA_REG_M, LSM303DLHC_TEMP_EN_BIT, buffer);
  return buffer[0];
}

/*Set the magetometer output data rate.  Valid rates are 0, 1, 3, 7, 15, 30, 75, 220.
@param rate The new output data rate of the magnetometer.
@see LSM303DLHC_RA_CRA_REG_M
@see LSM303DLHC_DO_BIT
@see LSM303DLHC_DO_LENGTH
@see LSM303DLHC_DO_RATE_0
@see LSM303DLHC_DO_RATE_1
@see LSM303DLHC_DO_RATE_3
@see LSM303DLHC_DO_RATE_7
@see LSM303DLHC_DO_RATE_15
@see LSM303DLHC_DO_RATE_30
@see LSM303DLHC_DO_RATE_75
@see LSM303DLHC_DO_RATE_220
*/
void LSM303DLHC_setMagOutputDataRate(uint8_t rate)
{
  uint8_t writeBit = 0x00;
  if (rate == 0)
  {
    writeBit = LSM303DLHC_DO_RATE_0;
  }
  else if (rate == 1)
  {
    writeBit = LSM303DLHC_DO_RATE_1;
  }
  else if (rate == 3)
  {
    writeBit = LSM303DLHC_DO_RATE_3;
  }
  else if (rate == 7)
  {
    writeBit = LSM303DLHC_DO_RATE_7;
  }
  else if (rate == 15)
  {
    writeBit = LSM303DLHC_DO_RATE_15;
  }
  else if (rate == 30)
  {
    writeBit = LSM303DLHC_DO_RATE_30;
  }
  else if (rate == 75)
  {
    writeBit = LSM303DLHC_DO_RATE_75;
  }
  else if (rate == 220)
  {
    writeBit = LSM303DLHC_DO_RATE_220;
  }

  writeBits(devAddrM, LSM303DLHC_RA_CRA_REG_M, LSM303DLHC_DO_BIT, LSM303DLHC_DO_LENGTH, writeBit);
}

/*Get the magetometer output data rate.
@return The output data rate of the magnetometer.
@see LSM303DLHC_RA_CRA_REG_M
@see LSM303DLHC_DO_BIT
@see LSM303DLHC_DO_LENGTH
@see LSM303DLHC_DO_RATE_0
@see LSM303DLHC_DO_RATE_1
@see LSM303DLHC_DO_RATE_3
@see LSM303DLHC_DO_RATE_7
@see LSM303DLHC_DO_RATE_15
@see LSM303DLHC_DO_RATE_30
@see LSM303DLHC_DO_RATE_75
@see LSM303DLHC_DO_RATE_220
*/
uint8_t LSM303DLHC_getMagOutputDataRate()
{
  readBits(devAddrM, LSM303DLHC_RA_CRA_REG_M, LSM303DLHC_DO_BIT,
           LSM303DLHC_DO_LENGTH, buffer);
  uint8_t rate = buffer[0];

  if (rate == LSM303DLHC_DO_RATE_0)
  {
    return 0;
  }
  else if (rate == LSM303DLHC_DO_RATE_1)
  {
    return 1;
  }
  else if (rate == LSM303DLHC_DO_RATE_3)
  {
    return 3;
  }
  else if (rate == LSM303DLHC_DO_RATE_7)
  {
    return 7;
  }
  else if (rate == LSM303DLHC_DO_RATE_15)
  {
    return 15;
  }
  else if (rate == LSM303DLHC_DO_RATE_30)
  {
    return 30;
  }
  else if (rate == LSM303DLHC_DO_RATE_75)
  {
    return 75;
  }
  else if (rate == LSM303DLHC_DO_RATE_220)
  {
    return 220;
  }
  return 0;
}

// CRB_REG_M, rws

/*Set the magnetometer gain.
@param gain The new gain of the magnetometer
@see LSM303DLHC_RA_CRB_REG_M
@see LSM303DLHC_GN_BIT
@see LSM303DLHC_GN_LENGTH
@see LSM303DLHC_GN_230
@see LSM303DLHC_GN_330
@see LSM303DLHC_GN_400
@see LSM303DLHC_GN_450
@see LSM303DLHC_GN_670
@see LSM303DLHC_GN_855
@see LSM303DLHC_GN_1100
*/
void LSM303DLHC_setMagGain(uint16_t gain)
{
  uint8_t writeBit = 0x00;
  if (gain == 230)
  {
    writeBit = LSM303DLHC_GN_230;
  }
  else if (gain == 330)
  {
    writeBit = LSM303DLHC_GN_330;
  }
  else if (gain == 400)
  {
    writeBit = LSM303DLHC_GN_400;
  }
  else if (gain == 450)
  {
    writeBit = LSM303DLHC_GN_450;
  }
  else if (gain == 670)
  {
    writeBit = LSM303DLHC_GN_670;
  }
  else if (gain == 855)
  {
    writeBit = LSM303DLHC_GN_855;
  }
  else if (gain == 1100)
  {
    writeBit = LSM303DLHC_GN_1100;
  }

  writeBits(devAddrM, LSM303DLHC_RA_CRB_REG_M, LSM303DLHC_GN_BIT, LSM303DLHC_GN_LENGTH, writeBit);
}

/*Get the magnetometer gain.
@return The gain of the magnetometer
@see LSM303DLHC_RA_CRB_REG_M
@see LSM303DLHC_GN_230
@see LSM303DLHC_GN_330
@see LSM303DLHC_GN_400
@see LSM303DLHC_GN_450
@see LSM303DLHC_GN_670
@see LSM303DLHC_GN_855
@see LSM303DLHC_GN_1100
*/
uint16_t LSM303DLHC_getMagGain()
{
  readBits(devAddrM, LSM303DLHC_RA_CRB_REG_M, LSM303DLHC_GN_BIT, LSM303DLHC_GN_LENGTH, buffer);
  uint8_t gain = buffer[0];
  if (gain == LSM303DLHC_GN_230)
  {
    return 230;
  }
  else if (gain == LSM303DLHC_GN_330)
  {
    return 330;
  }
  else if (gain == LSM303DLHC_GN_400)
  {
    return 400;
  }
  else if (gain == LSM303DLHC_GN_450)
  {
    return 450;
  }
  else if (gain == LSM303DLHC_GN_670)
  {
    return 670;
  }
  else if (gain == LSM303DLHC_GN_855)
  {
    return 855;
  }
  else if (gain == LSM303DLHC_GN_1100)
  {
    return 1100;
  }
  return 0;
}

// MR_REG_M, rw

/*Set the magnetometer mode.
@param mode The new mode for the magnetometer.
@see LSM303DLHC_RA_MR_REG_M
@see LSM303DLHC_MD_BIT
@see LSM303DLHC_MD_LENGTH
@see LSM303DLHC_MD_CONTINUOUS
@see LSM303DLHC_MD_SINGLE
@see LSM303DLHC_MD_SLEEP
*/
void LSM303DLHC_setMagMode(uint8_t mode)
{
  writeBits(devAddrM, LSM303DLHC_RA_MR_REG_M, LSM303DLHC_MD_BIT, LSM303DLHC_MD_LENGTH,
            mode);
}

/*Get the magnetometer mode.
@return The mode for the magnetometer.
@see LSM303DLHC_RA_MR_REG_M
@see LSM303DLHC_MD_BIT
@see LSM303DLHC_MD_LENGTH
@see LSM303DLHC_MD_CONTINUOUS
@see LSM303DLHC_MD_SINGLE
@see LSM303DLHC_MD_SLEEP
*/
uint8_t LSM303DLHC_getMagMode()
{
  readBits(devAddrM, LSM303DLHC_RA_MR_REG_M, LSM303DLHC_MD_BIT, LSM303DLHC_MD_LENGTH,
           buffer);
  return buffer[0];
}

// OUT_____M, r

// TODO
/** Get the magnetic field for all 3 axes
 * @param x 16-bit integer container for the X-axis magnetic field
 * @param y 16-bit integer container for the Y-axis magnetic field
 * @param z 16-bit integer container for the Z-axis magnetic field
 */
esp_err_t LSM303DLHC_getMag(int16_t *x, int16_t *y, int16_t *z)
{
  esp_err_t ret;
  ret = readBytes(devAddrM, LSM303DLHC_RA_OUT_X_H_M, 6, buffer);
  *x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
  *z = ((((int16_t)buffer[2]) << 8) | buffer[3]);
  *y = ((((int16_t)buffer[4]) << 8) | buffer[5]);
  return ret;
}

/** Get the magnetic field along the X-axis
 * @return magnetic field along the X-axis
 * @see LSM303DLHC_RA_OUT_X_L_M
 * @see LSM303DLHC_RA_OUT_X_H_M
 */
int16_t LSM303DLHC_getMagX()
{
  readBytes(devAddrM, LSM303DLHC_RA_OUT_X_H_M, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get the magnetic field along the X-axis
 * @return magnetic field along the X-axis
 * @see LSM303DLHC_RA_OUT_Y_L_M
 * @see LSM303DLHC_RA_OUT_Y_H_M
 */
int16_t LSM303DLHC_getMagY()
{
  readBytes(devAddrM, LSM303DLHC_RA_OUT_Y_H_M, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/** Get the magnetic field along the X-axis
 * @return magnetic field along the X-axis
 * @see LSM303DLHC_RA_OUT_Z_L_M
 * @see LSM303DLHC_RA_OUT_Z_H_M
 */
int16_t LSM303DLHC_getMagZ()
{
  readBytes(devAddrM, LSM303DLHC_RA_OUT_Z_H_M, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// SR_REG_M, r

/*Get data output register lock.  This bit is set when the first magnetic file
data register has been read.
@return the output data register lock
@see LSM303DLHC_RA_SR_REG_M
@see LSM303DLHC_RA_M_LOCK_BIT
*/
bool LSM303DLHC_getMagOutputDataRegisterLock()
{
  readBit(devAddrM, LSM303DLHC_RA_SR_REG_M, LSM303DLHC_M_LOCK_BIT, buffer);
  return buffer[0];
}

/*Get data ready bit.  This bit is when a new set of measurements is available.
@return the data ready bit status
@see LSM303DLHC_RA_SR_REG_M
@see LSM303DLHC_RA_M_DRDY_BIT
*/
bool LSM303DLHC_getMagDataReady()
{
  readBit(devAddrM, LSM303DLHC_RA_SR_REG_M, LSM303DLHC_M_DRDY_BIT, buffer);
  return buffer[0];
}

// TEMP_OUT_*_M

/*Get the tempearture data. 12-bit resolution.  8LSB/deg
@return The temperature data.
@see LSM303DLHC_RA_TEMP_OUT_H_M
@see LSM303DLHC_RA_TEMP_OUT_L_M
*/
int16_t LSM303DLHC_getTemperature()
{
  readBytes(devAddrM, LSM303DLHC_RA_TEMP_OUT_H_M | 0x80, 2, buffer);
  return ((((int16_t)buffer[0]) << 8) | buffer[1]) >> 4;
}

// WHO_AM_I register, read-only
// There is no ID register for this device.
uint8_t LSM303DLHC_getDeviceID()
{
  return 0;
  // read a single byte and return it
  // readByte(devAddrA, LSM303DLHC_RA_WHO_AM_I, buffer);
  // return buffer[0];
}
