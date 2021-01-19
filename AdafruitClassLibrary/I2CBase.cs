﻿/*------------------------------------------------------------------------
  Adafruit Class Library for Windows Core IoT: I2C base class.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------*/

using System;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;

namespace AdafruitClassLibrary
{
    public class I2CBase
    {
        public enum I2CSpeed { I2C_100kHz, I2C_400kHz };

        #region Properties

        private int I2CAddr { get; set; }
        protected I2cDevice Device { get; set; }

        #endregion Properties

        #region Constructor

        protected I2CBase(int addr)
        {
            I2CAddr = addr;
        }

        #endregion Constructor

        #region Initialization

        /// <summary>
        /// InitI2C
        /// Initialize I2C Communications
        /// </summary>
        /// <returns>async Task</returns>
        public async Task InitI2CAsync(I2CSpeed i2cSpeed = I2CSpeed.I2C_100kHz)
        {
            // initialize I2C communications
            try
            {
                I2cConnectionSettings i2cSettings = new I2cConnectionSettings(I2CAddr);
                if (i2cSpeed == I2CSpeed.I2C_400kHz)
                    i2cSettings.BusSpeed = I2cBusSpeed.FastMode;
                else
                    i2cSettings.BusSpeed = I2cBusSpeed.StandardMode;

                string deviceSelector = I2cDevice.GetDeviceSelector();
                var i2cDeviceControllers = await DeviceInformation.FindAllAsync(deviceSelector);
                Device = await I2cDevice.FromIdAsync(i2cDeviceControllers[0].Id, i2cSettings);
            }
            catch (Exception e)
            {
                System.Diagnostics.Debug.WriteLine("Exception: {0}", e.Message);
                return;
            }
        }

        #endregion Initialization

        #region I2C primitives

        /// <summary>
        /// WriteRead
        /// writes to I2C and reads back the result
        /// </summary>
        /// <param name="writeBuffer"></param>
        /// <param name="readBuffer"></param>
        protected void WriteRead(byte[] writeBuffer, byte[] readBuffer)
        {
            try
            {
                lock (Device)
                {
                    Device.WriteRead(writeBuffer, readBuffer);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("I2C WriteRead Exception: {0}", ex.Message);
            }
        }

        /// <summary>
        /// Read
        /// Reads a PCA9685 register
        /// </summary>
        /// <param name="readBuffer"></param>
        protected void Read(byte[] readBuffer)
        {
            try
            {
                lock (Device)
                {
                    Device.Read(readBuffer);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("I2C Read Exception: {0}", ex.Message);
            }
        }

        /// <summary>
        /// Write
        /// Writes to a PCA9685 register
        /// </summary>
        /// <param name="writeBuffer"></param>
        protected void Write(byte[] writeBuffer)
        {
            try
            {
                lock (Device)
                {
                    Device.Write(writeBuffer);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("I2C Write Exception: {0}", ex.Message);
            }
        }

        #endregion I2C primitives
    }
}