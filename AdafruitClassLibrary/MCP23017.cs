/*------------------------------------------------------------------------
  Windows IoT library to control the MCP23017 I2C expander chip.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------
  This file is part of the Adafruit Windows IoT Class Library

  Adafruit MCP23017 is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  MIT license, all text above must be included in any redistribution.
  ------------------------------------------------------------------------*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;
using Windows.Devices.Gpio;

namespace AdafruitClassLibrary
{
    public class MCP23017
    {
        //default I2C address
        const int MCP23017_ADDRESS = 0x20;

        // port A registers
        const byte MCP23017_IODIRA = 0x00;
        const byte MCP23017_IPOLA = 0x02;
        const byte MCP23017_GPINTENA = 0x04;
        const byte MCP23017_DEFVALA = 0x06;
        const byte MCP23017_INTCONA = 0x08;
        const byte MCP23017_IOCONA = 0x0A;
        const byte MCP23017_GPPUA = 0x0C;
        const byte MCP23017_INTFA = 0x0E;
        const byte MCP23017_INTCAPA = 0x10;
        const byte MCP23017_GPIOA = 0x12;
        const byte MCP23017_OLATA = 0x14;

        //Port B registers
        const byte MCP23017_IODIRB = 0x01;
        const byte MCP23017_IPOLB = 0x03;
        const byte MCP23017_GPINTENB = 0x05;
        const byte MCP23017_DEFVALB = 0x07;
        const byte MCP23017_INTCONB = 0x09;
        const byte MCP23017_IOCONB = 0x0B;
        const byte MCP23017_GPPUB = 0x0D;
        const byte MCP23017_INTFB = 0x0F;
        const byte MCP23017_INTCAPB = 0x11;
        const byte MCP23017_GPIOB = 0x13;
        const byte MCP23017_OLATB = 0x15;

        private int I2CAddr { get; set; }
        private I2cDevice PortExpander { get; set; }

        public enum Direction { INPUT = 0, OUTPUT = 1 };
        public enum Level { LOW = 0, HIGH = 1 };

        public MCP23017(int addr)
        {
            I2CAddr = addr;
        }

        public MCP23017()
        {
            I2CAddr = MCP23017_ADDRESS;
        }

        public async Task InitI2C()
        {
            // initialize I2C communications
            try
            {
                I2cConnectionSettings i2cSettings = new I2cConnectionSettings(I2CAddr);
                string deviceSelector = I2cDevice.GetDeviceSelector();
                var i2cDeviceControllers = await DeviceInformation.FindAllAsync(deviceSelector);
                PortExpander = await I2cDevice.FromIdAsync(i2cDeviceControllers[0].Id, i2cSettings);
            }
            catch (Exception e)
            {
                System.Diagnostics.Debug.WriteLine("Exception: {0}", e.Message);
                return;
            }
        }

        public async Task InitMCP23017()
        {
            byte[] writeBuffer;

            await InitI2C();

            // set defaults!
            // all inputs on Port A
            writeBuffer = new byte[] { MCP23017_IODIRA, 0xFF };
            Write(writeBuffer);

            // all inputs on Port B
            writeBuffer = new byte[] { MCP23017_IODIRB, 0xFF };
            Write(writeBuffer);
        }

        public void pinMode(int p, Direction d)
        {
            byte[] readBuffer;
            byte[] writeBuffer;
            byte iodir;
            byte iodiraddr;

            // only 16 bits!
            if (p > 15)
                return;

            if (p < 8)
                iodiraddr = MCP23017_IODIRA;
            else
            {
                iodiraddr = MCP23017_IODIRB;
                p -= 8;
            }

            // read the current IODIR
            readBuffer = new byte[1];

            WriteRead(new byte[] { iodiraddr }, readBuffer);
            iodir = readBuffer[0];

            // set the pin and direction
            if (d == Direction.INPUT)
            {
                iodir |= (byte)(1 << p);
            }
            else
            {
                iodir &= (byte) ~((1 << p));
            }

            // write the new IODIR
            writeBuffer = new byte[] { iodiraddr, iodir };
            Write(writeBuffer);
        }

        public void digitalWrite(int pin, Level d)
        {
            byte[] readBuffer;
            byte[] writeBuffer;
            byte pinRegister;
            byte registerAddr;
            byte olatAddr;

            // only 16 bits!
            if (pin > 15)
                return;

            if (pin < 8)
            {
                olatAddr = MCP23017_OLATA;
                registerAddr = MCP23017_GPIOA;
            }
            else
            {
                olatAddr = MCP23017_OLATB;
                registerAddr = MCP23017_GPIOB;
                pin -= 8;
            }

            // read the current GPIO output latches
            readBuffer = new byte[1];
            WriteRead(new byte[] { olatAddr }, readBuffer);
            pinRegister = readBuffer[0];

            // set the pin and direction
            if (d == Level.HIGH)
            {
                pinRegister |= (byte)(1 << pin);
            }
            else
            {
                pinRegister &= (byte) ~(1 << pin);
            }

            // write the new GPIO
            writeBuffer = new byte[] { registerAddr, pinRegister };
            Write(writeBuffer);
        }

        public void pullUp(int pin, Level d)
        {
            byte[] readBuffer;
            byte[] writeBuffer;
            byte pullupRegister;
            byte pullupAddr;

            // only 16 bits!
            if (pin > 15)
                return;

            if (pin < 8)
                pullupAddr = MCP23017_GPPUA;
            else
            {
                pullupAddr = MCP23017_GPPUB;
                pin -= 8;
            }

            // read the current pullup register set
            readBuffer = new byte[1];
            WriteRead(new byte[] {pullupAddr }, readBuffer);
            pullupRegister = readBuffer[0];

            // set the pin and direction
            if (d == Level.HIGH)
            {
                pullupRegister |= (byte)(1 << pin);
            }
            else
            {
                pullupRegister &= (byte) ~(1 << pin);
            }

            // write the new pullup
            writeBuffer = new byte[] { pullupAddr, pullupRegister };
            Write(writeBuffer);
        }

        public int digitalRead(int pin)
        {
            byte[] readBuffer;
            byte registerAddr;

            // only 16 bits!
            if (pin > 15)
                return 0;

            if (pin < 8)
                registerAddr = MCP23017_GPIOA;
            else
            {
                registerAddr = MCP23017_GPIOB;
                pin -= 8;
            }

            // read the current GPIO
            readBuffer = new byte[1];
            WriteRead(new byte[] { registerAddr }, readBuffer);
            return (readBuffer[0] >> pin) & 0x1;
        }

        public void writeGPIOAB(UInt16 ba)
        {
            byte[] writeBuffer;
            writeBuffer = new byte[] { MCP23017_GPIOA, (byte) (ba & 0xFF), (byte)(ba >> 8) };
            Write(writeBuffer);
        }

        public UInt16 readGPIOAB()
        {
            byte[] readBuffer;

            // read the current GPIO output latches
            readBuffer = new byte[2];
            WriteRead(new byte[] { MCP23017_GPIOA }, readBuffer);
            return (UInt16)((readBuffer[1] << 8)  | readBuffer[0]);

        }

        private void WriteRead(byte[] writeBuffer, byte[] readBuffer)
        {
            try
            {
                PortExpander.WriteRead(writeBuffer, readBuffer);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("I2C WriteRead Exception: {0}", ex.Message);
            }
        }

        private void Read(byte[] readBuffer)
        {
            try
            {
                PortExpander.Read(readBuffer);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("I2C Read Exception: {0}", ex.Message);
            }
        }

        private void Write(byte[] writeBuffer)
        {
            try
            {
                PortExpander.Write(writeBuffer);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("I2C Write Exception: {0}", ex.Message);
            }
        }

    }
}
