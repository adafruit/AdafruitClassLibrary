/*------------------------------------------------------------------------
  Adafruit Class Library for Windows Core IoT: MCP23017 I2C expander chip.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------*/

using System;
using System.Threading.Tasks;

namespace AdafruitClassLibrary
{
    public class Mcp23017 : I2CBase
    {
        #region Constants

        //default I2C address
        private const int MCP23017_ADDRESS = 0x20;

        // port A registers
        private const byte MCP23017_IODIRA = 0x00;

        private const byte MCP23017_IPOLA = 0x02;
        private const byte MCP23017_GPINTENA = 0x04;
        private const byte MCP23017_DEFVALA = 0x06;
        private const byte MCP23017_INTCONA = 0x08;
        private const byte MCP23017_IOCONA = 0x0A;
        private const byte MCP23017_GPPUA = 0x0C;
        private const byte MCP23017_INTFA = 0x0E;
        private const byte MCP23017_INTCAPA = 0x10;
        private const byte MCP23017_GPIOA = 0x12;
        private const byte MCP23017_OLATA = 0x14;

        //Port B registers
        private const byte MCP23017_IODIRB = 0x01;

        private const byte MCP23017_IPOLB = 0x03;
        private const byte MCP23017_GPINTENB = 0x05;
        private const byte MCP23017_DEFVALB = 0x07;
        private const byte MCP23017_INTCONB = 0x09;
        private const byte MCP23017_IOCONB = 0x0B;
        private const byte MCP23017_GPPUB = 0x0D;
        private const byte MCP23017_INTFB = 0x0F;
        private const byte MCP23017_INTCAPB = 0x11;
        private const byte MCP23017_GPIOB = 0x13;
        private const byte MCP23017_OLATB = 0x15;

        public enum Direction { INPUT = 0, OUTPUT = 1 };

        public enum Level { LOW = 0, HIGH = 1 };

        #endregion Constants

        #region Constructor

        public Mcp23017(int addr = MCP23017_ADDRESS) : base(addr)
        {
        }

        #endregion Constructor

        #region Initialization

        /// <summary>
        /// InitMCP23017
        /// Initialize MCP23017 chip
        /// </summary>
        /// <returns>async Task</returns>
        public async Task InitMCP23017Async(I2CSpeed i2cSpeed = I2CSpeed.I2C_100kHz)
        {
            byte[] writeBuffer;

            await InitI2CAsync(i2cSpeed);

            // set defaults!
            // all outputs on Port A
            writeBuffer = new byte[] { MCP23017_IODIRA, 0xFF };
            Write(writeBuffer);

            // all outputs on Port B
            writeBuffer = new byte[] { MCP23017_IODIRB, 0xFF };
            Write(writeBuffer);
        }

        #endregion Initialization

        #region Interrupts

        /// <summary>
        /// Enables the interrupts for pin p.
        /// </summary>
        /// <param name="p">Pin id (0..15).</param>
        public void EnableInterrupts(int p)
        {
            byte gppuAddr, gppu, gpintenAddr, gpinten;

            // only 16 bits!
            if (p > 15)
                return;
            if (p < 8)
            {
                gppuAddr = MCP23017_GPPUA;
                gpintenAddr = MCP23017_GPINTENA;
            }
            else
            {
                gppuAddr = MCP23017_GPPUB;
                gpintenAddr = MCP23017_GPINTENB;
                p -= 8;
            }

            byte[] readBuffer = new byte[1];

            // Enable pull-up resistor for pin p
            WriteRead(new byte[] { gppuAddr }, readBuffer);
            gppu = readBuffer[0];
            gppu |= (byte)(1 << p);
            Write(new byte[] { gppuAddr, gppu });

            // Enable interrup on pin p
            WriteRead(new byte[] { gpintenAddr }, readBuffer);
            gpinten = readBuffer[0];
            gpinten |= (byte)(1 << p);
            Write(new byte[] { gpintenAddr, gpinten });
        }

        /// <summary>
        /// Enables interrupts mirroring.
        /// Signalises both INTA and INTB on input change.
        /// </summary>
        public void EnableInterruptsMirroring()
        {
            byte[] ReadBuffer = new byte[1];

            WriteRead(new byte[] { MCP23017_IOCONA }, ReadBuffer); // 0x0A IOCONA
            byte NewValues = ReadBuffer[0];
            NewValues |= (byte)(1 << 6);
            Write(new byte[] { MCP23017_IOCONA, NewValues });
        }

        #endregion

        #region Operations

        /// <summary>
        /// pinMode
        /// Set pin direction, input or output
        /// </summary>
        /// <param name="p"></param>
        /// <param name="d"></param>
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
                iodir &= (byte)~((1 << p));
            }

            // write the new IODIR
            writeBuffer = new byte[] { iodiraddr, iodir };
            Write(writeBuffer);
        }

        /// <summary>
        /// digitalWrite
        /// Sets the state of an output pin
        /// </summary>
        /// <param name="pin"></param>
        /// <param name="d"></param>
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

            lock (Device)
            {
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
                    pinRegister &= (byte)~(1 << pin);
                }

                // write the new GPIO
                writeBuffer = new byte[] { registerAddr, pinRegister };
                Write(writeBuffer);
            }
        }

        /// <summary>
        /// pullUp
        /// Sets the pullup resistor on a gpio pin
        /// </summary>
        /// <param name="pin"></param>
        /// <param name="d"></param>
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

            lock (Device)
            {
                // read the current pullup register set
                readBuffer = new byte[1];
                WriteRead(new byte[] { pullupAddr }, readBuffer);
                pullupRegister = readBuffer[0];

                // set the pin and direction
                if (d == Level.HIGH)
                {
                    pullupRegister |= (byte)(1 << pin);
                }
                else
                {
                    pullupRegister &= (byte)~(1 << pin);
                }

                // write the new pullup
                writeBuffer = new byte[] { pullupAddr, pullupRegister };
                Write(writeBuffer);
            }
        }

        /// <summary>
        /// digitalRead
        /// returns the input state of a gpio pin
        /// </summary>
        /// <param name="pin"></param>
        /// <returns>Level</returns>
        public Level digitalRead(int pin)
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
            return ((readBuffer[0] >> pin) & 0x1) == 0 ? Level.LOW : Level.HIGH;
        }

        /// <summary>
        /// writeGPIOAB
        /// writes a 16-bit value to the A and B gpio registers
        /// </summary>
        /// <param name="ba"></param>
        public void writeGPIOAB(UInt16 ba)
        {
            byte[] writeBuffer;
            writeBuffer = new byte[] { MCP23017_GPIOA, (byte)(ba & 0xFF), (byte)(ba >> 8) };
            Write(writeBuffer);
        }

        /// <summary>
        /// readGPIOAB
        /// Reads a 16-bit value from the GPIO a and B registers
        /// </summary>
        /// <returns>UInt16</returns>
        public UInt16 readGPIOAB()
        {
            byte[] readBuffer;

            // read the current GPIO output latches
            readBuffer = new byte[2];
            WriteRead(new byte[] { MCP23017_GPIOA }, readBuffer);
            return (UInt16)((readBuffer[1] << 8) | readBuffer[0]);
        }

        #endregion Operations
    }
}