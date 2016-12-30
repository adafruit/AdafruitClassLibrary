/*------------------------------------------------------------------------
  Adafruit Class Library for Windows Core IoT: BNO055 IMU chip.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------
  This file is part of the Adafruit Windows IoT Class Library

  Adafruit Class Library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

  MIT license, all text above must be included in any redistribution.
  ------------------------------------------------------------------------*/
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.SerialCommunication;
using Windows.Storage.Streams;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;

namespace AdafruitClassLibrary
{
    public class BNO055
    {
        #region class definitions
        public class Revisions
        {
            public int Software { get; set; }
            public byte Bootloader { get; set; }
            public byte AccelID { get; set; }
            public byte MagID { get; set; }
            public byte GyroID { get; set; }
        }

        public class SystemStatus
        {
            public byte StatusReg { get; set; }
            public byte SelfTestResult { get; set; }
            public byte ErrorReg { get; set; }
        }

        public class CalibrationStatus
        {
            public byte System { get; set; }
            public byte Gyro { get; set; }
            public byte Accel { get; set; }
            public byte Mag { get; set; }
        }

        public class AxisRemap
        {
            public byte X { get; set; }
            public byte Y { get; set; }
            public byte Z { get; set; }
            public byte XSign { get; set; }
            public byte YSign { get; set; }
            public byte ZSign { get; set; }
        }

        public class Coords
        {
            public double X { get; set; }
            public double Y { get; set; }
            public double Z { get; set; }
        }

        public class Euler
        {
            public double Heading { get; set; }
            public double Roll { get; set; }
            public double Pitch { get; set; }
        }

        public class Quaternion
        {
            public double W { get; set; }
            public double X { get; set; }
            public double Y { get; set; }
            public double Z { get; set; }
        }

        #endregion
        #region Register Definitions
        // I2C addresses
        const byte BNO055_ADDRESS_A = 0x28;
        const byte BNO055_ADDRESS_B = 0x29;
        const byte BNO055_ID = 0xA0;

        // Page id register definition
        const byte BNO055_PAGE_ID_ADDR = 0X07;

        // PAGE0 REGISTER DEFINITION START
        const byte BNO055_CHIP_ID_ADDR = 0x00;
        const byte BNO055_ACCEL_REV_ID_ADDR = 0x01;
        const byte BNO055_MAG_REV_ID_ADDR = 0x02;
        const byte BNO055_GYRO_REV_ID_ADDR = 0x03;
        const byte BNO055_SW_REV_ID_LSB_ADDR = 0x04;
        const byte BNO055_SW_REV_ID_MSB_ADDR = 0x05;
        const byte BNO055_BL_REV_ID_ADDR = 0X06;

        // Accel data register
        const byte BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08;
        const byte BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09;
        const byte BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A;
        const byte BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B;
        const byte BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C;
        const byte BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D;

        // Mag data register
        const byte BNO055_MAG_DATA_X_LSB_ADDR = 0X0E;
        const byte BNO055_MAG_DATA_X_MSB_ADDR = 0X0F;
        const byte BNO055_MAG_DATA_Y_LSB_ADDR = 0X10;
        const byte BNO055_MAG_DATA_Y_MSB_ADDR = 0X11;
        const byte BNO055_MAG_DATA_Z_LSB_ADDR = 0X12;
        const byte BNO055_MAG_DATA_Z_MSB_ADDR = 0X13;

        // Gyro data registers
        const byte BNO055_GYRO_DATA_X_LSB_ADDR = 0X14;
        const byte BNO055_GYRO_DATA_X_MSB_ADDR = 0X15;
        const byte BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16;
        const byte BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17;
        const byte BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18;
        const byte BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19;

        // Euler data registers
        const byte BNO055_EULER_H_LSB_ADDR = 0X1A;
        const byte BNO055_EULER_H_MSB_ADDR = 0X1B;
        const byte BNO055_EULER_R_LSB_ADDR = 0X1C;
        const byte BNO055_EULER_R_MSB_ADDR = 0X1D;
        const byte BNO055_EULER_P_LSB_ADDR = 0X1E;
        const byte BNO055_EULER_P_MSB_ADDR = 0X1F;

        // Quaternion data registers
        const byte BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20;
        const byte BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21;
        const byte BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22;
        const byte BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23;
        const byte BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24;
        const byte BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25;
        const byte BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26;
        const byte BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27;

        // Linear acceleration data registers
        const byte BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28;
        const byte BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29;
        const byte BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A;
        const byte BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B;
        const byte BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C;
        const byte BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D;

        // Gravity data registers
        const byte BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E;
        const byte BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F;
        const byte BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30;
        const byte BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31;
        const byte BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32;
        const byte BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33;

        // Temperature data register
        const byte BNO055_TEMP_ADDR = 0X34;

        // Status registers
        const byte BNO055_CALIB_STAT_ADDR = 0X35;
        const byte BNO055_SELFTEST_RESULT_ADDR = 0X36;
        const byte BNO055_INTR_STAT_ADDR = 0X37;

        const byte BNO055_SYS_CLK_STAT_ADDR = 0X38;
        const byte BNO055_SYS_STAT_ADDR = 0X39;
        const byte BNO055_SYS_ERR_ADDR = 0X3A;

        // Unit selection register
        const byte BNO055_UNIT_SEL_ADDR = 0X3B;
        const byte BNO055_DATA_SELECT_ADDR = 0X3C;

        // Mode registers
        const byte BNO055_OPR_MODE_ADDR = 0X3D;
        const byte BNO055_PWR_MODE_ADDR = 0X3E;

        const byte BNO055_SYS_TRIGGER_ADDR = 0X3F;
        const byte BNO055_TEMP_SOURCE_ADDR = 0X40;

        // Axis remap registers
        const byte BNO055_AXIS_MAP_CONFIG_ADDR = 0X41;
        const byte BNO055_AXIS_MAP_SIGN_ADDR = 0X42;

        // Axis remap values
        const byte AXIS_REMAP_X = 0x00;
        const byte AXIS_REMAP_Y = 0x01;
        const byte AXIS_REMAP_Z = 0x02;
        const byte AXIS_REMAP_POSITIVE = 0x00;
        const byte AXIS_REMAP_NEGATIVE = 0x01;

        // SIC registers
        const byte BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43;
        const byte BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44;
        const byte BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45;
        const byte BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46;
        const byte BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47;
        const byte BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48;
        const byte BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49;
        const byte BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A;
        const byte BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B;
        const byte BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C;
        const byte BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D;
        const byte BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E;
        const byte BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F;
        const byte BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50;
        const byte BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51;
        const byte BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52;
        const byte BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53;
        const byte BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54;

        // Accelerometer Offset registers
        const byte ACCEL_OFFSET_X_LSB_ADDR = 0X55;
        const byte ACCEL_OFFSET_X_MSB_ADDR = 0X56;
        const byte ACCEL_OFFSET_Y_LSB_ADDR = 0X57;
        const byte ACCEL_OFFSET_Y_MSB_ADDR = 0X58;
        const byte ACCEL_OFFSET_Z_LSB_ADDR = 0X59;
        const byte ACCEL_OFFSET_Z_MSB_ADDR = 0X5A;

        // Magnetometer Offset registers
        const byte MAG_OFFSET_X_LSB_ADDR = 0X5B;
        const byte MAG_OFFSET_X_MSB_ADDR = 0X5C;
        const byte MAG_OFFSET_Y_LSB_ADDR = 0X5D;
        const byte MAG_OFFSET_Y_MSB_ADDR = 0X5E;
        const byte MAG_OFFSET_Z_LSB_ADDR = 0X5F;
        const byte MAG_OFFSET_Z_MSB_ADDR = 0X60;

        // Gyroscope Offset register s
        const byte GYRO_OFFSET_X_LSB_ADDR = 0X61;
        const byte GYRO_OFFSET_X_MSB_ADDR = 0X62;
        const byte GYRO_OFFSET_Y_LSB_ADDR = 0X63;
        const byte GYRO_OFFSET_Y_MSB_ADDR = 0X64;
        const byte GYRO_OFFSET_Z_LSB_ADDR = 0X65;
        const byte GYRO_OFFSET_Z_MSB_ADDR = 0X66;

        // Radius registers
        const byte ACCEL_RADIUS_LSB_ADDR = 0X67;
        const byte ACCEL_RADIUS_MSB_ADDR = 0X68;
        const byte MAG_RADIUS_LSB_ADDR = 0X69;
        const byte MAG_RADIUS_MSB_ADDR = 0X6A;

        // Power modes
        const byte POWER_MODE_NORMAL = 0X00;
        const byte POWER_MODE_LOWPOWER = 0X01;
        const byte POWER_MODE_SUSPEND = 0X02;

        // Operation mode settings
        public enum OperationMode { 
                    OPERATION_MODE_CONFIG                = 0X00,
                    OPERATION_MODE_ACCONLY               = 0X01,
                    OPERATION_MODE_MAGONLY               = 0X02,
                    OPERATION_MODE_GYRONLY               = 0X03,
                    OPERATION_MODE_ACCMAG                = 0X04,
                    OPERATION_MODE_ACCGYRO               = 0X05,
                    OPERATION_MODE_MAGGYRO               = 0X06,
                    OPERATION_MODE_AMG                   = 0X07,
                    OPERATION_MODE_IMUPLUS               = 0X08,
                    OPERATION_MODE_COMPASS               = 0X09,
                    OPERATION_MODE_M4G                   = 0X0A,
                    OPERATION_MODE_NDOF_FMC_OFF          = 0X0B,
                    OPERATION_MODE_NDOF                  = 0X0C
            };
        #endregion

        #region Properties
        private SerialDevice SerialPort { get; set; }
        private DataWriter DataWriterObject { get; set; }
        private DataReader DataReaderObject { get; set; }
        private GpioPin ResetPin { get; set; }
        private OperationMode OperatingMode { get; set; }
        #endregion

        #region Constructor
        public BNO055()
        {
            SerialPort = null;
            DataWriterObject = null;
            DataReaderObject = null;
        }
        #endregion

        #region SerialControl
        /// <summary>
        /// Connected
        /// Predicate returns true if UART is connected
        /// </summary>
        /// <returns>bool</returns>
        public bool Connected
        {
            get { return SerialPort != null; }
        }

        /// <summary>
        /// ConnectToUART
        /// - Use SerialDevice.GetDeviceSelector to find serial device named "UART0". 
        ///   This is the built-in Raspberry Pi serial port.
        /// </summary>
        /// <param name="baudRate"></param>
        /// <param name="uartID"></param>
        /// <returns>async Task</returns>
        public async Task ConnectToUARTAsync(string uartID = "UART0")
        {
            try
            {
                string aqs = SerialDevice.GetDeviceSelector(uartID);
                var dis = await DeviceInformation.FindAllAsync(aqs);

                if (dis.Count > 0)
                {
                    DeviceInformation uart = dis[0];

                    SerialPort = await SerialDevice.FromIdAsync(uart.Id);
                    // Configure serial settings
                    if (null != SerialPort)
                    {
                        SerialPort.WriteTimeout = TimeSpan.FromMilliseconds(100);
                        SerialPort.ReadTimeout = TimeSpan.FromMilliseconds(100);
                        SerialPort.BaudRate = 115200;
                        SerialPort.Parity = SerialParity.None;
                        SerialPort.StopBits = SerialStopBitCount.One;
                        SerialPort.DataBits = 8;
                        SerialPort.Handshake = SerialHandshake.None;

                        // Create the DataReader object and attach to InputStream
                        DataReaderObject = new DataReader(SerialPort.InputStream);
                        // Create the DataWriter object and attach to OutputStream
                        DataWriterObject = new DataWriter(SerialPort.OutputStream);
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(string.Format("Error creating Serial Port: {0}", ex.Message));
                if (null != SerialPort)
                    SerialPort.Dispose();
                SerialPort = null;
            }
        }

        /// <summary>
        /// DisconnectFromUART
        /// Disconnects from the UART
        /// </summary>
        public void DisconnectFromUART()
        {
            if (null != DataReaderObject)
            {
                DataReaderObject.Dispose();
            }
            if (null != DataWriterObject)
            {
                DataWriterObject.Dispose();
            }
            if (null != SerialPort)
            {
                SerialPort.Dispose();
                SerialPort = null;
            }
        }

        #endregion

        #region Serial IO

        /// <summary>
        /// SendCommand
        /// Asynchronous task to write to GPS through UART
        /// </summary>
        /// <param name="command"></param>
        /// <returns>async Task</returns>
        private async Task SendCommandAsync(byte[] command)
        {
            //Launch the storeAsync task to perform the write
            Task<UInt32> storeAsyncTask;

            if (command.Length != 0)
            {
                // Load the text from the sendText input text box to the dataWriter object
                DataWriterObject.WriteBytes(command);

                // Launch an async task to complete the write operation
                storeAsyncTask = DataWriterObject.StoreAsync().AsTask();

                UInt32 bytesWritten = await storeAsyncTask;
                if (bytesWritten != command.Length)
                    throw new Exception("Bytes written does not match command length");
            }
        }

        public async Task LoadResponseAsync(uint length)
        {
            Task<UInt32> loadAsyncTask;

            loadAsyncTask = DataReaderObject.LoadAsync(length).AsTask();
            uint bytesRead = await loadAsyncTask;
            if (length != bytesRead)
                throw new Exception("ReadDataAsync timeout");

        }

        /// <summary>
        /// ReadData
        /// </summary>
        /// <param name="data">Array of data read from device</param>
        /// <returns></returns>
        private byte[] ReadData(byte[] command, uint readLength)
        {
            byte[] headerBuffer = new byte[2];
            byte[] returnBuffer = new byte[readLength];

            SendCommandAsync(command).Wait();

            LoadResponseAsync(2).Wait();
            DataReaderObject.ReadBytes(headerBuffer);

            if (headerBuffer[0] != 0xBB)
                throw new Exception(string.Format("ReadData error 0x{0:x2}{0:x2}", headerBuffer[0], headerBuffer[1]));
            if (headerBuffer[1] != readLength)
                throw new Exception(string.Format("ReadData error: failed to read {0} bytes.  Read {1}", readLength, headerBuffer[1]));

            LoadResponseAsync(readLength).Wait();
            DataReaderObject.ReadBytes(returnBuffer);

            return returnBuffer;
        }

        /// <summary>
        /// WriteData
        /// </summary>
        /// <param name="command">command data to send</param>
        /// <param name="ack">true if ack is epected</param>
        /// <param name="retries">number of retries</param>
        /// <returns></returns>
        private byte[] WriteData(byte[] command, bool ack = true, int retries = 5)
        {
            byte[] ackBuffer = new byte[2];
            bool done = false;

            while ((!done) && (retries-- > 0))
            {
                SendCommandAsync(command).Wait();

                if (ack)
                {
                    LoadResponseAsync(2).Wait();
                    DataReaderObject.ReadBytes(ackBuffer);
                    done = 0xEE07 != (ackBuffer[0] << 8 | ackBuffer[1]);
                }
                else
                    done = true;
            }

            if (!done)
                throw new Exception("WriteData retries exceeded");
            return ackBuffer;
        }

        /// <summary>
        /// SendCommand
        /// </summary>
        /// <param name="cmdData">command string for device</param>
        private void WriteRegister(byte address, byte data, bool ack = true)
        {
            byte[] writeBuffer = new byte[] { 0xAA, 0x00, address, 1, data };
            byte[] response;
            try
            {
                response = WriteData(writeBuffer, ack);
                if (ack)
                {
                    if (0xEE01 != (response[0] << 8 | response[1]))
                        throw new Exception(string.Format("WriteRegister returned 0x{0:x2},0x{1:x2}", response[0], response[1]));
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(string.Format("WriteRegister error: {0}", ex.Message));
            }
        }

        private byte ReadRegister(byte register)
        {
            byte[] commandBuffer = new byte[] { 0xAA, 0x01, register, 1 };
            byte[] readBuffer = new byte[1];
            try
            {
                readBuffer = ReadData(commandBuffer, 1);
            }
            catch(Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(string.Format("ReadRegister error: {0}", ex.Message));
            }
            return readBuffer[0];
        }

        private List<short> ReadVector(byte address, byte length = 3)
        {
            List<short> vector = new List<short>();
            byte[] commandBuffer = new byte[] { 0xAA, 0x01, address, (byte)(length * 2) };
            byte[] readBuffer = new byte[length * 2];
            try
            {
                readBuffer = ReadData(commandBuffer, (uint)length * 2);

                for (int i = 0; i < length * 2; i = i + 2)
                {
                    int entry = ((readBuffer[i + 1] << 8) | readBuffer[i]);
                    if (entry > 32767)
                        entry -= 65536;
                    vector.Add((short)entry);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(string.Format("ReadVector error: {0}", ex.Message));
            }
            return vector;
        }

        public void SetMode(OperationMode mode)
        {
            WriteRegister(BNO055_OPR_MODE_ADDR, (byte)mode);
        }
        #endregion

        #region Initialization
        private void Reset(int RSTPin)
        {
            if (-1 == RSTPin)
            {
                WriteRegister(BNO055_SYS_TRIGGER_ADDR, 0x20, false); //do a soft reset
            }
            else  //do a hard reset
            {
                var gpio = GpioController.GetDefault();

                if (gpio != null)
                {
                    ResetPin = gpio.OpenPin(RSTPin);
                    ResetPin.Write(GpioPinValue.High);
                    ResetPin.SetDriveMode(GpioPinDriveMode.Output);

                    ResetPin.Write(GpioPinValue.Low);
                    Task.Delay(10).Wait();
                    ResetPin.Write(GpioPinValue.High);
                }
            }
            Task.Delay(650).Wait();     //wait 650ms after reset for the device to become ready, as
                                        // suggested in the datasheet
        }

        /// <summary>
        /// InitBNO055Async
        /// Initialize BNO055 chip
        /// </summary>
        /// <returns>async Task</returns>
        public async Task InitBNO055Async(OperationMode mode, string uartID = "UART0", int RSTPin = -1)
        {
            OperatingMode = mode;

            ConnectToUARTAsync(uartID).Wait();

            if (Connected)
            {

                SetMode(OperationMode.OPERATION_MODE_CONFIG);    //go into config mode
                WriteRegister(BNO055_PAGE_ID_ADDR, 0/*, false*/); //set to page 0

                byte chipID = ReadRegister(BNO055_CHIP_ID_ADDR);

                if (BNO055_ID == chipID)
                {                    
                    Reset(RSTPin);

                    WriteRegister(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL); //set power mode normal

                    WriteRegister(BNO055_SYS_TRIGGER_ADDR, 0x0); //set to use internal oscillator

                    SetMode(OperatingMode);
                }
            }
        }
        #endregion

        #region Operations
        public Revisions GetRevision()
        {
            Revisions revisionData = new Revisions();
            revisionData.AccelID = ReadRegister(BNO055_ACCEL_REV_ID_ADDR);
            revisionData.MagID = ReadRegister(BNO055_MAG_REV_ID_ADDR);
            revisionData.GyroID = ReadRegister(BNO055_GYRO_REV_ID_ADDR);
            revisionData.Bootloader = ReadRegister(BNO055_BL_REV_ID_ADDR);
            revisionData.Software = (ReadRegister(BNO055_SW_REV_ID_MSB_ADDR) << 8) | ReadRegister(BNO055_SW_REV_ID_MSB_ADDR);
            return revisionData;
        }

        public void SetExternalCrystal(bool externalCrystal)
        {
            SetMode(OperationMode.OPERATION_MODE_CONFIG);

            if (externalCrystal)
                WriteRegister(BNO055_SYS_TRIGGER_ADDR, 0x80);
            else
                WriteRegister(BNO055_SYS_TRIGGER_ADDR, 0x00);

            SetMode(OperatingMode);
        }

        public SystemStatus GetSystemStatus(bool runSelfTest)
        {
            SystemStatus status = new SystemStatus();

            if (runSelfTest)
            {
                SetMode(OperationMode.OPERATION_MODE_CONFIG);

                byte sysTrigger = ReadRegister(BNO055_SYS_TRIGGER_ADDR);
                WriteRegister(BNO055_SYS_TRIGGER_ADDR, (byte)(sysTrigger | 0x01));

                Task.Delay(1000).Wait();

                status.SelfTestResult = ReadRegister(BNO055_SELFTEST_RESULT_ADDR);

                SetMode(OperatingMode);
            }
            else
            {
                status.SelfTestResult = 0;
            }

            status.StatusReg = ReadRegister(BNO055_SYS_STAT_ADDR);
            status.ErrorReg = ReadRegister(BNO055_SYS_ERR_ADDR);

            return status;
        }

        public CalibrationStatus GetCalibrationStatus()
        {
            CalibrationStatus status = new CalibrationStatus();
            byte cal = ReadRegister(BNO055_CALIB_STAT_ADDR);

            status.System = (byte)((cal >> 6) & 0x03);
            status.Gyro = (byte)((cal >> 4) & 0x03);
            status.Accel = (byte)((cal >> 2) & 0x03);
            status.Mag = (byte)(cal & 0x03);

            return status;
        }

        public byte[] GetCalibration()
        {
            byte[] commandBuffer = new byte[] { 0xAA, 0x01, ACCEL_OFFSET_X_LSB_ADDR, 22 };
            byte[] calib = new byte[22];

            SetMode(OperationMode.OPERATION_MODE_CONFIG);
            calib = ReadData(commandBuffer, 22);
            SetMode(OperatingMode);

            return calib;
        }

        public byte[] SetCalibration(byte[] calib)
        {
            byte[] commandBuffer = new byte[26];

            commandBuffer[0] = 0xAA;
            commandBuffer[1] = 0x00;
            commandBuffer[2] = ACCEL_OFFSET_X_LSB_ADDR;
            commandBuffer[3] = 22;
            for (int i = 0; i < 22; i++)
                commandBuffer[i + 4] = calib[i];

            SetMode(OperationMode.OPERATION_MODE_CONFIG);
            WriteData(commandBuffer);
            SetMode(OperatingMode);

            return calib;
        }

        public AxisRemap GetAxisRemap()
        {
            AxisRemap map = new AxisRemap();
            byte mapConfig = ReadRegister(BNO055_AXIS_MAP_CONFIG_ADDR);
            byte signConfig = ReadRegister(BNO055_AXIS_MAP_SIGN_ADDR);

            map.Z = (byte)((mapConfig >> 4) & 0x03);
            map.Y = (byte)((mapConfig >> 2) & 0x03);
            map.X = (byte) (mapConfig & 0x03);

            map.ZSign = (byte)((signConfig >> 2) & 0x01);
            map.YSign = (byte)((signConfig >> 1) & 0x01);
            map.XSign = (byte) (signConfig & 0x01);

            return map;
        }

        public void SetAxisRemap(AxisRemap map)
        {
            byte mapConfig = 0;
            byte signConfig = 0;

            mapConfig |= (byte)((map.Z & 0x03) << 4);
            mapConfig |= (byte)((map.Y & 0x03) << 2);
            mapConfig |= (byte) (map.X & 0x03);

            signConfig |= (byte)((map.ZSign & 0x01) << 2);
            signConfig |= (byte)((map.YSign & 0x01) << 1);
            signConfig |= (byte) (map.XSign & 0x01);

            SetMode(OperationMode.OPERATION_MODE_CONFIG);
            WriteRegister(BNO055_AXIS_MAP_CONFIG_ADDR, mapConfig);
            WriteRegister(BNO055_AXIS_MAP_SIGN_ADDR, signConfig);
            SetMode(OperatingMode);
        }

        public Euler ReadEuler()
        {
            Euler euler = new Euler();

            List<short> vector = ReadVector(BNO055_EULER_H_LSB_ADDR);
            if (3 == vector.Count)
            {
                euler.Heading = vector[0] / 16.0;
                euler.Roll = vector[1] / 16.0;
                euler.Pitch = vector[2] / 16.0;
                return euler;
            }
            else
                return null;
        }

        public Coords ReadMagnetometer()
        {
            Coords mag = new Coords();

            List<short> vector = ReadVector(BNO055_MAG_DATA_X_LSB_ADDR);
            if (3 == vector.Count())
            {
                mag.X = vector[0] / 16.0;
                mag.Y = vector[1] / 16.0;
                mag.Z = vector[2] / 16.0;
            }
            return mag;
        }

        public Coords ReadGyroscope()
        {
            Coords gyro = new Coords();

            List<short> vector = ReadVector(BNO055_GYRO_DATA_X_LSB_ADDR);
            if (3 == vector.Count())
            {
                gyro.X = vector[0] / 900.0;
                gyro.Y = vector[1] / 900.0;
                gyro.Z = vector[2] / 900.0;
            }
            return gyro;
        }

        public Coords ReadAccelerometer()
        {
            Coords accel = new Coords();

            List<short> vector = ReadVector(BNO055_ACCEL_DATA_X_LSB_ADDR);
            if (3 == vector.Count())
            {
                accel.X = vector[0] / 100.0;
                accel.Y = vector[1] / 100.0;
                accel.Z = vector[2] / 100.0;
            }
            return accel;
        }

        public Coords ReadLinearAccel()
        {
            Coords accel = new Coords();

            List<short> vector = ReadVector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR);
            if (3 == vector.Count())
            {
                accel.X = vector[0] / 100.0;
                accel.Y = vector[1] / 100.0;
                accel.Z = vector[2] / 100.0;
            }
            return accel;
        }

        public Coords ReadGravity()
        {
            Coords gravity = new Coords();

            List<short> vector = ReadVector(BNO055_GRAVITY_DATA_X_LSB_ADDR);
            if (3 == vector.Count())
            {
                gravity.X = vector[0] / 100.0;
                gravity.Y = vector[1] / 100.0;
                gravity.Z = vector[2] / 100.0;
            }
            return gravity;
        }

        public Quaternion ReadQuaternion()
        {
            Quaternion quat = new Quaternion();

            List<short> vector = ReadVector(BNO055_QUATERNION_DATA_W_LSB_ADDR, 4);
            if (4 == vector.Count())
            {
                double scale = (1.0 / (1 << 14));

                quat.W = vector[0] / scale;
                quat.X = vector[1] / scale;
                quat.Y = vector[2] / scale;
                quat.Z = vector[3] / scale;
            }
            return quat;
        }

        public int ReadTemp()
        {
            byte temp = ReadRegister(BNO055_TEMP_ADDR);

            if (temp > 127)
                return temp - 256;
            else
                return temp;
        }
        #endregion
    }
}
