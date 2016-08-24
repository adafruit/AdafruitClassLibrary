/*------------------------------------------------------------------------
  Windows IoT library to control Adafruit Ultimate GPS Hat.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------
  This file is part of the Adafruit Windows IoT Class Library

  Adafruit GPS is free software: you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public License
  as published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  Adafruit GPS is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with DotStar.  If not, see <http://www.gnu.org/licenses/>.
  ------------------------------------------------------------------------*/
  using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.SerialCommunication;
using Windows.Storage.Streams;
using System.Collections.ObjectModel;
using Windows.Devices.Enumeration;
using System.Threading;
using System.Globalization;

namespace AdafruitClassLibrary
{
    public class GPS
    {
        #region Properties
        private SerialDevice SerialPort { get; set; }
        private DataWriter DataWriterObject { get; set; }
        private DataReader DataReaderObject { get; set; }
        private Task ReadTask { get; set; }

        private uint BaudRate { get; set; }

        private GPSRMC m_RMC = new GPSRMC();
        private GPSGGA m_GGA = new GPSGGA();
        private GPSGLL m_GLL = new GPSGLL();
        private GPSVTG m_VTG = new GPSVTG();
        private GPSGSA m_GSA = new GPSGSA();
        private GPSGSV m_GSV = new GPSGSV();

        public GPSRMC RMC
        {
            get
            {
                lock (m_RMC)
                {
                    return m_RMC;
                }
            }
            set
            {
                lock (m_RMC)
                {
                    m_RMC = value;
                }
            }
        }

        public GPSGGA GGA
        {
            get
            {
                lock (m_GGA)
                {
                    return m_GGA;
                }
            }
            set
            {
                lock (m_GGA)
                {
                    m_GGA = value;
                }
            }
        }

        public GPSGLL GLL
        {
            get
            {
                lock (m_GLL)
                {
                    return m_GLL;
                }
            }
            set
            {
                lock (m_GLL)
                {
                    m_GLL = value;
                }
            }
        }

        public GPSVTG VTG
        {
            get
            {
                lock (m_VTG)
                {
                    return m_VTG;
                }
            }
            set
            {
                lock (m_VTG)
                {
                    m_VTG = value;
                }
            }
        }

        public GPSGSA GSA
        {
            get
            {
                lock (m_GSA)
                {
                    return m_GSA;
                }
            }
            set
            {
                lock (m_GSA)
                {
                    m_GSA = value;
                }
            }
        }

        public GPSGSV GSV
        {
            get
            {
                lock (m_GSV)
                {
                    return m_GSV;
                }
            }
            set
            {
                lock (m_GSV)
                {
                    m_GSV = value;
                }
            }
        }

        #endregion

        CancellationTokenSource ReadCancellationTokenSource;

        public GPS()
        {
            SerialPort = null;
            DataWriterObject = null;
            DataReaderObject = null;
            GLLEvent += (o, e) => { };
            RMCEvent += (o, e) => { };
            VTGEvent += (o, e) => { };
            GGAEvent += (o, e) => { };
            GSAEvent += (o, e) => { };
            GSVEvent += (o, e) => { };
        }

        #region GPS Commands
        public async Task SetSentencesReporting(int GLLfreq, int RMCfreq, int VTGfreq, int GGAfreq, int GSAfreq, int GSVfreq)
        {
            string cmdString = "PMTK314";
            cmdString = string.Concat(string.Concat(cmdString, ","), Math.Min(5, GLLfreq).ToString());
            cmdString = string.Concat(string.Concat(cmdString, ","), Math.Min(5, RMCfreq).ToString());
            cmdString = string.Concat(string.Concat(cmdString, ","), Math.Min(5, VTGfreq).ToString());
            cmdString = string.Concat(string.Concat(cmdString, ","), Math.Min(5, GGAfreq).ToString());
            cmdString = string.Concat(string.Concat(cmdString, ","), Math.Min(5, GSAfreq).ToString());
            cmdString = string.Concat(string.Concat(cmdString, ","), Math.Min(5, GSVfreq).ToString());

            cmdString = string.Concat(cmdString, ",0,0,0,0,0,0,0,0,0,0,0,0,0");
            uint checksum = 0;
            for (int index = 0; index < cmdString.Length; index++)
                checksum ^= cmdString[index];

            cmdString = string.Concat(string.Concat("$", cmdString), "*");
            cmdString = string.Concat(cmdString, checksum.ToString("X2"));
            cmdString = string.Concat(cmdString, "\r\n");

            await SendCommand(cmdString);

        }

        public async Task SetUpdateFrequency(double freqHz)
        {
            string cmdString = "PMTK220";
            cmdString = string.Concat(string.Concat(cmdString, ","), (1000 / freqHz).ToString());

            uint checksum = 0;
            for (int index = 0; index < cmdString.Length; index++)
                checksum ^= cmdString[index];

            cmdString = string.Concat(string.Concat("$", cmdString), "*");
            cmdString = string.Concat(cmdString, checksum.ToString("X2"));
            cmdString = string.Concat(cmdString, "\r\n");

            await SendCommand(cmdString);
        }

        public async Task SetBaudRate(uint baudrate)
        {
            string cmdString = "PMTK251";
            cmdString = string.Concat(string.Concat(cmdString, ","),baudrate.ToString());

            uint checksum = 0;
            for (int index = 0; index < cmdString.Length; index++)
                checksum ^= cmdString[index];

            cmdString = string.Concat(string.Concat("$", cmdString), "*");
            cmdString = string.Concat(cmdString, checksum.ToString("X2"));
            cmdString = string.Concat(cmdString, "\r\n");

            await SendCommand(cmdString);
        }


        public async Task SendPMTKCommand(string pmtk)
        {
            await SendCommand(pmtk);
        }

        #endregion

        #region Serial Control
        public bool Connected
        {
            get { return SerialPort != null; }
        }

        /// <summary>
        /// ConnectToUART
        /// - Use SerialDevice.GetDeviceSelector to find serial device named "UART0". 
        ///   This is the built-in Raspberry Pi serial port.
        /// </summary>
        public async Task ConnectToUART(uint? baudRate = 9600, string uartID = "UART0")
        {
            if (null != baudRate)
                BaudRate = (uint)baudRate;
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
                        SerialPort.WriteTimeout = TimeSpan.FromMilliseconds(1000);
                        SerialPort.ReadTimeout = TimeSpan.FromMilliseconds(100);
                        SerialPort.BaudRate = BaudRate;
                        SerialPort.Parity = SerialParity.None;
                        SerialPort.StopBits = SerialStopBitCount.One;
                        SerialPort.DataBits = 8;
                        SerialPort.Handshake = SerialHandshake.None;

                        // Create the DataReader object and attach to InputStream
                        DataReaderObject = new DataReader(SerialPort.InputStream);
                        // Create the DataWriter object and attach to OutputStream
                        DataWriterObject = new DataWriter(SerialPort.OutputStream);

                        ReadCancellationTokenSource = new CancellationTokenSource();
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
        
        public void StartReading()
        {
            ReadCancellationTokenSource = new CancellationTokenSource();
            ReadTask = ReadAsync(ReadCancellationTokenSource.Token);
        }

        public void StopReading()
        {
            ReadCancellationTokenSource.Cancel();
            if (null != ReadTask)
            {
                ReadTask.Wait();
                ReadTask = null;
            }
        }

        public void DisconnectFromUART()
        {
            StopReading();
            if (null != DataReaderObject)
            {
                //DataReaderObject.DetachStream();
                //DataReaderObject.DetachBuffer();
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

        public async Task ReadAsync(CancellationToken cancellationToken)
        {
            Task<UInt32> loadAsyncTask;

            uint ReadBufferLength = 128;
            UInt32 bytesRead = 0;

            // Set InputStreamOptions to complete the asynchronous read operation when one or more bytes is available
            DataReaderObject.InputStreamOptions = InputStreamOptions.Partial;

            int retries = 5;

            string rawSentence = "";
            string leftovers = "";

            while ((!cancellationToken.IsCancellationRequested) && (retries > 0))
            {
                try
                {
                    while (true)
                    {
                        // If task cancellation was requested, comply
                        cancellationToken.ThrowIfCancellationRequested();

                        // Create a task object to wait for data on the serialPort.InputStream
                        loadAsyncTask = DataReaderObject.LoadAsync(ReadBufferLength).AsTask(cancellationToken);
                        // Launch the task and wait
                        bytesRead = await loadAsyncTask;
                        if (bytesRead > 0)
                        {
                            rawSentence = DataReaderObject.ReadString(bytesRead);
                            rawSentence = string.Concat(leftovers, rawSentence);
                            string[] splitSentences = rawSentence.Split(new char[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
                            leftovers = "";

                            foreach (string str in splitSentences)
                            {
                                if ((str.Length >= 3) && (str.LastIndexOf('*') == str.Length - 3))
                                {
                                    System.Diagnostics.Debug.WriteLine(str);
                                    DispatchSentence(str);
                                }
                                else
                                {
                                    leftovers = str;
                                }
                            }
                        }
                        retries = 5;
                    }
                }
                catch (OperationCanceledException ex)
                {
                    System.Diagnostics.Debug.WriteLine(string.Format("Exiting GPS read task"));
                }
                catch (ArgumentOutOfRangeException ex)
                {
                    //do nothing. this tends to sort itself out eventually
                    //System.Diagnostics.Debug.WriteLine(string.Format("ArgumentOutOfRangeException: {0}", ex.Message));
                    for (int b = 0; b < bytesRead; b++)
                    {
                        DataReaderObject.ReadByte();
                      //  bytesRead--;
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine(string.Format("Error reading from GPS: {0}", ex.Message));
                    leftovers = "";
                    retries--;
                }
            }
            SerialPort.Dispose();
            SerialPort = null;
        }

        private async Task SendCommand(string cmdString)
        {
            try
            {
                //Launch the WriteAsync task to perform the write
                Task<UInt32> storeAsyncTask;

                if (cmdString.Length != 0)
                {
                    // Load the text from the sendText input text box to the dataWriter object
                    DataWriterObject.WriteString(cmdString);

                    // Launch an async task to complete the write operation
                    storeAsyncTask = DataWriterObject.StoreAsync().AsTask();

                    UInt32 bytesWritten = await storeAsyncTask;
                    if (bytesWritten != cmdString.Length)
                        throw new Exception("Bytes written does not match command length");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(string.Format("Error writing to GPS: {0}", ex.Message));
            }
        }

        #endregion

        #region Parsing
        private bool IsValid(string sentence)
        {
            bool validSentence = false;
            if (null != sentence)
            {
                try
                {
                    if ('$' == sentence[0])
                    {
                        uint checksum = 0;
                        for (int index = 1; index < sentence.Length - 3; index++)
                            checksum ^= sentence[index];
                        uint targetChecksum = Convert.ToUInt16(sentence.Substring((sentence.LastIndexOf('*') + 1), 2), 16);
                        validSentence = (checksum == targetChecksum);
                    }
                    else
                        throw new Exception("Missing '$'");
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine(string.Format("Invalid RMC Sentence: {0}\r\n     {1}", sentence, ex.Message));
                }
            }
            if (!validSentence)
                System.Diagnostics.Debug.WriteLine(string.Format("Invalid RMC Sentence: {0}", sentence));

            return validSentence;
        }

        // GPGLL  - Geographic Position - Latitude longitude
        // GPRMC  - Recommended Minimum Specific GNSS Sentence
        // GPVTG  - Course over Ground and Ground Speed
        // GPGGA  - GPS Fix Data
        // GPGSA  - GNSS DOPS and Active Satellites
        // GPGSV  - GNSS Satellites in View
        private void DispatchSentence(string sentence)
        {
            if (IsValid(sentence))
            {
                string sentenceType = sentence.Substring(1, 5);
                switch (sentenceType)
                {
                    case "GPRMC":
                        RMC = new GPSRMC(sentence.Substring(sentence.IndexOf(',') + 1));
                        OnRMCEvent(RMC);
                        break;
                    case "GPGGA":
                        GGA = new GPSGGA(sentence.Substring(sentence.IndexOf(',') + 1));
                        OnGGAEvent(GGA);
                        break;
                    case "GPGLL":
                        GLL = new GPSGLL(sentence.Substring(sentence.IndexOf(',') + 1));
                        OnGLLEvent(GLL);
                        break;
                    case "GPVTG":
                        VTG = new GPSVTG(sentence.Substring(sentence.IndexOf(',') + 1));
                        OnVTGEvent(VTG);
                        break;
                    case "GPGSA":
                        GSA = new GPSGSA(sentence.Substring(sentence.IndexOf(',') + 1));
                        OnGSAEvent(GSA);
                        break;
                    case "GPGSV":
                        GSV = new GPSGSV(sentence.Substring(sentence.IndexOf(',') + 1));
                        OnGSVEvent(GSV);
                        break;

                    default:
                        break;
                }
            }
        }

        private bool CheckPMTKAck(string sentence, string cmdNumber)
        {
            bool isAck = false;
            if (IsValid(sentence))
            {
                string[] sections = sentence.Split(new char[] { ',', '*' });
                isAck = ("$PMTK001" == sections[0]) && (cmdNumber == sections[1]) && ("3" == sections[2]);
            }
            return isAck;
        }

        #endregion

        #region Data Classes
        public class GPSRMC : SentenceBase
        {
            public DateTime TimeStamp { get; set; }
            public bool Valid { get; set; }
            public double? Latitude { get; set; }
            public string LatHemisphere { get; set; }
            public double? Longitude { get; set; }
            public string LonHemisphere { get; set; }
            public double? Speed { get; set; }
            public double? Course { get; set; }
            public DateTime DateStamp { get; set; }
            public double? MagVariation { get; set; }
            public string VarDirection { get; set; }
            public double? LatDegrees { get; set; }
            public double? LonDegrees { get; set; }

            public GPSRMC()
            {
            }

            public GPSRMC(string sentence)
            {
                string[] splitString = sentence.Split(new char[] { ',', '*' });

                TimeStamp = ParseTimeStamp(splitString[0]);            // Time Stamp
                Valid = splitString[1] == "A" ? true : false;          // validity -A - ok, V - invalid
                Latitude = ParseDouble(splitString[2]);                // current Latitude
                LatHemisphere = splitString[3];                        // North/ South
                Longitude = ParseDouble(splitString[4]);               // current Longitude
                LonHemisphere = splitString[5];                        // East/ West
                Speed = ParseDouble(splitString[6]);                   // Speed in knots
                Course = ParseDouble(splitString[7]);                  // True course
                DateStamp = ParseDateStamp(splitString[8]);            // Date Stamp
                MagVariation = ParseDouble(splitString[9]);            // Magnetic Variation
                VarDirection = splitString[10];                        // East/ West

                LatDegrees = ToDegrees(Latitude);
                LonDegrees = ToDegrees(Longitude);
            }


        }

        public class GPSGGA : SentenceBase
        {
            public enum FixQuality { noFix = 0, gpsFix = 1, dgpsFix = 2 }

            public DateTime TimeStamp { get; set; }
            public double? Latitude { get; set; }
            public string LatHemisphere { get; set; }
            public double? Longitude { get; set; }
            public string LonHemisphere { get; set; }
            public FixQuality Quality { get; set; }
            public int? Satellites { get; set; }
            public double? Dilution { get; set; }
            public double? Altitude { get; set; }
            public string AltUnits { get; set; }
            public double? Geoidal { get; set; }
            public string GeoidalUnits { get; set; }
            public double? DGPSAge { get; set; }
            public int? DGPS_ID { get; set; }
            public double? LatDegrees { get; set; }
            public double? LonDegrees { get; set; }

            public GPSGGA()
            {
            }

            public GPSGGA(string sentence)
            {
                string[] splitString = sentence.Split(new char[] { ',', '*' });

                TimeStamp = ParseTimeStamp(splitString[0]);         // UTC of Position
                Latitude = ParseDouble(splitString[1]);             // Latitude
                LatHemisphere = splitString[2];                     // N or S
                Longitude = ParseDouble(splitString[3]);            // Longitude
                LonHemisphere = splitString[4];                     // E or W
                Quality = (FixQuality)ParseInt(splitString[5]);     // GPS quality indicator(0=invalid; 1=GPS fix; 2=Diff.GPS fix)
                Satellites = ParseInt(splitString[6]);              // Number of satellites in use[not those in view]
                Dilution = ParseDouble(splitString[7]);             // Horizontal dilution of position
                Altitude = ParseDouble(splitString[8]);             // Antenna altitude above/below mean sea level(geoid)
                AltUnits = splitString[9];                          // Meters(Antenna height unit)
                Geoidal = ParseDouble(splitString[10]);             // Geoidal separation(Diff.between WGS - 84 earth ellipsoid and mean sea level.  -=geoid is below WGS-84 ellipsoid)
                GeoidalUnits = splitString[11];                     // Meters(Units of geoidal separation)
                DGPSAge = ParseDouble(splitString[12]);             // Age in seconds since last update from diff.reference station
                DGPS_ID = ParseInt(splitString[13]);                // Diff.reference station ID#

                LatDegrees = ToDegrees(Latitude);
                LonDegrees = ToDegrees(Longitude);
            }
        }

        public class GPSGLL : SentenceBase
        {
            public DateTime TimeStamp { get; set; }
            public bool Valid { get; set; }
            public double? Latitude { get; set; }
            public string LatHemisphere { get; set; }
            public double? Longitude { get; set; }
            public string LonHemisphere { get; set; }
            public double? LatDegrees { get; set; }
            public double? LonDegrees { get; set; }

            public GPSGLL()
            {
            }

            public GPSGLL(string sentence)
            {
                string[] splitString = sentence.Split(new char[] { ',', '*' });

                Latitude = ParseDouble(splitString[0]);                // current Latitude
                LatHemisphere = splitString[1];                        // North/ South
                Longitude = ParseDouble(splitString[2]);               // current Longitude
                LonHemisphere = splitString[3];                        // East/ West
                TimeStamp = ParseTimeStamp(splitString[4]);            // Time Stamp
                Valid = splitString[5] == "A" ? true : false;          // validity -A - ok, V - invalid

                LatDegrees = ToDegrees(Latitude);
                LonDegrees = ToDegrees(Longitude);
            }
        }

        public class GPSVTG : SentenceBase
        {
            public double? TrackTrue { get; set; }
            public string TT { get; set; }
            public double? TrackMag { get; set; }
            public string TM { get; set; }
            public double? SpeedKnots { get; set; }
            public string SKn { get; set; }
            public double? SpeedKm { get; set; }
            public string SKm { get; set; }

            public GPSVTG()
            {
            }

            public GPSVTG(string sentence)
            {
                string[] splitString = sentence.Split(new char[] { ',', '*' });

                TrackTrue = ParseDouble(splitString[0]);   // Track true north
                TT = splitString[1];                        // "T"
                TrackMag = ParseDouble(splitString[2]);     // Track magnetic north
                TM = splitString[3];                        // "M"
                SpeedKnots = ParseDouble(splitString[4]);   // Speed in knots
                SKn = splitString[5];                       // "N"
                SpeedKm = ParseDouble(splitString[6]);      // Speed in km/h
                SKm = splitString[7];                       // "K"
            }
        }

        public class GPSGSA : SentenceBase
        {
            public enum FixType { noFix = 1, fix2D = 2, fix3D = 3 }

            public string Mode { get; set; }
            public FixType Fix { get; set; }
            public List<int?> SVIDs { get; set; }
            public double? PDOP { get; set; }
            public double? HDOP { get; set; }
            public double? VDOP { get; set; }

            public GPSGSA()
            {
            }

            public GPSGSA(string sentence)
            {
                string[] splitString = sentence.Split(new char[] { ',', '*' });

                Mode = splitString[0];                        // "M" - Manual, "A" - Automatic
                Fix = (FixType)ParseInt(splitString[1]);      // 0: no fix, 1 : 2D, 2 : 3D
                SVIDs = new List<int?>();                     //IDs of atellites in view
                for (int i = 2; i < 14; i++)
                    SVIDs.Add(ParseInt(splitString[i]));
                PDOP = ParseDouble(splitString[14]);          // PDOP
                HDOP = ParseDouble(splitString[15]);          // HDOP
                VDOP = ParseDouble(splitString[16]);          // VDOP
            }


        }

        public class GPSGSV : SentenceBase
        {
            public class SVRecord
            {
                public int? PRN { get; set; }
                public int? Elevation { get; set; }
                public int? Azimuth { get; set; }
                public int? SNR { get; set; }
            }

            public int? MsgCount { get; set; }
            public int? MsgNumber { get; set; }
            public int? Satellites { get; set; }
            public List<SVRecord> SVList { get; set; }

            public GPSGSV()
            {
            }

            public GPSGSV(string sentence)
            {
                string[] splitString = sentence.Split(new char[] { ',', '*' });

                MsgCount = ParseInt(splitString[0]);                                   // 1    = Total number of messages of this type in this cycle
                MsgNumber = ParseInt(splitString[1]);                                  // 2    = Message number
                Satellites = ParseInt(splitString[2]);                                 // 3    = Total number of SVs in view
                SVList = new List<SVRecord>();
                int stringIndex = 3;
                int recordCount = (MsgNumber < MsgCount) ? 4 : (((int)Satellites - 1) % 4) + 1;
                for (int svNum = 1; svNum <= recordCount; svNum++)
                {
                    SVRecord record = new SVRecord();
                    record.PRN = ParseInt(splitString[stringIndex++]);                 // 4    = SV PRN number
                    record.Elevation = ParseInt(splitString[stringIndex++]);           // 5    = Elevation in degrees, 90 maximum
                    record.Azimuth = ParseInt(splitString[stringIndex++]);             // 6    = Azimuth, degrees from true north, 000 to 359
                    record.SNR = ParseInt(splitString[stringIndex++]);                 // 7    = SNR, 00-99 dB (null when not tracking)
                    SVList.Add(record);                                                // 8-11 = Information about second SV, same as field 4-7
                }                                                                      // 12-15= Information about third SV, same as field 4-7
            }                                                                          // 16-19= Information about fourth SV, same as field 4-7
        }

        public class SentenceBase : EventArgs
        {
            protected double? ParseDouble(string str)
            {
                double? result = null;
                try
                {
                    if ("" != str)
                        result = Convert.ToDouble(str);
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine(string.Format("Error parsing double: {0}\r\n     {1}", str, ex.Message));
                }
                return result;
            }

            protected int? ParseInt(string str)
            {
                int? result = null;
                try
                {
                    if ("" != str)
                        result = Convert.ToInt32(str);
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine(string.Format("Error parsing integer: {0}\r\n     {1}", str, ex.Message));
                }
                return result;
            }

            protected DateTime ParseTimeStamp(string timeStr)
            {
                timeStr = timeStr.Insert(4, ":");
                timeStr = timeStr.Insert(2, ":");

                DateTime stamp = DateTime.Parse(timeStr, CultureInfo.InvariantCulture);
                return stamp;
            }
            protected DateTime ParseDateStamp(string dateStr)
            {
                dateStr = dateStr.Insert(4, "-");
                dateStr = dateStr.Insert(2, "-");

                //this wretched kludge exists because DateTime.ParseExact doesn't appear to work.
                //  so, we have to rearrange the month and day to make Parse happy
                string[] components = dateStr.Split(new char[]{'-'});
                dateStr = components[1] + '-' + components[0] + '-' + components[2];

                DateTime stamp = DateTime.Parse(dateStr, CultureInfo.InvariantCulture);
                return stamp;
            }

            protected double? ToDegrees(double? latlong)
            {
                double? degrees = null;
                if (null != latlong)
                {
                    degrees = (int)(latlong / 100);
                    double? minutes = (latlong % 100) / 60;
                    degrees = degrees + minutes;
                }
                return degrees;
            }
    }
    #endregion
    #region Events
    // Delegate declarations
    //
    public delegate void RMCEventHandler(object sender, GPSRMC e);
        public delegate void GLLEventHandler(object sender, GPSGLL e);
        public delegate void VTGEventHandler(object sender, GPSVTG e);
        public delegate void GGAEventHandler(object sender, GPSGGA e);
        public delegate void GSAEventHandler(object sender, GPSGSA e);
        public delegate void GSVEventHandler(object sender, GPSGSV e);

        public event RMCEventHandler RMCEvent;
        public event GLLEventHandler GLLEvent;
        public event VTGEventHandler VTGEvent;
        public event GGAEventHandler GGAEvent;
        public event GSAEventHandler GSAEvent;
        public event GSVEventHandler GSVEvent;

        protected virtual void OnRMCEvent(GPSRMC e)
        {
            RMCEvent(this, e);
        }

        protected virtual void OnGLLEvent(GPSGLL e)
        {
            GLLEvent(this, e);
        }

        protected virtual void OnVTGEvent(GPSVTG e)
        {
            VTGEvent(this, e);
        }

        protected virtual void OnGGAEvent(GPSGGA e)
        {
            GGAEvent(this, e);
        }

        protected virtual void OnGSAEvent(GPSGSA e)
        {
            GSAEvent(this, e);
        }

        protected virtual void OnGSVEvent(GPSGSV e)
        {
            GSVEvent(this, e);
        }


        #endregion


    }

}
