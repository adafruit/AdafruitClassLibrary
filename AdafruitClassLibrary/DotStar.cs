/*------------------------------------------------------------------------
  Adafruit Class Library for Windows Core IoT: Adafruit DotStar.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------*/

using System;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using Windows.Devices.Spi;

namespace AdafruitClassLibrary
{
    public class DotStar
    {
        public const int DOTSTAR_RGB = (1 | (2 << 8) | (3 << 16));
        public const int DOTSTAR_RBG = (1 | (3 << 8) | (2 << 16));
        public const int DOTSTAR_GRB = (2 | (1 << 8) | (3 << 16));
        public const int DOTSTAR_GBR = (3 | (1 << 8) | (2 << 16));
        public const int DOTSTAR_BRG = (2 | (3 << 8) | (1 << 16));
        public const int DOTSTAR_BGR = (3 | (2 << 8) | (1 << 16));

        public uint NumPixels { get; set; }
        private UInt32 ColorOrder { get; set; }
        private int DataPin { get; set; }
        private int ClockPin { get; set; }
        private int ROffset { get; set; }
        private int GOffset { get; set; }
        private int BOffset { get; set; }

        private byte[] Pixels { get; set; }

        private GpioPin SWSpiDataPin { get; set; }
        private GpioPin SWSpiClockPin { get; set; }
        private SpiDevice HWSpiDevice { get; set; }
        public bool HW_SPI { get; set; }

        // Stored brightness value is different than what's passed.  This
        // optimizes the actual scaling math later, allowing an 8x8-bit
        // multiply and taking the MSB.  'brightness' is a byte; adding 1
        // here may (intentionally) roll over...so 0 = max brightness (color
        // values are interpreted literally; no scaling), 1 = min brightness
        // (off), 255 = just below max brightness.
        private byte m_Brightness = 255;

        public byte Brightness
        {
            get
            {
                return (byte)(m_Brightness - 1);
            }
            set
            {
                m_Brightness = (byte)(value + 1);
            }
        }

        public DotStar(uint numPixels, UInt32 colorOrder = DOTSTAR_BRG) : this(numPixels, 255, 255, colorOrder)
        {
        }

        public DotStar(uint numPixels, int dataPin, int clockPin, UInt32 colorOrder = DOTSTAR_BRG)
        {
            UpdateLength(numPixels);
            ColorOrder = colorOrder;
            DataPin = dataPin;
            ClockPin = clockPin;
            Brightness = 255;

            ROffset = (int)(ColorOrder & 255);
            GOffset = (int)((ColorOrder >> 8) & 255);
            BOffset = (int)((ColorOrder >> 16) & 255);

            HW_SPI = (dataPin == 255);
        }

        public async Task BeginAsync()
        {
            if (HW_SPI)
                await hw_spi_initAsync();
            else
                sw_spi_init();

            ScopeTriggerInit(6);
        }

        public void End()
        {
            if (HW_SPI)
                hw_spi_end();
            else
                sw_spi_end();

            ScopeTriggerEnd();
        }

        #region Software Spi

        private void sw_spi_init()
        {
            var gpio = GpioController.GetDefault();

            if (gpio != null)
            {
                SWSpiDataPin = gpio.OpenPin(DataPin);
                SWSpiDataPin.Write(GpioPinValue.Low);
                SWSpiDataPin.SetDriveMode(GpioPinDriveMode.Output);

                SWSpiClockPin = gpio.OpenPin(ClockPin);
                SWSpiClockPin.Write(GpioPinValue.Low);
                SWSpiClockPin.SetDriveMode(GpioPinDriveMode.Output);
            }
        }

        private void sw_spi_out(byte n)
        { // Bitbang SPI write
            for (int i = 8; 0 != i--; n <<= 1)
            {
                if (0x80 == (n & 0x80))
                    SWSpiDataPin.Write(GpioPinValue.High);
                else
                    SWSpiDataPin.Write(GpioPinValue.Low);

                SWSpiClockPin.Write(GpioPinValue.High);
                SWSpiClockPin.Write(GpioPinValue.Low);
            }
        }

        private void sw_spi_out(byte[] pixels)
        {
            foreach (byte p in pixels)
                sw_spi_out(p);
        }

        private void sw_spi_end()
        { // Stop 'soft' SPI
            SWSpiDataPin.SetDriveMode(GpioPinDriveMode.Input);
            SWSpiClockPin.SetDriveMode(GpioPinDriveMode.Input);
        }

        #endregion Software Spi

        #region Hardware Spi

        private async Task hw_spi_initAsync()
        {
            // Get a selector string for bus "SPI0"
            string aqs = SpiDevice.GetDeviceSelector("SPI0");

            // Find the SPI bus controller device with our selector string
            var dis = await DeviceInformation.FindAllAsync(aqs);
            if (dis.Count == 0)
                return; // "SPI0" not found on this system

            // Use chip select line CS0
            var settings = new SpiConnectionSettings(0);

            // Create an SpiDevice with our bus controller and SPI settings
            HWSpiDevice = await SpiDevice.FromIdAsync(dis[0].Id, settings);
        }

        private void hw_spi_out(byte n)
        {
            byte[] writeBuf = { n };
            HWSpiDevice.Write(writeBuf);
        }

        private void hw_spi_out(byte[] pixels)
        {
            HWSpiDevice.Write(pixels);
        }

        private void hw_spi_end()
        {
            HWSpiDevice.Dispose();
        }

        #endregion Hardware Spi

        #region DotStarOptimized functions

        public void UpdateLength(uint numPixels)
        {
            NumPixels = numPixels;
            Pixels = new byte[NumPixels * 4];
            Clear();
        }

        public void Clear()
        {
            for (int pixel = 0; pixel < NumPixels; pixel++)
            {
                Pixels[pixel * 4] = 0xFF;
                Pixels[pixel * 4 + ROffset] = 0;
                Pixels[pixel * 4 + GOffset] = 0;
                Pixels[pixel * 4 + BOffset] = 0;
            }
        }

        // Set pixel color, separate R,G,B values (0-255 ea.)
        public void SetPixelColor(int pixel, byte r, byte g, byte b)
        {
            if ((pixel >= 0) && (pixel < NumPixels))
            {
                Pixels[pixel * 4] = 0xFF;
                Pixels[pixel * 4 + ROffset] = r;
                Pixels[pixel * 4 + GOffset] = g;
                Pixels[pixel * 4 + BOffset] = b;
            }
        }

        // Set pixel color, 'packed' RGB value (0x00000000 - 0x00FFFFFF)
        public void SetPixelColor(int pixel, UInt32 c)
        {
            if ((pixel >= 0) && (pixel < NumPixels))
            {
                byte[] colors = { 0xFF, (byte)((c >> 16) & 0xFF), (byte)((c >> 8) & 0xFF), (byte)(c & 0xFF) };
                SetPixelColor(pixel, colors[1], colors[2], colors[3]);
            }
        }

        // Get pixel color, 'packed' RGB value (0x00000000 - 0x00FFFFFF)
        public UInt32 GetPixelColor(int pixel)
        {
            UInt32 color = 0;
            if ((pixel >= 0) && (pixel < NumPixels))
            {
                color = (UInt32)(Pixels[pixel * 4 + ROffset] << 16 | Pixels[pixel * 4 + GOffset] << 8 | Pixels[pixel * 4 + BOffset]);
            }
            return color;
        }

        // Convert separate R,G,B to packed value
        public UInt32 Color(uint r, uint g, uint b)
        {
            byte[] colors = { 0xFF, 0, 0, 0 };
            colors[ROffset] = (byte)r;
            colors[GOffset] = (byte)g;
            colors[BOffset] = (byte)b;

            return (UInt32)(colors[ROffset] << 16 | colors[GOffset] << 8 | colors[BOffset]);
        }

        public void Show()
        {
            TriggerScopePin(true);
            byte[] m_startFrame = { 0, 0, 0, 0 };
            byte[] m_endFrame = { 0xFF, 0xFF, 0xFF, 0xFF };

            if (null != Pixels)
            {
                if (HW_SPI)
                {
                    hw_spi_out(m_startFrame);    // Start-frame marker
                    if (255 != Brightness)
                    {                     // Scale pixel brightness on output
                        int brite = Brightness + 1;
                        byte[] adjusted = new byte[NumPixels * 4];
                        for (int pixel = 0; pixel < NumPixels; pixel++)
                        {                               // For each pixel...
                            adjusted[pixel * 4] = 0xFF;
                            adjusted[pixel * 4 + 1] = (byte)(Pixels[pixel * 4 + 1] * brite / 256);
                            adjusted[pixel * 4 + 2] = (byte)(Pixels[pixel * 4 + 2] * brite / 256);
                            adjusted[pixel * 4 + 3] = (byte)(Pixels[pixel * 4 + 3] * brite / 256);
                        }
                        hw_spi_out(adjusted);
                    }
                    else
                    {                             // Full brightness (no scaling)
                                                  //  TriggerScopePin(true);
                        hw_spi_out(Pixels);
                        //  TriggerScopePin(false);
                    }

                    // Four end-frame bytes are seemingly indistinguishable from a white
                    // pixel, and empirical testing suggests it can be left out...but it's
                    // always a good idea to follow the datasheet, in case future hardware
                    // revisions are more strict (e.g. might mandate use of end-frame
                    // before start-frame marker).  i.e. let's not remove this.

                    hw_spi_out(m_endFrame); // End-frame marker (see note above)
                }
                else //sw spi
                {
                    sw_spi_out(m_startFrame);    // Start-frame marker

                    if (255 != Brightness)
                    {                     // Scale pixel brightness on output
                        int brite = Brightness + 1;
                        byte[] adjusted = new byte[NumPixels * 4];
                        for (int pixel = 0; pixel < NumPixels; pixel++)
                        {                               // For each pixel...
                            adjusted[pixel * 4] = 0xFF;
                            adjusted[pixel * 4 + 1] = (byte)(Pixels[pixel * 4 + 1] * brite / 256);
                            adjusted[pixel * 4 + 2] = (byte)(Pixels[pixel * 4 + 2] * brite / 256);
                            adjusted[pixel * 4 + 3] = (byte)(Pixels[pixel * 4 + 3] * brite / 256);
                        }
                        sw_spi_out(adjusted);
                    }
                    else
                    {                             // Full brightness (no scaling)
                        sw_spi_out(Pixels);
                    }

                    sw_spi_out(m_endFrame); // End-frame marker (see note above)
                }
            }

            TriggerScopePin(false);
        }

        #endregion DotStarOptimized functions

        #region debugging

        private GpioPin ScopePin { get; set; }

        private void ScopeTriggerInit(int scopePin)
        {
            var gpio = GpioController.GetDefault();

            // Show an error if there is no GPIO controller
            if (gpio == null)
            {
                return;
            }
            ScopePin = gpio.OpenPin(scopePin);
            ScopePin.Write(GpioPinValue.Low);
            ScopePin.SetDriveMode(GpioPinDriveMode.Output);
        }

        private void ScopeTriggerEnd()
        {
            ScopePin.SetDriveMode(GpioPinDriveMode.Input);
            ScopePin.Dispose();
        }

        private void TriggerScopePin(bool triggerOn)
        {
            if (triggerOn)
                ScopePin.Write(GpioPinValue.High);
            else
                ScopePin.Write(GpioPinValue.Low);
        }

        #endregion debugging
    }
}