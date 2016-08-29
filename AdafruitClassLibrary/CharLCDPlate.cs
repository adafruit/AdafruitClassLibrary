/*------------------------------------------------------------------------
  Windows IoT library to control the Adafruit Character LCD Plate.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------
  This file is part of the Adafruit Windows IoT Class Library

  Adafruit CharLCDPlate is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

  MIT license, all text above must be included in any redistribution.
  ------------------------------------------------------------------------*/
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AdafruitClassLibrary
{
    public class CharLCDPlate
    {
        // commands
        private const byte LCD_CLEARDISPLAY = 0x01;
        private const byte LCD_RETURNHOME = 0x02;
        private const byte LCD_ENTRYMODESET = 0x04;
        private const byte LCDDisplayControl = 0x08;
        private const byte LCD_CURSORSHIFT = 0x10;
        private const byte LCD_FUNCTIONSET = 0x20;
        private const byte LCD_SETCGRAMADDR = 0x40;
        private const byte LCD_SETDDRAMADDR = 0x80;

        // flags for display entry mode    
        private const byte LCD_ENTRYRIGHT = 0x00;
        private const byte LCD_ENTRYLEFT = 0x02;
        private const byte LCD_ENTRYSHIFTINCREMENT = 0x01;
        private const byte LCD_ENTRYSHIFTDECREMENT = 0x00;

        // flags for display on/off control
        private const byte LCD_DISPLAYON = 0x04;
        private const byte LCD_DISPLAYOFF = 0x00;
        private const byte LCD_CURSORON = 0x02;
        private const byte LCD_CURSOROFF = 0x00;
        private const byte LCD_BLINKON = 0x01;
        private const byte LCD_BLINKOFF = 0x00;

        // flags for display/cursor shift  
        private const byte LCD_DISPLAYMOVE = 0x08;
        private const byte LCD_CURSORMOVE = 0x00;
        private const byte LCD_MOVERIGHT = 0x04;
        private const byte LCD_MOVELEFT = 0x00;

        // flags for function set          
        private const byte LCD_8BITMODE = 0x10;
        private const byte LCD_4BITMODE = 0x00;
        private const byte LCD_2LINE = 0x08;
        private const byte LCD_1LINE = 0x00;
        private const byte LCD_5x10DOTS = 0x04;
        private const byte LCD_5x8DOTS = 0x00;

        public const byte BUTTON_UP = 0x08;
        public const byte BUTTON_DOWN = 0x04;
        public const byte BUTTON_LEFT = 0x10;
        public const byte BUTTON_RIGHT = 0x02;
        public const byte BUTTON_SELECT = 0x01;

        private MCP23017 MCP { get; set; }

        private byte DisplayFunction { get; set; }
        private byte DisplayControl { get; set; }
        private byte DisplayMode { get; set; }
        private int NumLines { get; set; }
        private int NumColumns { get; set; }
        private int CurrentLine { get; set; }
        private int CurrentCol { get; set; }

        // GPIO Pins
        private byte RSPin { get; set; }
        private byte RWPin { get; set; }
        private byte EnablePin { get; set; }
        private List<byte> DataPins { get; set; }
        private List<byte> ButtonPins { get; set; }
        private List<byte> ColorPins { get; set; }

        public const int RED = 0x1;
        public const int YELLOW = 0x3;
        public const int GREEN = 0x2;
        public const int TEAL = 0x6;
        public const int BLUE = 0x4;
        public const int VIOLET = 0x5;
        public const int WHITE = 0x07;

        //public enum Color { RED = 0x1,
        //                    YELLOW = 0x3,
        //                    GREEN = 0x2,
        //                    TEAL = 0x6,
        //                    BLUE = 0x4,
        //                    VIOLET = 0x5,
        //                    WHITE = 0x07 };

        public CharLCDPlate()
        {
            MCP = new MCP23017();

            DisplayFunction = (byte)(LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS);

            // the I/O expander pinout
            RSPin = 15;
            RWPin = 14;
            EnablePin = 13;
            DataPins = new List<byte>() { 12, 11, 10, 9 };  // really d4, d5, d6, d7
            ButtonPins = new List<byte>() { 0, 1, 2, 3, 4 }; // select, right, up, down, left
            ColorPins = new List<byte>() { 6, 7, 8 };  //red, green, blue
        }

        public async Task Begin(int cols, int lines, int dotsize = LCD_5x8DOTS)
        {
            await MCP.InitMCP23017();

            MCP.pinMode(RWPin, MCP23017.Direction.OUTPUT);
            MCP.pinMode(RSPin, MCP23017.Direction.OUTPUT);
            MCP.pinMode(EnablePin, MCP23017.Direction.OUTPUT);

            foreach (var pin in DataPins)
            {
                MCP.pinMode(pin, MCP23017.Direction.OUTPUT);
            }

            foreach (var pin in ButtonPins)
            {
                MCP.pinMode(pin, MCP23017.Direction.INPUT);
                MCP.pullUp(pin, MCP23017.Level.HIGH);
            }

            foreach (var pin in ColorPins)
            {
                MCP.pinMode(pin, MCP23017.Direction.OUTPUT);
            }
            setBacklight(WHITE);



            if (lines > 1)
            {
                DisplayFunction |= LCD_2LINE;
            }
            NumLines = lines;
            NumColumns = cols;
            CurrentLine = 0;
            CurrentCol = 0;

            // for some 1 line displays you can select a 10 pixel high font
            if ((dotsize != 0) && (lines == 1))
            {
                DisplayFunction |= LCD_5x10DOTS;
            }

            // Pull both RS and R/W low to begin commands
            MCP.digitalWrite(RSPin, MCP23017.Level.LOW);
            MCP.digitalWrite(EnablePin, MCP23017.Level.LOW);
            MCP.digitalWrite(RWPin, MCP23017.Level.LOW);

            //put the LCD into 4 bit or 8 bit mode
            if (0 == (DisplayFunction & LCD_8BITMODE))
            {
                // this is according to the hitachi HD44780 datasheet
                // figure 24, pg 46

                // we start in 8bit mode, try to set 4 bit mode
                write4bits(0x03);
                usDelay(4500);

                // second try
                write4bits(0x03);
                usDelay(4500);

                // third go!
                write4bits(0x03);
                usDelay(150);

                // finally, set to 8-bit interface
                write4bits(0x02);
            }
            else
            {
                // this is according to the hitachi HD44780 datasheet
                // page 45 figure 23

                // Send function set command sequence
                command((byte)(LCD_FUNCTIONSET | DisplayFunction));
                usDelay(4500);

                // second try
                command((byte)(LCD_FUNCTIONSET | DisplayFunction));
                usDelay(150);

                // third go
                command((byte)(LCD_FUNCTIONSET | DisplayFunction));
            }

            // finally, set # lines, font size, etc.
            command((byte)(LCD_FUNCTIONSET | DisplayFunction));

            // turn the display on with no cursor or blinking default
            DisplayControl = (byte)(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
            display();

            // clear it off
            clear();

            // Initialize to default text direction (for romance languages)
            DisplayMode = (byte)(LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
            // set the entry mode
            command((byte)(LCD_ENTRYMODESET | DisplayMode));

        }

        public void clear()
        {
            command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
            usDelay(2000);
        }

        public void home()
        {
            command(LCD_RETURNHOME);  // set cursor position to zero
            usDelay(2000);
        }

        public void setCursor(byte col, byte line)
        {
            byte[] row_offsets = { 0x00, 0x40, 0x14, 0x54 };
            if (line > NumLines)
            {
                line = (byte)(NumLines - 1);    // we count rows starting w/0
            }
            CurrentLine = line;
            CurrentCol = col;
            command((byte)(LCD_SETDDRAMADDR | (col + row_offsets[line])));
        }

        // Turn the display on/off (quickly)
        public void noDisplay()
        {
            unchecked { DisplayControl &= (byte)(~LCD_DISPLAYON); }
            command((byte)(LCDDisplayControl | DisplayControl));
        }

        public void display()
        {
            DisplayControl |= LCD_DISPLAYON;
            command((byte)(LCDDisplayControl | DisplayControl));
        }

        // Turns the underline cursor on/off
        public void noCursor()
        {
            unchecked { DisplayControl &= (byte)~LCD_CURSORON; }
            command((byte)(LCDDisplayControl | DisplayControl));
        }

        public void cursor()
        {
            DisplayControl |= LCD_CURSORON;
            command((byte)(LCDDisplayControl | DisplayControl));
        }

        // Turn on and off the blinking cursor
        public void noBlink()
        {
            unchecked { DisplayControl &= (byte)~LCD_BLINKON; }
            command((byte)(LCDDisplayControl | DisplayControl));
        }

        public void blink()
        {
            DisplayControl |= LCD_BLINKON;
            command((byte)(LCDDisplayControl | DisplayControl));
        }

        // These commands scroll the display without changing the RAM
        public void scrollDisplayLeft()
        {
            command((byte)(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT));
        }

        public void scrollDisplayRight()
        {
            command((byte)(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT));
        }

        // This is for text that flows Left to Right
        public void leftToRight()
        {
            DisplayMode |= LCD_ENTRYLEFT;
            command((byte)(LCD_ENTRYMODESET | DisplayMode));
        }

        // This is for text that flows Right to Left
        public void rightToLeft()
        {
            unchecked { DisplayMode &= (byte)~LCD_ENTRYLEFT; }
            command((byte)(LCD_ENTRYMODESET | DisplayMode));
        }

        // This will 'right justify' text from the cursor
        public void autoscroll()
        {
            DisplayMode |= LCD_ENTRYSHIFTINCREMENT;
            command((byte)(LCD_ENTRYMODESET | DisplayMode));
        }

        // This will 'left justify' text from the cursor
        public void noAutoscroll()
        {
            unchecked { DisplayMode &= (byte)~LCD_ENTRYSHIFTINCREMENT; }
            command((byte)(LCD_ENTRYMODESET | DisplayMode));
        }

        // Allows us to fill the first 8 CGRAM locations
        // with custom characters
        public void createChar(byte location, byte[] charmap)
        {
            location &= 0x7; // we only have 8 locations 0-7
            command((byte)(LCD_SETCGRAMADDR | (location << 3)));
            for (int i = 0; i < 8; i++)
            {
                write(charmap[i]);
            }
            command(LCD_SETDDRAMADDR);  // unfortunately resets the location to 0,0
        }

        public byte readButtons()
        {
            byte reply = 0x1F;

            for (byte i = 0; i < 5; i++)
            {
                reply &= (byte)~((MCP.digitalRead(ButtonPins[i])) << i);
            }
            return reply;
        }

        public void setBacklight(int color)
        {
            // check if i2c or SPI
            MCP.digitalWrite(8, (MCP23017.Level)(~(color >> 2) & 0x1));
            MCP.digitalWrite(7, (MCP23017.Level)(~(color >> 1) & 0x1));
            MCP.digitalWrite(6, (MCP23017.Level)(~color & 0x1));
        }

        public void print(string str)
        {
            foreach (char c in str)
            {
                write((byte)c);
                CurrentCol++;
                if (CurrentCol == NumColumns)
                {
                    Math.Min(NumLines, CurrentLine++);
                    CurrentCol = 0;
                }
                setCursor((byte)CurrentCol,(byte) CurrentLine);
            }
        }

        public void print(int number)
        {
            print(number.ToString());
        }

        public void print(double number)
        {
            print(number.ToString());
        }

        #region  mid level commands
        public void command(byte value)
        {
            send(value, MCP23017.Level.LOW);
        }

        private void write(byte value)
        {
            send(value, MCP23017.Level.HIGH);
        }
        #endregion

        #region low level commands

        // write either command or data, with automatic 4/8-bit selection
        private void send(byte value, MCP23017.Level mode)
        {
            MCP.digitalWrite(RSPin, mode);

            MCP.digitalWrite(RWPin, MCP23017.Level.LOW);

            write4bits((byte)(value >> 4));
            write4bits(value);
        }

        private void write4bits(byte value)
        {
            UInt16 registerValue;

            registerValue = MCP.readGPIOAB();

            // speed up for i2c since its sluggish
            for (int i = 0; i < 4; i++)
            {
                registerValue &= (UInt16)~(1 << DataPins[i]);
                registerValue |= (UInt16)(((value >> i) & 0x1) << DataPins[i]);
            }

            // make sure enable is low
            registerValue &= (UInt16)~(1 << EnablePin);

            MCP.writeGPIOAB(registerValue);

            pulseEnable();
        }

        private void pulseEnable()
        {
            MCP.digitalWrite(EnablePin, MCP23017.Level.LOW);
            usDelay(2);
            MCP.digitalWrite(EnablePin, MCP23017.Level.HIGH);
            usDelay(2);    // enable pulse must be >450ns
            MCP.digitalWrite(EnablePin, MCP23017.Level.LOW);
            usDelay(100);   // commands need > 37us to settle
        }


        #endregion

        private void usDelay(long duration)
        {
            // Static method to initialize and start stopwatch
            var sw = Stopwatch.StartNew();

            long nanosecPerTick = (1000L * 1000L * 1000L) / Stopwatch.Frequency;
            long durationTicks = (duration * 1000) / nanosecPerTick;

            while (sw.ElapsedTicks < durationTicks)
            {

            }
        }
    }
}
