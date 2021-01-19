/*------------------------------------------------------------------------
  Adafruit Class Library for Windows Core IoT: Adafruit Motor Hat.

  Written by Rick Lesniak for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------*/

using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace AdafruitClassLibrary
{
    public class MotorHat : Pca9685
    {
        #region Subordinate Classes

        public class DCMotor
        {
            public enum Command { FORWARD, BACKWARD, RELEASE };

            private MotorHat Hat { get; set; }
            private int Index { get; set; }
            private int PWMPin { get; set; }
            private int In1Pin { get; set; }
            private int In2Pin { get; set; }

            public DCMotor(MotorHat hat, int index)
            {
                Hat = hat;
                Index = index;
                switch (index)
                {
                    case 0:
                        PWMPin = 8;
                        In2Pin = 9;
                        In1Pin = 10;
                        break;

                    case 1:
                        PWMPin = 13;
                        In2Pin = 12;
                        In1Pin = 11;
                        break;

                    case 2:
                        PWMPin = 2;
                        In2Pin = 3;
                        In1Pin = 4;
                        break;

                    case 3:
                        PWMPin = 7;
                        In2Pin = 6;
                        In1Pin = 5;
                        break;

                    default:
                        break;
                }
            }

            public void Run(Command cmd)
            {
                switch (cmd)
                {
                    case Command.FORWARD:
                        Hat.SetPin(In2Pin, PinState.LOW);
                        Hat.SetPin(In1Pin, PinState.HIGH);
                        break;

                    case Command.BACKWARD:
                        Hat.SetPin(In1Pin, PinState.LOW);
                        Hat.SetPin(In2Pin, PinState.HIGH);
                        break;

                    case Command.RELEASE:
                        Hat.SetPin(In1Pin, PinState.LOW);
                        Hat.SetPin(In2Pin, PinState.LOW);
                        break;

                    default:
                        break;
                }
            }

            public void SetSpeed(uint speed)
            {
                Hat.SetPWM(PWMPin, (ushort)(speed * 16));
            }
        }

        public class Stepper
        {
            public enum Command { FORWARD, BACKWARD };

            public enum Style { SINGLE, DOUBLE, INTERLEAVE, MICROSTEP };

            private MotorHat Hat { get; set; }
            private int Index { get; set; }
            private int PWMAPin { get; set; }
            private int AIn1Pin { get; set; }
            private int AIn2Pin { get; set; }
            private int PWMBPin { get; set; }
            private int BIn1Pin { get; set; }
            private int BIn2Pin { get; set; }

            private ushort RevSteps { get; set; }  //steps per revolution
            private ushort CurrentStep { get; set; }
            private uint MicrosecondsPerStep { get; set; }

            private const int MICROSTEPS = 16;         // 8 or 16

            private List<int> MicrostepCurve;

            public Stepper(MotorHat hat, ushort steps, int index)
            {
                Hat = hat;
                Index = index;
                RevSteps = steps;

                if (8 == MICROSTEPS)
                    MicrostepCurve = new List<int>() { 0, 50, 98, 142, 180, 212, 236, 250, 255 };
                else
                    MicrostepCurve = new List<int>() { 0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255 };

                switch (index)
                {
                    case 0:
                        PWMAPin = 8;
                        AIn2Pin = 9;
                        AIn1Pin = 10;
                        PWMBPin = 13;
                        BIn2Pin = 12;
                        BIn1Pin = 11;
                        break;

                    case 1:
                        PWMAPin = 2;
                        AIn2Pin = 3;
                        AIn1Pin = 4;
                        PWMBPin = 7;
                        BIn2Pin = 6;
                        BIn1Pin = 5;
                        break;

                    default:
                        break;
                }
            }

            public void SetSpeed(uint rpm)
            {
                MicrosecondsPerStep = 60000000 / (RevSteps * rpm);
            }

            public void Release()
            {
                Hat.SetPin(AIn1Pin, PinState.LOW);
                Hat.SetPin(AIn2Pin, PinState.LOW);
                Hat.SetPin(BIn1Pin, PinState.LOW);
                Hat.SetPin(BIn2Pin, PinState.LOW);
                Hat.SetPWM(PWMAPin, 0);
                Hat.SetPWM(PWMBPin, 0);
            }

            public void step(ushort steps, Command direction, Style style)
            {
                uint usPerStep = MicrosecondsPerStep;
                int ret;

                if (style == Style.INTERLEAVE)
                {
                    usPerStep /= 2;
                }
                else if (style == Style.MICROSTEP)
                {
                    usPerStep /= MICROSTEPS;
                    steps *= MICROSTEPS;
                }

                while (0 != steps--)
                {
                    ret = OneStep(direction, style);
                    Hat.usDelay(usPerStep);
                }
            }

            public int OneStep(Command direction, Style style)
            {
                int ocrb, ocra;

                ocra = ocrb = 255;

                if (style == Style.SINGLE)
                {
                    if (1 == ((CurrentStep / (MICROSTEPS / 2)) % 2))
                    { // we're at an odd step, weird
                        if (direction == Command.FORWARD)
                        {
                            CurrentStep += MICROSTEPS / 2;
                        }
                        else
                        {
                            CurrentStep -= MICROSTEPS / 2;
                        }
                    }
                    else
                    {           // go to the next even step
                        if (direction == Command.FORWARD)
                        {
                            CurrentStep += MICROSTEPS;
                        }
                        else
                        {
                            CurrentStep -= MICROSTEPS;
                        }
                    }
                }
                else if (style == Style.DOUBLE)
                {
                    if (0 == (CurrentStep / (MICROSTEPS / 2) % 2))
                    { // we're at an even step, weird
                        if (direction == Command.FORWARD)
                        {
                            CurrentStep += MICROSTEPS / 2;
                        }
                        else
                        {
                            CurrentStep -= MICROSTEPS / 2;
                        }
                    }
                    else
                    {           // go to the next odd step
                        if (direction == Command.FORWARD)
                        {
                            CurrentStep += MICROSTEPS;
                        }
                        else
                        {
                            CurrentStep -= MICROSTEPS;
                        }
                    }
                }
                else if (style == Style.INTERLEAVE)
                {
                    if (direction == Command.FORWARD)
                    {
                        CurrentStep += MICROSTEPS / 2;
                    }
                    else
                    {
                        CurrentStep -= MICROSTEPS / 2;
                    }
                }

                if (style == Style.MICROSTEP)
                {
                    if (direction == Command.FORWARD)
                    {
                        CurrentStep++;
                    }
                    else
                    {
                        // BACKWARDS
                        CurrentStep--;
                    }

                    CurrentStep += MICROSTEPS * 4;
                    CurrentStep %= MICROSTEPS * 4;

                    ocra = ocrb = 0;
                    if ((CurrentStep >= 0) && (CurrentStep < MICROSTEPS))
                    {
                        ocra = MicrostepCurve[MICROSTEPS - CurrentStep];
                        ocrb = MicrostepCurve[CurrentStep];
                    }
                    else if ((CurrentStep >= MICROSTEPS) && (CurrentStep < MICROSTEPS * 2))
                    {
                        ocra = MicrostepCurve[CurrentStep - MICROSTEPS];
                        ocrb = MicrostepCurve[MICROSTEPS * 2 - CurrentStep];
                    }
                    else if ((CurrentStep >= MICROSTEPS * 2) && (CurrentStep < MICROSTEPS * 3))
                    {
                        ocra = MicrostepCurve[MICROSTEPS * 3 - CurrentStep];
                        ocrb = MicrostepCurve[CurrentStep - MICROSTEPS * 2];
                    }
                    else if ((CurrentStep >= MICROSTEPS * 3) && (CurrentStep < MICROSTEPS * 4))
                    {
                        ocra = MicrostepCurve[CurrentStep - MICROSTEPS * 3];
                        ocrb = MicrostepCurve[MICROSTEPS * 4 - CurrentStep];
                    }
                }

                CurrentStep += MICROSTEPS * 4;
                CurrentStep %= MICROSTEPS * 4;

                Hat.SetPWM(PWMAPin, (ushort)(ocra * 16));
                Hat.SetPWM(PWMBPin, (ushort)(ocrb * 16));

                uint latch_state = 0; // all motor pins to 0

                if (style == Style.MICROSTEP)
                {
                    if ((CurrentStep >= 0) && (CurrentStep < MICROSTEPS))
                        latch_state |= 0x03;
                    if ((CurrentStep >= MICROSTEPS) && (CurrentStep < MICROSTEPS * 2))
                        latch_state |= 0x06;
                    if ((CurrentStep >= MICROSTEPS * 2) && (CurrentStep < MICROSTEPS * 3))
                        latch_state |= 0x0C;
                    if ((CurrentStep >= MICROSTEPS * 3) && (CurrentStep < MICROSTEPS * 4))
                        latch_state |= 0x09;
                }
                else
                {
                    switch (CurrentStep / (MICROSTEPS / 2))
                    {
                        case 0:
                            latch_state |= 0x1; // energize coil 1 only
                            break;

                        case 1:
                            latch_state |= 0x3; // energize coil 1+2
                            break;

                        case 2:
                            latch_state |= 0x2; // energize coil 2 only
                            break;

                        case 3:
                            latch_state |= 0x6; // energize coil 2+3
                            break;

                        case 4:
                            latch_state |= 0x4; // energize coil 3 only
                            break;

                        case 5:
                            latch_state |= 0xC; // energize coil 3+4
                            break;

                        case 6:
                            latch_state |= 0x8; // energize coil 4 only
                            break;

                        case 7:
                            latch_state |= 0x9; // energize coil 1+4
                            break;
                    }
                }

                if (0x01 == (latch_state & 0x1))
                {
                    Hat.SetPin(AIn2Pin, PinState.HIGH);
                }
                else
                {
                    Hat.SetPin(AIn2Pin, PinState.LOW);
                }
                if (0x02 == (latch_state & 0x2))
                {
                    Hat.SetPin(BIn1Pin, PinState.HIGH);
                }
                else
                {
                    Hat.SetPin(BIn1Pin, PinState.LOW);
                }
                if (0x04 == (latch_state & 0x4))
                {
                    Hat.SetPin(AIn1Pin, PinState.HIGH);
                }
                else
                {
                    Hat.SetPin(AIn1Pin, PinState.LOW);
                }
                if (0x08 == (latch_state & 0x8))
                {
                    Hat.SetPin(BIn2Pin, PinState.HIGH);
                }
                else
                {
                    Hat.SetPin(BIn2Pin, PinState.LOW);
                }

                return CurrentStep;
            }

            public void usDelay(long duration)
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

        #endregion Subordinate Classes

        #region Main class

        public enum PinState { LOW, HIGH };

        private List<DCMotor> MotorList;
        private List<Stepper> StepperList;

        public MotorHat(int i2cAddr = 0x60) : base(i2cAddr)
        {
            MotorList = new List<DCMotor>() { null, null, null, null };
            StepperList = new List<Stepper>() { null, null };
        }

        public async Task InitAsync(uint freq)
        {
            await InitPCA9685Async();
            SetPWMFrequency(freq);
            SetAllPWM(0, 0);
        }

        public void SetPWM(int pin, ushort value)
        {
            if (value > 4096)
            {
                SetPWM(pin, 4096, 0);
            }
            else
            {
                SetPWM(pin, 0, value);
            }
        }

        public void SetPin(int pin, PinState state)
        {
            if (state == PinState.LOW)
            {
                SetPWM(pin, 0, 0);
            }
            else
            {
                SetPWM(pin, 4096, 0);
            }
        }

        public DCMotor GetMotor(int index)
        {
            if (index > 4)
                return null;
            index--;

            if (null == MotorList[index])
            {
                MotorList[index] = new DCMotor(this, index);
            }

            return MotorList[index];
        }

        public Stepper GetStepper(ushort steps, int index)
        {
            if (index > 2)
                return null;
            index--;

            if (null == StepperList[index])
            {
                StepperList[index] = new Stepper(this, steps, index);
            }

            return StepperList[index];
        }

        #endregion Main class
    }
}