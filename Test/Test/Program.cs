﻿using System;
using System.Collections;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;

namespace Test
{
    public class Program
    {
        const float intakeSpeed = 0.35f;
        const float feederSpeed = 1f;
        const float hoodSpeed = 0.4f;
        const float turretSpeed = 0.2f;
        const float driveSpeed = 0.8f;
        const float turnSpeed = 0.6f;

        const float SHOOTER_RPM_LIMIT = 3850f; // [RPM] at shooter wheels
        const float SHOOTER_kP = 10f / 1930f; // [percentVbus]/[RPM error at shooter wheels]
        const float SHOOTER_KFF = 0.5f / 1930f;
        const float SHOOTER_MARGIN = 25f;

        // Hood characteristics
        const int HOOD_LOWER_BOUND_ANALOG = 586;
        const int HOOD_UPPER_BOUND_ANALOG = 232; //232
        const int HOOD_LOWER_BOUND_ANGLE = 0; // degrees- when hood is in lowest position (ie all the way in)
        const int HOOD_UPPER_BOUND_ANGLE = 60; // degrees- when hood is all the way out - lowest angle shot

        // Turret characteristics
        const int TURRET_OFFSET = 1483;
        const int TURRET_LOWER_BOUND_ANALOG = 2048; // todo adjust these
        const int TURRET_UPPER_BOUND_ANALOG = -2048;
        const int TURRET_LOWER_BOUND_ANGLE = -20; // degrees- when turret is all the way right
        const int TURRET_UPPER_BOUND_ANGLE = 20; // degrees- when turret is all the way left

        const int kTimeoutMs = 30;
        // This is a stupid queue
        // Don't worry about it
        static bool hasNewVisionData = false;
        static float visionAngleQueue;
        static ushort visionDistQueue;
        static bool shooterAdjustLockout;
        static bool turretAdjustLockout;

        static bool isIntaking = false;

        enum EBut : uint
        {
            X = 1, A = 2, B = 3, Y = 4,
            LB = 5, RB = 6, LT = 7, RT = 8,
            SELECT = 9, START = 10,
            LJ = 11, RJ = 12, T = 13
        };

        // Create a GamePad Object
        static GameController gamepad = new GameController(new UsbHostDevice(0));

        // Initialize Talon SRX Objects
        static TalonSRX hood = new TalonSRX(7); // Hood positional
        static TalonSRX turret = new TalonSRX(6); // Turret positional
        static TalonSRX shooterSensorTalon = new TalonSRX(4); // Shooter velocity reader (also used for ring lights)
        static TalonSRX intakeLft = new TalonSRX(3);  // Intake R1 button
        static TalonSRX intakeRgt = new TalonSRX(2);  // Intake L0 button
        static TalonSRX feederL = new TalonSRX(1); // Feeder R1 button
        static TalonSRX feederR = new TalonSRX(0); // Feeder L0 button

        // Initialize Victor SPX Objects
        static VictorSPX victor3 = new VictorSPX(3); // Right 1
        static VictorSPX victor2 = new VictorSPX(2); // Right 0
        static VictorSPX victor1 = new VictorSPX(1); // Left 1
        static VictorSPX victor0 = new VictorSPX(0); // Left 0

        // UART for rpi connection
        static System.IO.Ports.SerialPort _uart;

        // PWM for VESC shooter motor
        // We only need one because the other follows over CAN
        // Andrew- add SPOT.Hardware.PWM to your references
        static PWMSpeedController shooterVESC;
        static float shooterRPMTarget = 3000f;

        // PCM for shooter speed feedback
        static PneumaticControlModule pcm = new PneumaticControlModule(0);
        static int shooterFeedbackLEDPort = 7;

        const int state = 2; // 0 = normal, 1 = skillshot mode, 2 = horse (distance) mode

        public static void Main()
        {
            Initialize();

            while (true)
            {
                if (gamepad.GetConnectionStatus() == UsbDeviceConnection.Connected)
                {
                    CTRE.Phoenix.Watchdog.Feed();

                    Drive();
                    Camera();
                    Intake();
                    Feeder();
                    Shooter();
                    AdjustThing();
                    Turret();
                    if (state == 0)
                    {
                        AcquireTarget();
                    }
                    else
                    {
                        doFlash();
                    }
                    Debug.Print("" + feetDist);
                    Debug.Print("H: " + getHoodAngle() + " T: " + getTurretAngle() +"(" + turretTarget +") SV: " + getShooterRPM() + " SVT: " + shooterRPMTarget);
                }

                Thread.Sleep(10);
            }
        }

        static void TargetLight()
        {
            if (!ledFlashActive)
            {
                if (gamepad.GetButton((uint)EBut.LT))
                {
                    shooterSensorTalon.Set(ControlMode.PercentOutput, 1.0);
                }
                else
                {
                    shooterSensorTalon.Set(ControlMode.PercentOutput, 0.0);
                }
            }
        }

        static void Drive()
        {
            float x = gamepad.GetAxis(1);
            float z = gamepad.GetAxis(2);

            DeadZone(ref x);
            DeadZone(ref z);

            z *= turnSpeed;
            x *= driveSpeed;

            float left = x + z;
            float right = x - z;

            right *= -1;

            left = left * left * left;
            right = right * right * right;

            victor0.Set(ControlMode.PercentOutput, left);
            victor2.Set(ControlMode.PercentOutput, right);
        }

        static void DeadZone(ref float val)
        {
            if (val < 0.1 && val > -0.1)
            {
                val = 0;
            }
        }

        static void Feeder()
        {
            if (isIntaking)
            {
                feederR.Set(ControlMode.PercentOutput, -0.1);
            }
            else
            {
                // Buttons are toggles
                if (gamepad.GetButton((uint)EBut.RB))
                {
                    feederR.Set(ControlMode.PercentOutput, feederSpeed);
                }
                else
                {
                    feederR.Set(ControlMode.PercentOutput, 0);
                }
            }
        }

        static void Intake()
        {
            // Buttons are toggles
            if (gamepad.GetButton((uint)EBut.LB))
            {
                intakeRgt.Set(ControlMode.PercentOutput, intakeSpeed);
                setTurretAngle(0);
                isIntaking = true;
            }
            else
            {
                intakeRgt.Set(ControlMode.PercentOutput, 0);
                isIntaking = false;
            }
        }

        static void Shooter()
        {
            AdjustShooterSpeed();

            float appVoltage = 0f;
            float shooterRPM = getShooterRPM();
            if (gamepad.GetButton((uint)EBut.RT))
            {
                shooterRPMTarget = (float)System.Math.Min(shooterRPMTarget, SHOOTER_RPM_LIMIT);
                appVoltage = shooterRPMTarget * SHOOTER_KFF + (shooterRPMTarget - shooterRPM) * SHOOTER_kP;
                appVoltage = (float)System.Math.Min(System.Math.Max(0.0f, appVoltage), 1f);
                shooterVESC.Set(appVoltage);
                shooterVESC.Enable();

                if (System.Math.Abs(shooterRPMTarget - shooterRPM) < SHOOTER_MARGIN)
                {
                    pcm.SetSolenoidOutput(shooterFeedbackLEDPort, true);
                }
                else
                {
                    pcm.SetSolenoidOutput(shooterFeedbackLEDPort, false);
                }
            }
            else
            {
                pcm.SetSolenoidOutput(shooterFeedbackLEDPort, false);
                shooterVESC.Disable();
            }
        }

        static float hoodSetpoint = 45f;
        static bool adjThingLockout = false;
        static int skillPose = 0;
        static int feetDist = 0;
        static void AdjustThing()
        {
            // Buttons are toggles
            if (gamepad.GetButton((uint)EBut.A))
            {
                if (!adjThingLockout)
                {
                    if (state == 0)
                    {
                        hoodSetpoint += 0.5f;
                    }
                    else if (state == 1)
                    {
                        // Skill setpoint
                        skillPose++;
                        if (skillPose > 5)
                        {
                            skillPose = 5;
                        }
                        setSkillshot(skillPose);
                        flashCount(skillPose);
                    }
                    else if (state == 2)
                    {
                        feetDist++;
                        flashCount(feetDist);
                        setHoodAngleAndSpeed(feetDist * 12f);
                    }
                    adjThingLockout = true;
                }
            }
            else if (gamepad.GetButton((uint)EBut.Y))
            {
                if (!adjThingLockout)
                {
                    if (state == 0)
                    {
                        hoodSetpoint -= 0.5f;
                    }
                    else if (state == 1)
                    {
                        // Skill setpoint
                        skillPose--;
                        if (skillPose < 0)
                        {
                            skillPose = 0;
                        }
                        setSkillshot(skillPose);
                        flashCount(skillPose);
                    }
                    else if (state == 2)
                    {
                        feetDist--;
                        if (feetDist < 0)
                        {
                            feetDist = 0;
                        }
                        flashCount(feetDist);
                        setHoodAngleAndSpeed(feetDist * 12f);
                    }
                    adjThingLockout = true;
                }
            }
            else
            {
                adjThingLockout = false;
            }
            setHoodAngle(hoodSetpoint);
        }

        static void setSkillshot(int skill)
        {
            switch(skill)
            {
                case 1:
                    hoodSetpoint = 7.8f;
                    shooterRPMTarget = 2750f;
                    break;
                case 2:
                    hoodSetpoint = 19.3f;
                    shooterRPMTarget = 3050f;
                    break;
                case 3:
                    hoodSetpoint = 28.5f;
                    shooterRPMTarget = 3225f;
                    break;
                case 4:
                    hoodSetpoint = 34.4f;
                    shooterRPMTarget = 3550f;
                    break;
                case 5:
                    hoodSetpoint = 34.4f;
                    shooterRPMTarget = 3550f;
                    break;
            }
        }

        static void flashCount(int count)
        {
            ledFlashCounts = count;
            ledFlashActive = true;
        }

        static bool ledFlashActive = false;
        static int ledFlashCounts = 0;
        static long flashTime = getMS();
        static bool ledOn = false;
        static bool isLongFlash = false;
        static void doFlash()
        {
            if (ledFlashActive)
            {
                long time_ = getMS();
                long delay = 100;
                if (isLongFlash)
                {
                    delay = 500;
                }
                if (time_ - flashTime > delay)
                {
                    flashTime = time_;
                    if (ledOn)
                    {
                        ledOn = false;
                        shooterSensorTalon.Set(ControlMode.PercentOutput, 0.0);
                        if (ledFlashCounts <= 0)
                        {
                            ledFlashActive = false;
                        }
                    }
                    else
                    {
                        ledOn = true;
                        shooterSensorTalon.Set(ControlMode.PercentOutput, 1.0);
                        isLongFlash = ledFlashCounts >= 5;
                        if (isLongFlash)
                        {
                            ledFlashCounts -= 5;
                        }
                        else
                        {
                            ledFlashCounts--;
                        }
                    }
                }
            }
        }

        static float turretTarget = 0;
        static void Turret()
        {
            const float adj = 0.5f;
            if (gamepad.GetButton((uint)EBut.X))
            {

                if (!turretAdjustLockout)
                {
                    turretTarget -= adj;
                    turretAdjustLockout = true;
                }
            }
            else if (gamepad.GetButton((uint)EBut.B))
            {
                if (!turretAdjustLockout)
                {
                    turretTarget += adj;
                    turretAdjustLockout = true;
                }
            }
            else
            {
                turretAdjustLockout = false;
            }

            if (!isIntaking)
            {
                setTurretAngle(turretTarget);
            }
        }

        static int acqTargetState = 0;
        static float angleFilterSum = 0;
        static float distanceFilterSum = 0;
        static int filterCounter = 0;
        static long timeStart;
        static void AcquireTarget()
        {
            if (gamepad.GetButton((uint)EBut.LT))
            {
                if (acqTargetState == 0)
                {
                    angleFilterSum = 0;
                    distanceFilterSum = 0;
                    filterCounter = 0;
                    timeStart = getMS();
                    acqTargetState = 1;
                    shooterSensorTalon.Set(ControlMode.PercentOutput, 1.0);
                }
                else if (acqTargetState == 1 && getMS() - timeStart > 50)
                {
                    acqTargetState = 2;
                }
                else if (acqTargetState == 2)
                {
                    if (hasNewVisionData)
                    {
                        filterCounter++;
                        angleFilterSum += visionAngleQueue;
                        distanceFilterSum += (float)visionDistQueue;
                        hasNewVisionData = false;
                    }
                    if (getMS() - timeStart > 1000)
                    {
                        if (filterCounter == 0)
                        {
                            acqTargetState = 10;
                        }
                        else
                        {
                            acqTargetState = 3;
                            turretTarget = getTurretAngle() + angleFilterSum / filterCounter;
                            float actualDist = distanceFilterSum / filterCounter;

                            // Apply model
                            setHoodAngleAndSpeed(actualDist);
                        }
                    }
                }
                else if (acqTargetState == 3 && getMS() - timeStart > 1300)
                {
                    acqTargetState = 4;
                    shooterSensorTalon.Set(ControlMode.PercentOutput, 0.0);
                }
                else if (acqTargetState == 4 && getMS() - timeStart > 1400)
                {
                    acqTargetState = 5;
                    shooterSensorTalon.Set(ControlMode.PercentOutput, 1.0);
                }
                else if (acqTargetState == 5 && getMS() - timeStart > 1500)
                {
                    shooterSensorTalon.Set(ControlMode.PercentOutput, 0.0);
                    acqTargetState = 6; // Done state
                }
                else if (acqTargetState >= 10 && acqTargetState <= 20)
                {
                    // State >= 10 is when we don't see the target and need to indicate that
                    shooterSensorTalon.Set(ControlMode.PercentOutput, acqTargetState %2 == 0 ? 1.0 : 0.0);
                    if (getMS() - timeStart > 50)
                    {
                        acqTargetState++;
                        timeStart = 0;
                    }

                }
            }
            else
            {
                acqTargetState = 0;
                shooterSensorTalon.Set(ControlMode.PercentOutput, 0.0);
            }
        }

        static void AdjustShooterSpeed()
        {
            const float adj = 25;
            if (gamepad.GetButton((uint)EBut.SELECT))
            {
                if (!shooterAdjustLockout)
                {
                    shooterRPMTarget += adj;
                    shooterAdjustLockout = true;
                }
            }
            else if (gamepad.GetButton((uint)EBut.START))
            {
                if (!shooterAdjustLockout)
                {
                    shooterRPMTarget -= adj;
                    shooterAdjustLockout = true;
                }
            }
            else
            {
                shooterAdjustLockout = false;
            }
        }

        static void Initialize()
        {
            // Serial port
            _uart = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port1.UART, 115200);
            _uart.Open();

            // Victor SPX Slaves
            // Left Slave
            victor1.Set(ControlMode.Follower, 0);
            // Right Slave
            victor3.Set(ControlMode.Follower, 2);

            // Talon SRX Slaves
            // Feeder Slave
            feederL.Set(ControlMode.Follower, 0);
            feederL.SetInverted(true);
            // Intake Slave
            intakeLft.Set(ControlMode.Follower, 2);
            intakeLft.SetInverted(true);

            // Hood
            hood.ConfigSelectedFeedbackSensor(FeedbackDevice.Analog, 0, kTimeoutMs);
            hood.SetSensorPhase(false);
            hood.Config_kP(0, 30f, kTimeoutMs); /* tweak this first, a little bit of overshoot is okay */
            hood.Config_kI(0, 0.0005f, kTimeoutMs);
            hood.Config_kD(0, 0f, kTimeoutMs);
            hood.Config_kF(0, 0f, kTimeoutMs);
            /* use slot0 for closed-looping */
            hood.SelectProfileSlot(0, 0);

            /* set the peak and nominal outputs, 1.0 means full */
            hood.ConfigNominalOutputForward(0.0f, kTimeoutMs);
            hood.ConfigNominalOutputReverse(0.0f, kTimeoutMs);
            hood.ConfigPeakOutputForward(+1.0f, kTimeoutMs);
            hood.ConfigPeakOutputReverse(-1.0f, kTimeoutMs);
            hood.ConfigForwardSoftLimitThreshold(HOOD_LOWER_BOUND_ANALOG, kTimeoutMs);
            hood.ConfigReverseSoftLimitThreshold(HOOD_UPPER_BOUND_ANALOG, kTimeoutMs);
            hood.ConfigForwardSoftLimitEnable(true, kTimeoutMs);
            hood.ConfigReverseSoftLimitEnable(true, kTimeoutMs);

            /* how much error is allowed?  This defaults to 0. */
            hood.ConfigAllowableClosedloopError(0, 5, kTimeoutMs);

            //***********************
            // MAY NEED TUNING
            //***********************
            // Turret
            turret.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, kTimeoutMs);
            int absPos = turret.GetSelectedSensorPosition(1);
            turret.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
            turret.SetSelectedSensorPosition(0, 0, kTimeoutMs);
            turret.SetSensorPhase(true);
            turret.Config_IntegralZone(20);
            turret.Config_kP(0, 2f, kTimeoutMs); // tweak this first, a little bit of overshoot is okay
            turret.Config_kI(0, 0f, kTimeoutMs);
            turret.Config_kD(0, 0f, kTimeoutMs);
            turret.Config_kF(0, 0f, kTimeoutMs);
            // use slot0 for closed-looping
            turret.SelectProfileSlot(0, 0);

            // set the peak and nominal outputs, 1.0 means full
            turret.ConfigNominalOutputForward(0.0f, kTimeoutMs);
            turret.ConfigNominalOutputReverse(0.0f, kTimeoutMs);
            turret.ConfigPeakOutputForward(+0.5f, kTimeoutMs);
            turret.ConfigPeakOutputReverse(-0.5f, kTimeoutMs);
            turret.ConfigReverseSoftLimitThreshold(TURRET_UPPER_BOUND_ANALOG, kTimeoutMs);
            turret.ConfigForwardSoftLimitThreshold(TURRET_LOWER_BOUND_ANALOG, kTimeoutMs);
            turret.ConfigReverseSoftLimitEnable(true, kTimeoutMs);
            turret.ConfigForwardSoftLimitEnable(true, kTimeoutMs);

            // how much error is allowed?  This defaults to 0.
            turret.ConfigAllowableClosedloopError(0, 5, kTimeoutMs);

            // Shooter
            shooterSensorTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
            shooterSensorTalon.SetSensorPhase(false);
            shooterSensorTalon.ConfigPeakOutputReverse(0.0f, kTimeoutMs);
            shooterSensorTalon.ConfigPeakOutputForward(1.0f, kTimeoutMs);

            shooterVESC = new PWMSpeedController(CTRE.HERO.IO.Port3.PWM_Pin7);
            shooterVESC.Set(0);

            pcm.SetSolenoidOutput(shooterFeedbackLEDPort, false);
        }

        static void Camera()
        {
            // Each packet is 8 bytes
            if (_uart.BytesToRead >= 8)
            {
                byte[] _rx = new byte[24];
                
                int readCnt = _uart.Read(_rx, 0, (int)System.Math.Min(_uart.BytesToRead, 24));
                int zeroCount = 0;

                // Each packet is four null bytes followed by the data
                // To make sure we have a full packet (and align ourselves) read until we see four zeros,
                // and then read the next four as the packet
                for (int i = 0; i < readCnt; i++)
                {
                    // Count zeros
                    if (_rx[i] == 0)
                    {
                        zeroCount++;
                    }
                    else
                    {
                        zeroCount = 0;
                    }
                    // We have a packet
                    if (zeroCount == 4)
                    {
                        int packetOfs = i + 1;
                        if (readCnt - packetOfs >= 4)
                        {
                            short angle = BitConverter.ToInt16(_rx, packetOfs);
                            ushort distance = BitConverter.ToUInt16(_rx, packetOfs + 2);
                            float rangle = ((float)angle) / 10f; // Angle is multiplied by 10 on pi to be sent over wire
                            if (System.Math.Abs(rangle) <= 30 && distance < 400)
                            {
                                hasNewVisionData = true;
                                visionAngleQueue = rangle;
                                visionDistQueue = distance;
                            }

                            Debug.Print("Camera says: " + distance + " in @ " + rangle + "deg");
                        }
                        break;
                    }
                }
            }
        }

        // Takes an input X on the range AB and relates it to an output in the range CD
        // That is, if A is passed in as X, this function returns C. If B is passed in, this function returns D
        // Everywhere in between is linear
        static float linRelate(float x, float a, float b, float c, float d)
        {
            float scale = (x - a) / (b - a);
            return scale * (d - c) + c;
        }

        static float getShooterRPM()
        {
            return shooterSensorTalon.GetSelectedSensorVelocity() / (10f*4096f) * 6000f;
        }

        static float getHoodAngle()
        {
            int sensorPos = hood.GetSelectedSensorPosition();
            return linRelate(sensorPos, HOOD_LOWER_BOUND_ANALOG, HOOD_UPPER_BOUND_ANALOG,
                             HOOD_LOWER_BOUND_ANGLE, HOOD_UPPER_BOUND_ANGLE);
        }

        static float getTurretAngle()
        {
            int sensorPos = turret.GetSelectedSensorPosition();
            return linRelate(sensorPos, TURRET_LOWER_BOUND_ANALOG, TURRET_UPPER_BOUND_ANALOG,
                             TURRET_LOWER_BOUND_ANGLE, TURRET_UPPER_BOUND_ANGLE);
        }

        static void setHoodAngle(float angle)
        {
            int sensorPos = (int)linRelate(angle, HOOD_LOWER_BOUND_ANGLE, HOOD_UPPER_BOUND_ANGLE,
                                            HOOD_LOWER_BOUND_ANALOG, HOOD_UPPER_BOUND_ANALOG);
            hood.Set(ControlMode.Position, sensorPos);
        }

        static void setTurretAngle(float angle)
        {
            if (angle > TURRET_UPPER_BOUND_ANGLE)
            {
                angle = TURRET_UPPER_BOUND_ANGLE;
            }
            else if (angle < TURRET_LOWER_BOUND_ANGLE)
            {
                angle = TURRET_LOWER_BOUND_ANGLE;
            }
            int sensorPos = (int)linRelate(angle, TURRET_LOWER_BOUND_ANGLE, TURRET_UPPER_BOUND_ANGLE,
                                            TURRET_LOWER_BOUND_ANALOG, TURRET_UPPER_BOUND_ANALOG);
            turret.Set(ControlMode.Position, sensorPos);
        }

        static long getMS()
        {
            return DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        }

        static void setHoodAngleAndSpeed(float distanceIn)
        {
            float robotDist = distanceIn;
            float x = robotDist;

            double hoodAngle = 7.091e-8 * System.Math.Pow(x, 4) - 3.842e-05 * System.Math.Pow(x, 3) + 0.006269 * System.Math.Pow(x, 2) - 0.1573 * x + 7.459;
            double y = hoodAngle;
            double targetSpeed = 2932 + 10.89 * x - 57 * y - 0.008035 * System.Math.Pow(x, 2) - 0.0891 * x * y + 0.9558 * System.Math.Pow(y, 2);
            hoodSetpoint = (float)hoodAngle;
            shooterRPMTarget = (float)targetSpeed;
        }
    }
}
