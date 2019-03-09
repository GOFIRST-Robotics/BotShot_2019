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
    
    const float SHOOTER_RPM_LIMIT = 6000f; // [RPM] at shooter wheels
    const float SHOOTER_kP = 48f / 7500f; // [V]/[RPM error at shooter wheels]

    // Shooter characteristics
    const float SHOOTER_MOTOR_KV = 125; // [RPM]/[V]

    // Hood characteristics
    const int HOOD_LOWER_BOUND_ANALOG = 0; // todo adjust these
    const int HOOD_UPPER_BOUND_ANALOG = 1023;
    const int HOOD_LOWER_BOUND_ANGLE = 90; // degrees- when hood is in lowest position (ie all the way in)
    const int HOOD_UPPER_BOUND_ANGLE = 0; // degrees- when hood is all the way out - lowest angle shot

    // Turret characteristics
    const int TURRET_LOWER_BOUND_ANALOG = 570; // todo adjust these
    const int TURRET_UPPER_BOUND_ANALOG = 700;
    const int TURRET_LOWER_BOUND_ANGLE = -26; // degrees- when turret is all the way right
    const int TURRET_UPPER_BOUND_ANGLE = 17; // degrees- when turret is all the way left

    const int kTimeoutMs = 30;
    static float filteredBasketAngle;
    static short filteredBasketDistance;
    static bool shooterAdjustLockout;

    static float hoodTargetAngle;
    static float turretTargetAngle;

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
    static TalonSRX shooterSensorTalon = new TalonSRX(4); // Shooter velocity reader
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
    static float shooterRPMTarget = 0;

    public static void Main()
    {
      Initialize();

      while (true)
      {
        if (gamepad.GetConnectionStatus() == UsbDeviceConnection.Connected)
        {
          CTRE.Phoenix.Watchdog.Feed();

          //Drive();
          //Camera();
          //Intake();
          //Feeder();
          //Shooter();
          Hood();
          Turret();
        }

        Thread.Sleep(10);
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
      // Buttons are toggles
      if (gamepad.GetButton((uint)EBut.RB))
      {
        feederR.Set(ControlMode.PercentOutput, feederSpeed);
      }
      else if (gamepad.GetButton(11))
      {
        feederR.Set(ControlMode.PercentOutput, -1 * feederSpeed);
      }
      else
      {
        feederR.Set(ControlMode.PercentOutput, 0);
      }
    }

    static void Intake()
    {
      // Buttons are toggles
      if (gamepad.GetButton((uint)EBut.LB))
      {
        intakeRgt.Set(ControlMode.PercentOutput, intakeSpeed);
      }
      else if (gamepad.GetButton(11))
      {
        intakeRgt.Set(ControlMode.PercentOutput, -1 * intakeSpeed);
      }
      else
      {
        intakeRgt.Set(ControlMode.PercentOutput, 0);
      }
    }

    static void Shooter()
    {
      AdjustShooterSpeed();

      float appVoltage = 0f;
      float shooterRPM = getShooterRPM();
      if (gamepad.GetButton((uint)EBut.RT))
      {
        shooterRPMTarget = (float) System.Math.Min(shooterRPMTarget, SHOOTER_RPM_LIMIT);
        appVoltage = shooterRPMTarget / SHOOTER_MOTOR_KV + (shooterRPMTarget - shooterRPM) * SHOOTER_kP;
      }
      else
      {
        appVoltage = 0;
      }
      // Voltage compensation
      shooterVESC.Set(appVoltage / 48f);
      
      Debug.Print("H: " + hood.GetSelectedSensorPosition(0).ToString() + " V: " + shooterRPM +
                    " T: " + shooterRPMTarget);
    }

    static void Hood()
    {
      // Buttons are toggles
      if (gamepad.GetButton((uint)EBut.A))
      {
        if (hood.GetSelectedSensorPosition(0) > 350)
        {
          hood.Set(ControlMode.Position, 350);
        }
        else
        {
          hood.Set(ControlMode.PercentOutput, hoodSpeed);
        }
      }
      else if (gamepad.GetButton((uint)EBut.Y))
      {
        if (hood.GetSelectedSensorPosition(0) < 50)
        {
          hood.Set(ControlMode.Position, 50);
        }
        else
        {
          hood.Set(ControlMode.PercentOutput, -1 * hoodSpeed);
        }
      }
      else
      {
        hood.Set(ControlMode.PercentOutput, 0);
      }
    }

    static int turretTarget = -472;
    static void Turret()
    {
      if (turret.GetSelectedSensorPosition(0) > -370)
      {
        turret.Set(ControlMode.Position, -370);
      }

      if (turret.GetSelectedSensorPosition(0) < -495)
      {
        turret.Set(ControlMode.Position, -495);
      }

      // Buttons are toggles
      if (gamepad.GetButton((uint)EBut.X))
      {
        turretTarget -= 1;
      }
      else if (gamepad.GetButton((uint)EBut.B))
      {
        turretTarget += 1;
      }
      else
      {
        turretTarget += 0;
        //turret.Set(ControlMode.PercentOutput, 0);
      }

      turret.Set(ControlMode.Position, turretTarget);

      //  turret.Set(ControlMode.Position, turretTarget);
      //}
      Debug.Print(turretTarget.ToString());
      //Debug.Print("Turret angle: " + getTurretAngle().ToString());
      Debug.Print("Turret Pos:" + turret.GetSelectedSensorPosition(0).ToString());
    }

    static void AdjustShooterSpeed()
    {
      if (gamepad.GetButton((uint)EBut.SELECT))
      {
        if (!shooterAdjustLockout)
        {
          Debug.Print("Increasing shooter target...");
          shooterRPMTarget += 100;
          shooterAdjustLockout = true;
        }
      }
      else if (gamepad.GetButton((uint)EBut.START))
      {
        if (!shooterAdjustLockout)
        {
          Debug.Print("Decreasing shooter target...");
          shooterRPMTarget -= 100;
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

      /* how much error is allowed?  This defaults to 0. */
      hood.ConfigAllowableClosedloopError(0, 0, kTimeoutMs);

      //hood.ConfigForwardSoftLimitThreshold(, kTimeoutMs);
      //hood.ConfigReverseSoftLimitThreshold(, kTimeoutMs);
      //hood.ConfigReverseSoftLimitEnable(true, kTimeoutMs);
      //hood.ConfigForwardSoftLimitEnable(true, kTimeoutMs);

      //***********************
      // MAY NEED TUNING
      //***********************
      // Turret
      turret.ConfigSelectedFeedbackSensor(FeedbackDevice.Analog, 0, kTimeoutMs);
      turret.SetSensorPhase(true);
      turret.Config_IntegralZone(2);
      turret.Config_kP(0, 10f, kTimeoutMs); // tweak this first, a little bit of overshoot is okay
      turret.Config_kI(0, 0f, kTimeoutMs);
      turret.Config_kD(0, 1f, kTimeoutMs);
      turret.Config_kF(0, 0f, kTimeoutMs);
      // use slot0 for closed-looping
      turret.SelectProfileSlot(0, 0);

      // set the peak and nominal outputs, 1.0 means full
      turret.ConfigNominalOutputForward(0.0f, kTimeoutMs);
      turret.ConfigNominalOutputReverse(0.0f, kTimeoutMs);
      turret.ConfigPeakOutputForward(+0.5f, kTimeoutMs);
      turret.ConfigPeakOutputReverse(-0.5f, kTimeoutMs);

      // how much error is allowed?  This defaults to 0.
      turret.ConfigAllowableClosedloopError(1, 0, kTimeoutMs);

      // Shooter
      shooterSensorTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
      shooterSensorTalon.SetSensorPhase(true);

      shooterVESC = new PWMSpeedController(CTRE.HERO.IO.Port3.PWM_Pin7);
      shooterVESC.Set(0);
    }

    static void Camera()
    {
      // Each packet is 8 bytes
      if (_uart.BytesToRead >= 8)
      {
        byte[] _rx = new byte[24];
        int readCnt = _uart.Read(_rx, 0, _uart.BytesToRead);
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
              ushort distance = BitConverter.ToUInt16(_rx, packetOfs);
              short angle = BitConverter.ToInt16(_rx, packetOfs + 2);
              float rangle = ((float)angle) / 10f; // Angle is multiplied by 10 on pi to be sent over wire
              

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
      int talonVel = shooterSensorTalon.GetSelectedSensorVelocity(); // ticks/decisecond
      float shooterRPS = talonVel / 1023f * 10f; // 1023 ticks/rev, 10 deciseconds / second
      float shooterRPM = shooterRPS * 60;
      return shooterRPM;
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
      int sensorPos = (int) linRelate(angle, HOOD_LOWER_BOUND_ANGLE, HOOD_UPPER_BOUND_ANGLE, 
                                      HOOD_LOWER_BOUND_ANALOG, HOOD_UPPER_BOUND_ANALOG);
      hood.Set(ControlMode.Position, sensorPos);
    }

    static void setTurretAngle(float angle)
    {
      int sensorPos = (int)linRelate(angle, TURRET_LOWER_BOUND_ANGLE, TURRET_UPPER_BOUND_ANGLE,
                                      TURRET_LOWER_BOUND_ANALOG, TURRET_UPPER_BOUND_ANALOG);
      turret.Set(ControlMode.Position, sensorPos);
    }

  }
}
