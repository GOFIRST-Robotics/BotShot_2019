using System;
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
    const float intakeSpeed = 0.25f;
    const float feederSpeed = 1f;
    const float hoodSpeed = 0.4f;
    const float turretSpeed = 0.4f;

    const int kTimeoutMs = 30;
    const float kF = .0133f;

    // Create a GamePad Object
    static GameController gamepad = new GameController(new UsbHostDevice(0));

    // Initialize Talon SRX Objects
    static TalonSRX hood = new TalonSRX(7); // Hood positional
    static TalonSRX turret = new TalonSRX(6); // Turret positional
    static TalonSRX shooterSlave = new TalonSRX(5); // Shooter R1 velocity
    static TalonSRX shooter = new TalonSRX(4); // Shooter L0 velocity
    static TalonSRX intakeLft = new TalonSRX(3);  // Intake R1 button
    static TalonSRX intakeRgt = new TalonSRX(2);  // Intake L0 button
    static TalonSRX feederL = new TalonSRX(1); // Feeder R1 button
    static TalonSRX feederR = new TalonSRX(0); // Feeder L0 button

    // Initialize Victor SPX Objects
    static VictorSPX victor3 = new VictorSPX(3); // Right 1
    static VictorSPX victor2 = new VictorSPX(2); // Right 0
    static VictorSPX victor1 = new VictorSPX(1); // Left 1
    static VictorSPX victor0 = new VictorSPX(0); // Left 0

    static System.IO.Ports.SerialPort _uart;

    public static void Main()
    {
      //Initialize();

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
      if (gamepad.GetButton(6))
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
      if (gamepad.GetButton(5))
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
      if (gamepad.GetAxis(1) != 0)
      {
        float x = gamepad.GetAxis(1); // Left stick up/down
                                      //x = x * x * x * x * x;
                                      //Debug.Print(x.ToString());
        if (shooter.GetSelectedSensorVelocity(0) > 85000)
        {
          shooter.Set(ControlMode.Velocity, 85000);
        }
        else
        {
          shooter.Set(ControlMode.PercentOutput, x);
        }
        //Debug.Print("V: " + shooter.GetSelectedSensorVelocity(0).ToString());
      }
      else if (gamepad.GetButton(3))
      {
        // Never let target exceed 85000
        shooter.Set(ControlMode.Velocity, 50000);
        //Debug.Print("V: " + shooter.GetSelectedSensorVelocity(0).ToString());
      }
      else
      {
        shooter.Set(ControlMode.PercentOutput, 0);
      }

      float rpm = (shooter.GetSelectedSensorVelocity(0) * 150) / 1024;
      //Debug.Print("Rpm: " + rpm.ToString());
    }

    static void Hood()
    {
      // Buttons are toggles
      if (gamepad.GetButton(2))
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
      else if (gamepad.GetButton(4))
      {
        if(hood.GetSelectedSensorPosition(0) < 50)
        {
          hood.Set(ControlMode.Position, 50);
        }
        else
        {
          hood.Set(ControlMode.PercentOutput, -1 * hoodSpeed);
        }
      }
      else if (gamepad.GetButton(1))
      {
        Debug.Print("Trying to move to target...");
        hood.Set(ControlMode.Position, 200);
        Debug.Print(hood.GetSelectedSensorPosition(0).ToString());
      }
      else
      {
        hood.Set(ControlMode.PercentOutput, 0);
      }



      Debug.Print(hood.GetSelectedSensorPosition(0).ToString());
    }

    static void Turret()
    {
      // Buttons are toggles
      if (gamepad.GetButton(7))
      {
        turret.Set(ControlMode.PercentOutput, turretSpeed);
      }
      else if (gamepad.GetButton(8))
      {
        turret.Set(ControlMode.PercentOutput, -1 * turretSpeed);
      }
      else if (gamepad.GetButton(1))
      {
        Debug.Print("Trying to move to target...");
        //turret.Set(ControlMode.Position, 200);
        Debug.Print(turret.GetSelectedSensorPosition(0).ToString());
      }
      else
      {
        turret.Set(ControlMode.PercentOutput, 0);
      }

      //Debug.Print(turret.GetSelectedSensorPosition(0).ToString());
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
      // Shooter Slave
      shooterSlave.Set(ControlMode.Follower, 4);
      shooterSlave.SetInverted(true);

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
      turret.SetSensorPhase(false);
      turret.Config_kP(0, 30f, kTimeoutMs); // tweak this first, a little bit of overshoot is okay
      turret.Config_kI(0, 0.0005f, kTimeoutMs);
      turret.Config_kD(0, 0f, kTimeoutMs);
      turret.Config_kF(0, 0f, kTimeoutMs);
      // use slot0 for closed-looping
      turret.SelectProfileSlot(0, 0);

      // set the peak and nominal outputs, 1.0 means full
      turret.ConfigNominalOutputForward(0.0f, kTimeoutMs);
      turret.ConfigNominalOutputReverse(0.0f, kTimeoutMs);
      turret.ConfigPeakOutputForward(+1.0f, kTimeoutMs);
      turret.ConfigPeakOutputReverse(-1.0f, kTimeoutMs);

      // how much error is allowed?  This defaults to 0.
      turret.ConfigAllowableClosedloopError(0, 0, kTimeoutMs);

      // Shooter
      shooter.ConfigSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
      shooter.SetSensorPhase(true);
      shooter.SetInverted(true);
      shooter.Config_kP(0, 0.01f, kTimeoutMs); // tweak this first, a little bit of overshoot is okay
      shooter.Config_kI(0, 0f, kTimeoutMs);
      shooter.Config_kD(0, 0f, kTimeoutMs);
      shooter.Config_kF(0, kF, kTimeoutMs);
      // use slot0 for closed-looping
      shooter.SelectProfileSlot(0, 0);

      // set the peak and nominal outputs, 1.0 means full
      shooter.ConfigNominalOutputForward(0.0f, kTimeoutMs);
      shooter.ConfigNominalOutputReverse(0.0f, kTimeoutMs);
      shooter.ConfigPeakOutputForward(+1.0f, kTimeoutMs);
      shooter.ConfigPeakOutputReverse(-1.0f, kTimeoutMs);

      // how much error is allowed?  This defaults to 0.
      shooter.ConfigAllowableClosedloopError(0, 0, kTimeoutMs);

      shooter.ConfigClosedloopRamp(0.5f, kTimeoutMs);
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
  }
}
