/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int TopMotorId = 5;
  private static final int BottomMotorID = 6;
  private CANSparkMax m_motorTop;
  private CANSparkMax m_motorBottom;
  private CANPIDController m_pidController;
  private CANPIDController m_pidControllerBottom;
  private CANEncoder m_encoder;
  private CANEncoder m_encoderBottom;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_motorTop = new CANSparkMax(TopMotorId, MotorType.kBrushless);
    m_motorBottom = new CANSparkMax(BottomMotorID, MotorType.kBrushless);
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters in the SPARK MAX to their factory default state. If no argument is
     * passed, these parameters will not persist between power cycles
     */
    m_motorTop.restoreFactoryDefaults();
    m_motorBottom.restoreFactoryDefaults();

    m_motorTop.setIdleMode(IdleMode.kCoast);
    m_motorBottom.setIdleMode(IdleMode.kCoast);

    m_motorBottom.setInverted(true);

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motorTop.getPIDController();
    m_pidControllerBottom = m_motorBottom.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motorTop.getEncoder();
    m_encoderBottom = m_motorBottom.getEncoder();

    // PID coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 15000;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // set PID coefficients
    m_pidControllerBottom.setP(kP);
    m_pidControllerBottom.setI(kI);
    m_pidControllerBottom.setD(kD);
    m_pidControllerBottom.setIZone(kIz);
    m_pidControllerBottom.setFF(kFF);
    m_pidControllerBottom.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("SetPoint", 0);
    SmartDashboard.putNumber("Velocity", 0);

   



  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      m_pidController.setP(p);
      m_pidControllerBottom.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      m_pidControllerBottom.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      m_pidControllerBottom.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      m_pidControllerBottom.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      m_pidControllerBottom.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      m_pidControllerBottom.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    /**
     * PIDController objects are commanded to a set point using the SetReference()
     * method.
     * 
     * The first parameter is the value of the set point, whose units vary depending
     * on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four
     * parameters: com.revrobotics.ControlType.kDutyCycle
     * com.revrobotics.ControlType.kPosition com.revrobotics.ControlType.kVelocity
     * com.revrobotics.ControlType.kVoltage
     */
    double setPoint = m_stick.getY() * maxRPM;
    m_pidController.setReference((setPoint * .8), ControlType.kVelocity);
    m_pidControllerBottom.setReference(setPoint , ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Velocity", m_encoder.getVelocity());
  }
}
