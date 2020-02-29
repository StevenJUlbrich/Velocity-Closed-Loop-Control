/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int TopMotorId = 5;
  private static final int BottomMotorID = 6;
  private static final int IndexMotorID = 7;
  private CANSparkMax m_motorTop;
  private CANSparkMax m_motorBottom;
  private WPI_TalonFX m_indexMotor;
  private CANPIDController m_pidController;
  private CANPIDController m_pidControllerBottom;
  private CANEncoder m_encoder;
  private CANEncoder m_encoderBottom;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public double maxRPM_Top, maxRPM_Bottom, kMaxRPM_Top, kMaxRPM_Bottom;

  public static String cnameMaxRpmTop = "Top Motor Max RPM";
  public static String cnameMaxRpmBottom = "Bottom Motor Max RPM";



  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_motorTop = new CANSparkMax(TopMotorId, MotorType.kBrushless);
    m_motorBottom = new CANSparkMax(BottomMotorID, MotorType.kBrushless);
    m_indexMotor = new WPI_TalonFX(IndexMotorID);



    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters in the SPARK MAX to their factory default state. If no argument is
     * passed, these parameters will not persist between power cycles
     */
    m_motorTop.restoreFactoryDefaults();
    m_motorBottom.restoreFactoryDefaults();

    m_motorTop.setIdleMode(IdleMode.kCoast);
    m_motorBottom.setIdleMode(IdleMode.kCoast);

    m_motorTop.setInverted(true);

    m_indexMotor.set(ControlMode.PercentOutput, 0);
    m_indexMotor.configFactoryDefault();
    m_indexMotor.setNeutralMode(NeutralMode.Brake);

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
    kMaxRPM_Top = 15000;
    kMaxRPM_Bottom = 15000;
    maxRPM = 15000;
    maxRPM_Bottom = 15000;
    maxRPM_Top = 15000;




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

    SmartDashboard.putNumber("SetPoint-TopMotor", 0);
    SmartDashboard.putNumber("RPM-TopMotor", 0);
    SmartDashboard.putNumber(cnameMaxRpmTop, kMaxRPM_Top);

    SmartDashboard.putNumber("SetPoint-BottomMotor", 0);
    SmartDashboard.putNumber("RPM-BottomMotor", 0);
    SmartDashboard.putNumber(cnameMaxRpmBottom, kMaxRPM_Bottom);

  }

  @Override
  public void teleopPeriodic() {

    // read PID coefficients from SmartDashboard
    /**
     * final double p = SmartDashboard.getNumber("P Gain", 0); final double i =
     * SmartDashboard.getNumber("I Gain", 0); final double d =
     * SmartDashboard.getNumber("D Gain", 0); final double iz =
     * SmartDashboard.getNumber("I Zone", 0); final double ff =
     * SmartDashboard.getNumber("Feed Forward", 0);
     */




    if (maxRPM_Top != kMaxRPM_Top){
        kMaxRPM_Top= maxRPM_Top;
    }

    if (maxRPM_Bottom != kMaxRPM_Bottom){
        kMaxRPM_Bottom = maxRPM_Bottom;
    }

    final double maxRPM_Top = SmartDashboard.getNumber(cnameMaxRpmTop, kMaxRPM_Bottom);
    final double maxRPM_Bottom = SmartDashboard.getNumber(cnameMaxRpmBottom, kMaxRPM_Bottom);
    
 
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
    final double m_stickY = Math.abs(m_stick.getY());

    final double setPointTop = m_stickY * maxRPM_Top;
    final double setPointBottom = m_stickY * maxRPM_Bottom;

    m_pidController.setReference((setPointTop), ControlType.kVelocity);
    m_pidControllerBottom.setReference(setPointBottom, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint Top",(setPointTop));
    SmartDashboard.putNumber("RPM Raw Motor Top", Math.abs(m_encoder.getVelocity()));

    SmartDashboard.putNumber("SetPoint Bottom", (setPointBottom));
    SmartDashboard.putNumber("RPM Raw Motor Bottom", Math.abs(m_encoderBottom.getVelocity()));

    //m_stick.getRawButtonPressed(button)
    m_indexMotor.set(ControlMode.PercentOutput, 0.20);
    
    
  }

}
