/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.DataRecorder;
import frc.robot.subsystems.DataRecorder.datapoint;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX m_IntakeMotor;
  private final WPI_TalonSRX m_IntakeArm;
  private boolean intakeExtended;
  private double m_CurrentSpeed;

  private DataRecorder dataRecorder;
  private Integer RecordCurrentSpeedix;
  private Integer IntakeUpOrDownix;
  
  public static final class IntakeArmConstants {
    //TODO run arm motor to extended position, find right position
    public static final double armRetractedPos = 630000;
    public static final double armExtendedPos = 0;
    
    // intake arm PID values
    public static final double kP = 0.04; 
    public static final double kI = 0.0002;
    public static final double kD = 0.0; 
    public static final double kIz = 8000; 
    public static final double kFF = 0;//.000015; 
    // public static final double kMaxOutput = 1; 
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700;  
}

  public static final class IntakeMotorConstants {

    public static final double kP = 0.04; 
    public static final double kI = 0.00;
    public static final double kD = 0; 
    public static final double kIz = 8000; 
    public static final double kFF = 0;//.000015; 
    // public static final double kMaxOutput = 1; 
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700; 

  }
  /**
   * Creates a new Intake.
   */
  public Intake() {
    m_IntakeMotor = new WPI_TalonSRX(Motors.BALL_INTAKE);
    m_IntakeMotor.configFactoryDefault();
    m_IntakeMotor.setInverted(false);
    m_IntakeMotor.configClosedloopRamp(0.5);

    
    m_IntakeMotor.config_kP(0, IntakeMotorConstants.kP);
    m_IntakeMotor.config_kI(0, IntakeMotorConstants.kI);
    m_IntakeMotor.config_kD(0, IntakeMotorConstants.kD);
    m_IntakeMotor.config_IntegralZone(0, IntakeMotorConstants.kIz);
    m_IntakeMotor.config_kF(0, IntakeMotorConstants.kFF);

    double junk = SmartDashboard.getNumber("intakeSpeed", 1000);
    SmartDashboard.putNumber("intakeSpeed", junk);

    m_CurrentSpeed = 0;
   

    m_IntakeArm = new WPI_TalonSRX(Motors.INTAKE_ARM);
    m_IntakeArm.configFactoryDefault();
    m_IntakeArm.setInverted(false);
    m_IntakeArm.setNeutralMode(NeutralMode.Brake);
    m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armRetractedPos);
    // PID values for INTAKE_ARM
    m_IntakeArm.config_kP(0, IntakeArmConstants.kP);
    m_IntakeArm.config_kI(0, IntakeArmConstants.kI);
    m_IntakeArm.config_kD(0, IntakeArmConstants.kD);
    m_IntakeArm.config_IntegralZone(0, IntakeArmConstants.kIz);
    m_IntakeArm.config_kF(0, IntakeArmConstants.kFF);


    m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armRetractedPos);
    // m_IntakeArm.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs)
    // m_IntakeArm.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs)

    dataRecorder = null;
    RecordCurrentSpeedix = 0;
    IntakeUpOrDownix = null;

    intakeExtended = false;
  }

  public void setDataRecorder(DataRecorder _dataRecorder, Integer _intakeUpOrDown, Integer _currentSpeed){
    this.dataRecorder = _dataRecorder;
    this.IntakeUpOrDownix = _intakeUpOrDown;
    this.RecordCurrentSpeedix = _currentSpeed;
  }

   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // runMotor(m_CurrentSpeed);
        //m_IntakeMotor.set(ControlMode.PercentOutput, m_CurrentSpeed);
    m_IntakeMotor.set(ControlMode.Velocity, m_CurrentSpeed);
SmartDashboard.putNumber("IntakeArmAt", m_IntakeArm.getSelectedSensorPosition());
    // if upper or lower limit switch is hit, then reset encoder position to upper or lower
    if (m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed()){
      m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armRetractedPos);
    }
    else if (m_IntakeArm.getSensorCollection().isRevLimitSwitchClosed()){
      m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armExtendedPos);
    }
  }
  // public void runMotor(double speed){
  //   //m_IntakeMotor.set(ControlMode.PercentOutput, speed);
  //   m_IntakeMotor.set(ControlMode.Velocity, speed);
  // }
public void extendArm(){
  // m_IntakeArm.set(ControlMode.PercentOutput, 1);
  m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armExtendedPos);
}
public void retractArm(){
  //m_IntakeArm.set(ControlMode.PercentOutput, -1);
  m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armRetractedPos);
}

public void stopArm() {
  m_IntakeArm.set(ControlMode.PercentOutput, 0);
}

public void ToggleIntake(){
  
  if (!intakeExtended){
      intakeExtended = true;
      m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armExtendedPos);
      // m_CurrentSpeed = m_IntakeSpeed;
      //m_CurrentSpeed = SmartDashboard.getNumber("intakeSpeed", 0.20);

      

    }
    else if (intakeExtended) {
      intakeExtended = false;
      //run arm motor to retracted position
      m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armRetractedPos);
      //m_CurrentSpeed = 0;
    }
  
    if (this.dataRecorder != null) {

      if (intakeExtended) {
        this.dataRecorder.recordValue(IntakeUpOrDownix, 1);
      }
      if (!intakeExtended) {
        this.dataRecorder.recordValue(IntakeUpOrDownix, 0);
      }
  
    }
  }

  public void HeimlichManeuver() {
    //m_CurrentSpeed = -0.2;
    m_CurrentSpeed = -1 * SmartDashboard.getNumber("intakeSpeed", 0.20);
  }
  public void StopIntake() {
    //m_CurrentSpeed = -0.2;
    m_CurrentSpeed = 0;
  }
  public void StartIntake() {
    //m_CurrentSpeed = -0.2;
    m_CurrentSpeed = SmartDashboard.getNumber("intakeSpeed", 0.20);
  }
  // public void ResumeNormalSpeed() {
  //   if (intakeExtended) {
  //     //m_CurrentSpeed = m_IntakeSpeed;
  //     m_CurrentSpeed = SmartDashboard.getNumber("intakeSpeed", 0.20);
  //   }

  //   else {
  //     m_CurrentSpeed = 0;
  //   }
  // }

  }