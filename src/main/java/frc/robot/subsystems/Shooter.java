/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.Motors;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX m_shootbottom, m_shoottop;
  private double setSpeedTop, setSpeedBottom;

  // private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;
  // private int kIz;
  private boolean manualForceReady;
  private int readyCounter;  // number of times shooters report ready in a row

  private boolean cacheTopReady, cacheBottomReady;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    super();

    manualForceReady = false;
    cacheBottomReady = false;
    cacheTopReady = false;

    readyCounter = 0;
    m_shootbottom = new WPI_TalonFX(Motors.SHOOTER_BOTTOM);
    m_shoottop = new WPI_TalonFX(Motors.SHOOTER_TOP);

    m_shootbottom.configFactoryDefault();
    m_shoottop.configFactoryDefault();

    // m_shootbottom.configClosedloopRamp(0.3);
    // m_shoottop.configClosedloopRamp(0.3);
   
    setSpeedTop = 0;
    setSpeedBottom = 0;

    // setup PID closed-loop values
    // kP = 0.25; 
    // kI = 0.0005;
    // kD = 0.0001; 
    // kIz = 8000; 
    // kFF = 0;//.000015; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;  
    m_shootbottom.config_kP(0, ShooterConstants.kP);
    m_shootbottom.config_kI(0, ShooterConstants.kI);
    m_shootbottom.config_kD(0, ShooterConstants.kD);
    m_shootbottom.config_IntegralZone(0, ShooterConstants.kIz);
    m_shootbottom.config_kF(0, ShooterConstants.kFF);
    // m_shootbottom.configClosedLoopPeakOutput(slotIdx, percentOut)

    m_shoottop.config_kP(0, ShooterConstants.kP);
    m_shoottop.config_kI(0, ShooterConstants.kI);
    m_shoottop.config_kD(0, ShooterConstants.kD);
    m_shoottop.config_IntegralZone(0, ShooterConstants.kIz);
    m_shoottop.config_kF(0, ShooterConstants.kFF);
    // m_shoottop.configClosedLoopPeakOutput(slotIdx, percentOut)

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Shooter Real B", m_shootbottom.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Shooter Real T", m_shoottop.getSelectedSensorVelocity());

    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // int iz = (int)SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    //double max = SmartDashboard.getNumber("Max Output", 0);
    //double min = SmartDashboard.getNumber("Min Output", 0);

    // SmartDashboard.putNumber("shoot pos", m_shootbottom.getSelectedSensorPosition());

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { kP = p; m_shoottop.config_kP(0, p);m_shootbottom.config_kP(0, p); }
    // if((i != kI)) { kI = i; m_shoottop.config_kI(0, i); m_shootbottom.config_kI(0, i); }
    // if((d != kD)) { kD = d; m_shoottop.config_kD(0, d); m_shootbottom.config_kD(0, d); }
    // if((iz != kIz)) { kIz = iz; m_shoottop.config_IntegralZone(0, iz); m_shootbottom.config_IntegralZone(0, iz); }
    // if((ff != kFF)) { kFF = ff; m_shoottop.config_kF(0, ff);m_shootbottom.config_kF(0, ff); }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   m_shootbottom.setOutputRange(min, max);  kMinOutput = min; kMaxOutput = max; }
 }

 
  public void runMotor(double speedbottom, double speedtop){
    setSpeedBottom = -speedbottom;
    setSpeedTop = speedtop;

    if (setSpeedTop == 0 ){
      m_shootbottom.set(TalonFXControlMode.PercentOutput, 0);
      m_shoottop.set(TalonFXControlMode.PercentOutput,0);
    }
    else
    {
      m_shootbottom.set(TalonFXControlMode.Velocity, setSpeedBottom);
      m_shoottop.set(TalonFXControlMode.Velocity, setSpeedTop);      
    }
  }

  public void setForceReady(boolean enableOverride){
    manualForceReady = enableOverride;
  }

  public boolean isReady(){
    if (manualForceReady) {
      return true;
    }

    boolean topReady = (setSpeedTop!=0 && Math.abs(m_shoottop.getSelectedSensorVelocity() - setSpeedTop) <= 100);
    boolean bottomReady = (setSpeedBottom!=0 && Math.abs(m_shootbottom.getSelectedSensorVelocity() - setSpeedBottom) <= 100);
    
    if (topReady && bottomReady) {readyCounter += 1; }
    //else {readyCounter = 0; }

    if (cacheTopReady != topReady) {cacheTopReady=topReady; SmartDashboard.putBoolean("i T Ready", topReady); }
    if (cacheBottomReady != bottomReady) {cacheBottomReady=bottomReady; SmartDashboard.putBoolean("i B Ready", bottomReady); }
    
    return (readyCounter > 15); //require 15 consecutive readies before reporting we are ready
  }

  public void clearReadyFlags(){
    cacheTopReady=false;
    cacheBottomReady=false;
    readyCounter = 0;
    SmartDashboard.putString("ResetShooter", "Reset");
  }
}