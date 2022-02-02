/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final Shooter m_shoot;
  private final BooleanSupplier m_forceShot;
  
  private double iTop, iBottom;
 
  public Shoot(Shooter _shooter, BooleanSupplier _forceShot) {
    m_shoot = _shooter;
    m_forceShot = _forceShot;

    iTop = SmartDashboard.getNumber("i Top", 5000);
    iBottom = SmartDashboard.getNumber("i Bottom", 7000);
    SmartDashboard.putNumber("i Top", iTop);
    SmartDashboard.putNumber("i Bottom", iBottom);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shoot);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedTop = 8000;
    double speedBottom = 10000;
    double liftPosition = 0;
    
    speedTop = SmartDashboard.getNumber("i Top", iTop);
    speedBottom = SmartDashboard.getNumber("i Bottom", iBottom);
    liftPosition = SmartDashboard.getNumber("i Lift", liftPosition);


    // if (m_Limelight.Havetarget()) {
    //   // double m_tX = m_Limelight.tX();   // how far off, the 'X' value
    //   // double m_tY = m_Limelight.tY();   //how far off, the 'Y' value
    //   double m_tA = m_Limelight.avgTA();   //distance from target estimation
    //   speedTop = m_tA * 3000;
    //   speedBottom = m_tA * 4000;
    //   speedTop = SmartDashboard.getNumber("i Top", iTop);
    //   speedBottom = SmartDashboard.getNumber("i Bottom", iBottom);
    // }

    m_shoot.runMotor(speedBottom ,speedTop);
    // SmartDashboard.putNumber("mShoot Top", speedTop);
    // SmartDashboard.putNumber("mShoot Bot", speedBottom);
   // m_turret.setLifterPosition(liftPosition);


    if (m_forceShot.getAsBoolean() || (m_shoot.isReady()) ) { //&& m_turret.isLiftReady()) ) { //&& m_Limelight.isReady()) ){
     // m_Indexer.runMotor(0.3);
     // m_Hopper.runMotor(0.41);

      double tt = Timer.getFPGATimestamp();
       if ((int)(tt * 10) % 3 == 0) {
       //  m_Hopper.runMotor(0.35);
       }
       else {
        // m_Hopper.runMotor(0);
       }
    }
     else
    {
      // m_Indexer.runMotor(-0.3);
      // m_Hopper.runMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_turret.isShooting = false;
    m_shoot.runMotor(0, 0);
    m_shoot.clearReadyFlags();
    //m_Indexer.runMotor(0);
    //m_Hopper.runMotor(0);
    //m_Limelight.DisableVisionProcessing();

    // SmartDashboard.putNumber("mShoot Top", 0);
    // SmartDashboard.putNumber("mShoot Bot", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}