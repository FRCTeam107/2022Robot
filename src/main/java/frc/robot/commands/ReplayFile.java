/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.opencv.core.Size;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DataRecorder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.DataRecorder.datapoint;

public class ReplayFile extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final SwerveDrivetrain m_drivetrain;
  private final Shooter m_Shooter;
  private final Intake m_Intake;
  private final DataRecorder m_datarecorder;
  private final List<double[]> m_replayList;
  private int replayPoint;
  private boolean intakeArmDown = false;

  private StringBuilder msg = new StringBuilder("");
 
  public ReplayFile(SwerveDrivetrain _drivetrain, Intake _intake, Shooter _shooter, DataRecorder _datarecoder, String filename) {
    m_drivetrain = _drivetrain;
    m_Intake = _intake;
    m_Shooter = _shooter;
    m_datarecorder = _datarecoder;
    m_replayList = m_datarecorder.LoadFile(filename);
   

    // SmartDashboard.putString("ReplayFile", "instantiate");
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(m_drivetrain, m_Shooter, m_datarecorder);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putString("ReplayFile", "initailize");
        //m_Limelight.EnableVisionProcessing();
    replayPoint = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putString("ReplayFile", "execute");

    if (replayPoint > m_replayList.size() - 1) { return; }

  //   if (m_replayList.size() > 0){
    double[] replayRow = m_replayList.get(replayPoint);

    if (replayRow.length<10)
    {
      SmartDashboard.putNumber("ReplayErrorLine", replayPoint);
      SmartDashboard.putNumberArray("ReplayErrorData", replayRow);
      replayPoint += 1;
      return;
    }

    m_drivetrain.drive(replayRow[datapoint.Drive_X], 
          replayRow[datapoint.Drive_Y],
          replayRow[datapoint.Drive_Z], 
          true);

    // move intake arm up or down
    if (replayRow[datapoint.IntakeIsExtended]>0 && !intakeArmDown) {
      m_Intake.extendArm();
      intakeArmDown = true;
    }
    else if (replayRow[datapoint.IntakeIsExtended]==0 && intakeArmDown) {
      m_Intake.retractArm();
      intakeArmDown = false;
    }
    m_Intake.runIntake(replayRow[datapoint.IntakeMotorSpeed]);

    m_Shooter.runMotor(replayRow[datapoint.ShooterBottom], replayRow[datapoint.ShooterTop]);
    m_Shooter.runKicker(replayRow[datapoint.KickerSpeed]);

    //m_replayList.remove(0);   
    replayPoint += 1;

    //SmartDashboard.putString("ReplayData", msg.toString());
    //System.out.println(">>> REPLAY <<<" + msg.toString());
    //msg = new StringBuilder("");
    //}
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_turret.isShooting = false;
    // SmartDashboard.putString("ReplayFile", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    // stop when we hit the end of the list
    return(replayPoint > m_replayList.size() - 1);
  }
}
