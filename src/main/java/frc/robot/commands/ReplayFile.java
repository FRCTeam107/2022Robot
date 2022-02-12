/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DataRecorder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

public class ReplayFile extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final SwerveDrivetrain m_drivetrain;
  private final Shooter m_Shooter;
  private final DataRecorder m_datarecorder;
  private final List<double[]> m_replayList;
  
  
 
  public ReplayFile(SwerveDrivetrain _drivetrain, Shooter _shooter, DataRecorder _datarecoder, String filename) {
    m_drivetrain = _drivetrain;
    m_Shooter = _shooter;
    m_datarecorder = _datarecoder;
    m_replayList = m_datarecorder.LoadFile(filename);
   
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(m_drivetrain, m_Shooter, m_datarecorder);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_replayList.size()>0){
      double[] replayRow = m_replayList.get(0);
      m_drivetrain.replayRow(replayRow);
      m_replayList.remove(0);   
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_turret.isShooting = false;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
