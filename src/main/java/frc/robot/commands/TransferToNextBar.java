/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TransferToNextBar extends CommandBase {
    private final Climber m_climber;

    private enum commandState {
      Starting,
      BendArmBackToNextBar,
      ReachHookPastBar,
      RetractArmToTouchBar,
      PullHookToReleaseTalons,
      Finished;

      public commandState getNext() {
        return this.ordinal() < commandState.values().length - 1
            ? commandState.values()[this.ordinal() + 1]
            : null;
      }
    }
    private commandState currentState;



  public TransferToNextBar(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    currentState = commandState.Starting;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentState){
      case Starting:
      // if both talon hooks are NOT set, don't do anything!
          currentState = currentState.getNext();

      case BendArmBackToNextBar:
      // move arm to reach backwards slightly past the next bar

      case ReachHookPastBar:
      // extend the hook past the next bar

      case RetractArmToTouchBar:
      // now bring arm forward to make arm touch the bar

      case PullHookToReleaseTalons:
      // pull the hook far enough to release the talons hooks 

      case Finished:

      default:
    }
    //m_climber.runMotor(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopHook();
    m_climber.stopArm();
    currentState = commandState.Starting; // reset to starting state
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
