/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberConstants;

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
    private static commandState currentState;

  public TransferToNextBar(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    currentState = commandState.Starting;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = commandState.Starting;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean moveToNextState = false;

    switch (currentState){
      case Starting:
        // if both talon hooks are NOT set, don't do anything!
        if (m_climber.AllTalonsHooked()){
          moveToNextState = true;
        }
        else {
          currentState = commandState.Finished;
        }
         
      case BendArmBackToNextBar:
        // move arm to reach backwards slightly past the next bar
        if (m_climber.moveArmToPosition(ClimberConstants.armReachForNextBar)){
          moveToNextState = true;
        }
        
      case ReachHookPastBar:
        // extend the hook past the next bar
        if (m_climber.moveHookToPosition(ClimberConstants.hookReachPastNextBarPos)){
          moveToNextState = true;
        }

      case RetractArmToTouchBar:
        // now bring arm forward to make arm touch the bar
        if (m_climber.moveArmToPosition(ClimberConstants.armTouchNextBar)) {
          moveToNextState = true;
        }

      case PullHookToReleaseTalons:
        // pull the hook far enough to release the talons hooks
        if (m_climber.moveHookToPosition(ClimberConstants.hookTransferToTalonsPos) ) {
          moveToNextState = true;
        }
     
      case Finished:
        // m_climber.stopArm();
        // m_climber.stopHook();

      default:
    }

    // move to next state?
    if (moveToNextState){ 
      currentState = currentState.getNext();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted){
      m_climber.stopHook();
      m_climber.stopArm(); 
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
