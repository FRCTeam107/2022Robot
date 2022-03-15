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

public class PullUpOntoTalonHooks extends CommandBase {
    private final Climber m_climber;

    private enum commandState {
      Starting,
      StraightenArm,
      PullTalonsAboveBar,
      TransferOntoTalons,
      CheckIfHooksLatched,
      Finished;

      public commandState getNext() {
        return this.ordinal() < commandState.values().length - 1
            ? commandState.values()[this.ordinal() + 1]
            : null;
      }
    }
    private commandState currentState;



  public PullUpOntoTalonHooks(Climber climber) {
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
    boolean moveToNextState = false;

    switch (currentState){
      case Starting:
          // if both talon hooks are set, then don't do anything
          if (m_climber.AllTalonsHooked()) {
            currentState=commandState.Finished;
          }
          else {
            moveToNextState=true;
          }

      case StraightenArm:
          // move arm to vertical
          if (m_climber.pullArmForwardToPosition(ClimberConstants.armVerticalPos)){
            moveToNextState = true;
          }

      case PullTalonsAboveBar:
      // pull hook so talon hooks go past the bar
      if (m_climber.pullHookToPosition(ClimberConstants.hookAboveBarPos)){
        moveToNextState = true;
      }

      case TransferOntoTalons:
      // extend hook up so weight is transferred to talon hooks
      if (m_climber.extendHookToPosition(ClimberConstants.hookTransferToTalonsPos)){
        moveToNextState = true;
      }

      case CheckIfHooksLatched:
      // if both talon hooks are not set, then do another pull-up to try again
        if (m_climber.AllTalonsHooked()){
          moveToNextState = true;
        } 
        else { // failed transfer, try again
          //currentState = commandState.PullTalonsAboveBar;
        }

      case Finished:
      m_climber.stopArm();
      m_climber.stopHook();
      
      default:
    }

    // move to next state?
    if (moveToNextState){ 
      currentState = currentState.getNext();
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