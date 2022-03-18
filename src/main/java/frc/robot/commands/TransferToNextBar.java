/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberConstants;

public class TransferToNextBar extends CommandBase {
    private final Climber m_climber;
    private final BooleanSupplier m_forceToRun;

    private enum commandState {
      Starting,
      RaiseHookPunchNextBar,
      BendArmToPunchNextBar,
      WaitForSwingToStop,
      LowerHookBelowBar,
      BendArmBackToNextBar,
      ReachHookPastNextBar,
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
    private static int countDown;

  public TransferToNextBar(Climber climber, BooleanSupplier _forceToRun) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_forceToRun = _forceToRun;

    currentState = commandState.Starting;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = commandState.Starting;
    countDown = 0;
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
          moveToNextState = m_forceToRun.getAsBoolean();
        }
        // else (if m_btnForceReady.is
        //   moveToNextState = true;
        // }
        // else {
        //   currentState = commandState.Finished;
        // }
        break;
      
      case RaiseHookPunchNextBar:
        // extend the hook past the current bar and to punch next one
        if (m_climber.moveHookToPosition(ClimberConstants.hookToPunchNextBar, true)){
          moveToNextState = true;
        }
        break;

      case BendArmToPunchNextBar:
        if (m_climber.moveArmToPosition(ClimberConstants.armToPunchNextBar)){
          countDown = 100; // 20ms loop * countdown timer
          moveToNextState = true;
        }
        break;

      case WaitForSwingToStop:
        countDown --;
        moveToNextState = (countDown<=0);
        break;

      case LowerHookBelowBar:
        // extend the hook past the current bar and to punch next one
        if (m_climber.moveHookToPosition(ClimberConstants.hookBelowNextBar, true)){
          moveToNextState = true;
        }
        break;
      
      case BendArmBackToNextBar:
        // move arm to reach backwards slightly past the next bar
        if (m_climber.moveArmToPosition(ClimberConstants.armReachPastNextBar)){
          moveToNextState = true;
        }
        break;
        
      case ReachHookPastNextBar:
        // extend the hook past the next bar
        if (m_climber.moveHookToPosition(ClimberConstants.hookPastNextBar, true)){
          moveToNextState = true;
        }
        break;

      case RetractArmToTouchBar:
        // now bring arm forward to make arm touch the bar
        if (m_climber.moveArmToPosition(ClimberConstants.armHugNextBar)) {
          moveToNextState = true;
        }
        break;

      case PullHookToReleaseTalons:
        // pull the hook far enough to release the talons hooks
        if (m_climber.moveHookToPosition(ClimberConstants.hookPullTalonsOffBar, false) ) {
          moveToNextState = true;
        }
        break;
     
      case Finished:
        break;

      default:
        //break;
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
