package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class ButtonTestingCmd extends Command {
    public ButtonTestingCmd() {

    }

      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        System.out.println("Button Pressed");
      }

      @Override
      public void end(boolean isInterrupted){

      }

      @Override
      public boolean isFinished() {
        return false;
      }
}
