package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.LiftSys;

public class LiftCmd extends Command {
    
    private final ProfiledPIDController liftController;
    private final boolean shouldMirrorDirection;
    private final LiftSys liftSys;

    public LiftCmd(LiftSys liftSys, boolean shouldMirrorDirection) {
        this.liftSys = liftSys;
        this.shouldMirrorDirection = shouldMirrorDirection;

        liftController = new ProfiledPIDController(AutoConstants.autoLiftkP, 0, AutoConstants.autoLiftkD, 
        new Constraints(AutoConstants.liftMaxVelMetersPerSec,
        AutoConstants.liftMaxAccelMeterPerSecSq));
    }

}
