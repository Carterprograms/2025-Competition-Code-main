package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSys;

public class AutoAlignCmd extends Command {
    private final SwerveSys swerveSys;
    private static final double kP = 0.1; // Proportional gain
    private static final double kP_X = 0.1; // Proportional gain for x adjustment
    private static final double kP_Y = 0.1; // Proportional gain for y adjustment
    private static final double kTolerance = 1.0;

    public AutoAlignCmd(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        addRequirements(swerveSys);
    }

    @Override
    public void initialize() {
        // Initialize code if needed
    }

    @Override
    public void execute() {
        double tx = NetworkTableInstance.getDefault().getTable("Limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        double steeringAdjust = kP * tx;
        double xAdjust = kP_X * tx;
        double yAdjust = kP_Y * ty;

        swerveSys.drive(xAdjust, yAdjust, steeringAdjust, true);

        
    }

    @Override
    public boolean isFinished() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        return Math.abs(tx) < kTolerance && Math.abs(ty) < kTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSys.drive(0.0, 0.0, 0.0, true); // Stop the robot
    }
}
