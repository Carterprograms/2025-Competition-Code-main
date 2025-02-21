package frc.robot.commands.drivetrain;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class ReefPositioningCmd extends Command{
    
    private final SwerveSys swerveSys;
    private Rotation2d targetheading;
    private final boolean shouldMirrorHeading;
    private final ProfiledPIDController aimController;


    public ReefPositioningCmd(Rotation2d targetheading, Boolean shouldMirrorHeading, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.targetheading = targetheading;
        this.shouldMirrorHeading = shouldMirrorHeading;

        aimController = new ProfiledPIDController(AutoConstants.autoAimkP, 0, AutoConstants.autoAimkD,
        new Constraints(AutoConstants.autoAimTurnSpeedRadPerSec,
        AutoConstants.autoAumTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ReefPositioningCmd(Rotation2d targetheading, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.targetheading = targetheading;
        this.shouldMirrorHeading = true;

        aimController = new ProfiledPIDController(AutoConstants.autoAimkP, 0, AutoConstants.autoAimkD,
        new Constraints(AutoConstants.autoAimTurnSpeedRadPerSec,
        AutoConstants.autoAumTurnAccelRadPerSecSq));
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        PPHolonomicDriveController.setRotationTargetOverride(()-> Optional.of(targetheading));

        Rotation2d mirroredHeading;
        if(shouldMirrorHeading && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            mirroredHeading = Rotation2d.fromDegrees(180).minus(targetheading);
        }
        else{
           mirroredHeading = targetheading;
        }

        if(Math.abs(swerveSys.getHeading().getDegrees() - mirroredHeading.getDegrees()) > AutoConstants.autoAimToleranceDeg) {
            double aimRadPerSec = aimController.calculate(swerveSys.getHeading().getDegrees(), mirroredHeading.getRadians());
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(aimRadPerSec));
        }
        else{
            swerveSys.setOmegaOverrideRadPerSec(Optional.of(0.0));
        }
    }

    @Override
    public void end(boolean isInterrupted){
        swerveSys.setOmegaOverrideRadPerSec(Optional.empty());
        PPHolonomicDriveController.setRotationTargetOverride(()-> Optional.of(targetheading));
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
