package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.LockCmd;
import frc.robot.commands.drivetrain.ReefPositioningCmd;
import frc.robot.commands.drivetrain.TurnToHeadingCmd;
import frc.robot.commands.drivetrain.AimToReefCmd;
import frc.robot.commands.lights.LightsDefaultCmd;
import frc.robot.commands.lights.PartyModeCmd;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.LiftSys;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final static LightsSys lightsSys = new LightsSys();
        private final LiftSys liftSys = new LiftSys();
        
        //Initialize joysticks.
        private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
        public final static CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);
    
        //Initialize auto selector.
        SendableChooser<Command> autoSelector = new SendableChooser<Command>();
    
        public RobotContainer() {
            RobotController.setBrownoutVoltage(DriveConstants.brownoutVoltage);
    
            SmartDashboard.putData("auto selector", autoSelector);
    
            // Add programs to auto selector.
            /*autoSelector.setDefaultOption("Do Nothing", null);
            autoSelector.addOption("Example Auto", new ExampleAuto(swerveSys));
            autoSelector.addOption("AllianceFour", new AllianceFour(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
            autoSelector.addOption("AmpMidlineTwo", new AmpMidlineTwo(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
            //autoSelector.addOption("AmpMidlineThree", new AmpMidlineThree(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
            // autoSelector.addOption("PiHiThreePiece", new PiHiThreePiece(swerveSys, feederSys, rollerSys, pivotSys));
            autoSelector.addOption("SmashySmash", new SmashySmash(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
            // autoSelector.addOption("TestFive", new TestFivePiece(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
            autoSelector.addOption("AllianceFive", new AllianceFive(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
            // autoSelector.addOption("SecondPickThree", new SecondPickThree(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
            autoSelector.addOption("SourceMidlineTwo", new SourceMidlineTwo(swerveSys, feederSys, rollerSys, pivotSys, spacebarSys));
    */
            configDriverBindings();
            configOperatorBindings();
    
            //lightsSys.setDefaultCommand(new LightsDefaultCmd(lightsSys, rollerSys::hasNote));
        }
    
        public static void configOperatorBindings() {
    
            //     () -> (operatorController.getRightTriggerAxis() * RollerConstants.manualFirePower) - 
            //           (operatorController.getLeftTriggerAxis() * RollerConstants.manualIntakePower),
            //     rollerSys));
    
            /*pivotSys.setDefaultCommand(new PivotManualCmd( 
                () -> MathUtil.applyDeadband((operatorController.getLeftY()), ControllerConstants.joystickDeadband),
                pivotSys));*/
    
            //operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold).onFalse(new RollersStopCmd(rollerSys));
    
            //operatorController.leftBumper().onTrue(new FeederInCmd(feederSys)).onFalse(new FeederStopCmd(feederSys));
    
            //operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.triggerPressedThreshhold).onFalse(new RollersStopCmd(rollerSys));
    
            //operatorController.rightBumper().onTrue(new FeederFeedCmd(feederSys)).onFalse(new FeederStopCmd(feederSys));
    
            //operatorController.a().onTrue(new AutoAmpFireCmd(feederSys, rollerSys, pivotSys, spacebarSys));
            
            /*
            operatorController.b()
                .onTrue(new AutoSpeakerFireCmd(feederSys, rollerSys, pivotSys, swerveSys))
                .onFalse(new RollersStopCmd(rollerSys))
                .onFalse(new PivotHomePresetCmd(pivotSys))
                .onFalse(new FeederStopCmd(feederSys));
            */
    
            operatorController.start().toggleOnTrue(new PartyModeCmd(lightsSys));
    }

    public void configDriverBindings() {
        swerveSys.setDefaultCommand(new ArcadeDriveCmd(
            () -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
            () -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
            true,
            true,
            swerveSys));

        driverController.start().onTrue(Commands.runOnce(() -> swerveSys.resetHeading()));

        //Swerve locking system

        //driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
        //    .whileTrue(new LockCmd(swerveSys));
        
        //driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.triggerPressedThreshhold)
        //  .onTrue(new AutoGroundIntakeCmd(pivotSys, feederSys, rollerSys, spacebarSys)).onFalse(new AutoAllHomeCmd(pivotSys, feederSys, rollerSys));

        /*driverController.leftBumper().onTrue(new AutoFeedCmd(feederSys, rollerSys, pivotSys));

        driverController.rightBumper().onTrue(new AutoSourceIntakeCmd(pivotSys, feederSys, rollerSys)).onFalse(new AutoAllHomeCmd(pivotSys, feederSys, rollerSys));
        */
        
        driverController.y().whileTrue(new ReefPositioningCmd(Rotation2d.fromDegrees(180), swerveSys));
        driverController.b().whileTrue(new ReefPositioningCmd(Rotation2d.fromDegrees(120), swerveSys));
        driverController.rightBumper().whileTrue(new ReefPositioningCmd(Rotation2d.fromDegrees(60), swerveSys));
        driverController.a().whileTrue(new ReefPositioningCmd(Rotation2d.fromDegrees(0), swerveSys));
        driverController.leftBumper().whileTrue(new ReefPositioningCmd(Rotation2d.fromDegrees(-60), swerveSys));
        driverController.x().whileTrue(new ReefPositioningCmd(Rotation2d.fromDegrees(-120), swerveSys));
        driverController.rightStick().whileTrue(new AimToReefCmd(swerveSys));
    }

    public Command getAutonomousCommand() {
        // Returns Auto Selected from Shuffle Board
        return autoSelector.getSelected();
    } 

    // For uniformity, any information sent to Shuffleboard/SmartDashboard should go here.
    public void updateInterface() {
        // Heading & Speed
        SmartDashboard.putNumber("heading degrees", swerveSys.getHeading().getDegrees());
        SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSec());

        // Position X/Y from Pose Estimator
        SmartDashboard.putNumber("pose x meters", swerveSys.getPose().getX());
        SmartDashboard.putNumber("pose y meters", swerveSys.getPose().getY());

        // Position X from Pose Estimator from blue side
        SmartDashboard.putNumber("blue pose x meters", swerveSys.getBlueSidePose().getX());

        // Module Positions from Pose Estimator
        SmartDashboard.putNumber("FL angle degrees", swerveSys.getModuleStates()[0].angle.getDegrees());
        SmartDashboard.putNumber("FR angle degrees", swerveSys.getModuleStates()[1].angle.getDegrees());
        SmartDashboard.putNumber("BL angle degrees", swerveSys.getModuleStates()[2].angle.getDegrees());
        SmartDashboard.putNumber("BR angle degrees", swerveSys.getModuleStates()[3].angle.getDegrees());

        // Raw Module Positions
        SmartDashboard.putNumber("FL raw CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees());
        SmartDashboard.putNumber("FR raw CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees());
        SmartDashboard.putNumber("BL raw CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees());
        SmartDashboard.putNumber("BR raw CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees());
        
        // Offset
        SmartDashboard.putNumber("FL offset CANCoder degrees", swerveSys.getCanCoderAngles()[0].getDegrees() - DriveConstants.frontLeftModOffset.getDegrees());
        SmartDashboard.putNumber("FR offset CANCoder degrees", swerveSys.getCanCoderAngles()[1].getDegrees() - DriveConstants.frontRightModOffset.getDegrees());
        SmartDashboard.putNumber("BL offset CANCoder degrees", swerveSys.getCanCoderAngles()[2].getDegrees() - DriveConstants.backLeftModOffset.getDegrees());
        SmartDashboard.putNumber("BR offset CANCoder degrees", swerveSys.getCanCoderAngles()[3].getDegrees() - DriveConstants.backRightModOffset.getDegrees());

        // Average Drive Voltages
        SmartDashboard.putNumber("drive voltage", swerveSys.getAverageDriveVoltage());
    }   
}
