package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.ButtonTestingCmd;
import frc.robot.commands.drivetrain.LockCmd;
import frc.robot.commands.drivetrain.Lvl1Cmd;
import frc.robot.commands.drivetrain.Lvl2Cmd;
import frc.robot.commands.drivetrain.Lvl3Cmd;
import frc.robot.commands.drivetrain.Lvl4Cmd;
import frc.robot.commands.drivetrain.ReefPositioningCmd;
import frc.robot.commands.drivetrain.TurnToHeadingCmd;
import frc.robot.commands.drivetrain.AimToReefCmd;
import frc.robot.commands.lights.LightsDefaultCmd;
import frc.robot.commands.lights.PartyModeCmd;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.LiftSys;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final static LightsSys lightsSys = new LightsSys();
    private final LiftSys liftSys = new LiftSys();
    
        //Initialize joysticks.
        private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverGamepadPort);
        public final static CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorGamepadPort);
        public static Joystick ButtonPanel = new Joystick(ControllerConstants.buttonPanelPort);
    
        //Initialize auto selector.
        SendableChooser<Command> autoSelector = new SendableChooser<Command>();
    
        public RobotContainer() {
            RobotController.setBrownoutVoltage(DriveConstants.brownoutVoltage);

            NamedCommands.registerCommand("lvl4", new Lvl4Cmd(liftSys));
            NamedCommands.registerCommand("lvl3", new Lvl3Cmd(liftSys));
            NamedCommands.registerCommand("lvl2", new Lvl2Cmd(liftSys));
            NamedCommands.registerCommand("lvl1", new Lvl1Cmd(liftSys));
    
            SmartDashboard.putData("auto selector", autoSelector);

            configDriverBindings();
            buttonPanelBindings();
    
        }


    public void buttonPanelBindings() {
        JoystickButton lvl4ButtonRight = new JoystickButton(ButtonPanel, 0);
        JoystickButton lvl4ButtonLeft = new JoystickButton(ButtonPanel, 1);
        JoystickButton lvl3ButtonRight = new JoystickButton(ButtonPanel, 2);
        JoystickButton lvl3ButtonLeft = new JoystickButton(ButtonPanel, 3);
        JoystickButton lvl2ButtonRight = new JoystickButton(ButtonPanel, 4);
        JoystickButton lvl2ButtonLeft = new JoystickButton(ButtonPanel, 5);
        JoystickButton lvl1ButtonRight = new JoystickButton(ButtonPanel, 6);
        JoystickButton lvl1ButtonLeft = new JoystickButton(ButtonPanel, 7);
        Joystick joystick = new Joystick(8);
        JoystickButton conveyorButton = new JoystickButton(ButtonPanel, 9);
        JoystickButton coralReleaseButton = new JoystickButton(ButtonPanel, 10);

        lvl4ButtonRight.whileTrue(new Lvl4Cmd(liftSys));
        lvl3ButtonRight.whileTrue(new Lvl3Cmd(liftSys));
        lvl2ButtonRight.whileTrue(new Lvl2Cmd(liftSys));
        lvl1ButtonRight.whileTrue(new Lvl1Cmd(liftSys));
        System.out.println(joystick.getX());
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

        driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.triggerPressedThreshhold)
            .whileTrue(new LockCmd(swerveSys));
        
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

       //driverController.leftStick().toggleOnTrue(new Lvl4Cmd(liftSys));
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
