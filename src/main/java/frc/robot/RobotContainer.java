/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOPhotonVision;
import frc.lib.io.vision.VisionIOPhotonVisionSim;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.AutoCommand;
import frc.lib.util.CommandXboxControllerExtended;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.OnTheFlyPathCommand;
import frc.robot.commands.autos.BranchingAuto;
import frc.robot.commands.autos.ExampleAuto;
import frc.robot.commands.autos.NoneAuto;
import frc.robot.commands.autos.WheelCharacterizationAuto;
import frc.robot.subsystems.clawroller.ClawRoller;
import frc.robot.subsystems.clawroller.ClawRollerConstants;
import frc.robot.subsystems.clawrollerlasercan.ClawRollerLaserCAN;
import frc.robot.subsystems.clawrollerlasercan.ClawRollerLaserCANConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsConstants;
import frc.robot.subsystems.tounge.Tounge;
import frc.robot.subsystems.tounge.Tounge;
import frc.robot.subsystems.tounge.Tounge.Setpoint;
import frc.robot.subsystems.tounge.ToungeConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.Arrays;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    // Subsystems
    public final Drive drive;
    private final LEDs leds;
    private final ClawRollerLaserCAN laserCAN1;
    private final ClawRoller flywheel;
    private final Elevator linear;
    private final Vision vision;
    private final Tounge tounge;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoCommand> autoChooser;
    private final LoggedDashboardChooser<Boolean> conditionalChooser;
    public static Field2d autoPreviewField = new Field2d();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(DriveConstants.FrontLeft),
                    new ModuleIOTalonFX(DriveConstants.FrontRight),
                    new ModuleIOTalonFX(DriveConstants.BackLeft),
                    new ModuleIOTalonFX(DriveConstants.BackRight));

                leds = new LEDs(LEDsConstants.getLightsIOReal());
                laserCAN1 = new ClawRollerLaserCAN(ClawRollerLaserCANConstants.getReal());
                flywheel = new ClawRoller(ClawRollerConstants.getReal());

                linear = new Elevator(ElevatorConstants.getReal());
                vision = new Vision(
                    drive::addVisionMeasurement,
                    () -> drive.getTimestampedHeading(),
                    new VisionIOPhotonVision(
                        VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0,
                        VisionConstants.aprilTagLayout,
                        PoseStrategy.CONSTRAINED_SOLVEPNP));
                tounge = new Tounge(ToungeConstants.getReal());
            }

            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(DriveConstants.FrontLeft),
                    new ModuleIOSim(DriveConstants.FrontRight),
                    new ModuleIOSim(DriveConstants.BackLeft),
                    new ModuleIOSim(DriveConstants.BackRight));

                leds = new LEDs(LEDsConstants.getLightsIOSim());
                laserCAN1 =
                    new ClawRollerLaserCAN(ClawRollerLaserCANConstants.getSim());
                flywheel = new ClawRoller(ClawRollerConstants.getSim());

                linear = new Elevator(ElevatorConstants.getSim());
                vision = new Vision(
                    drive::addVisionMeasurement,
                    () -> drive.getTimestampedHeading(),
                    new VisionIOPhotonVisionSim(
                        () -> drive.getPose(),
                        VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0,
                        VisionConstants.aprilTagLayout,
                        PoseStrategy.CONSTRAINED_SOLVEPNP));
                tounge = new Tounge(ToungeConstants.getSim());
            }

            default -> {
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});

                leds = new LEDs(LEDsConstants.getLightsIOReplay());
                laserCAN1 =
                    new ClawRollerLaserCAN(ClawRollerLaserCANConstants.getReplay());
                flywheel = new ClawRoller(ClawRollerConstants.getReplay());

                linear = new Elevator(ElevatorConstants.getReplay());
                tounge = new Tounge(ToungeConstants.getReplay());
                vision = new Vision(
                    drive::addVisionMeasurement,
                    () -> drive.getTimestampedHeading(),
                    new VisionIO() {});
            }
        }

        conditionalChooser = new LoggedDashboardChooser<>("Conditional Choice");
        conditionalChooser.addOption("True", true);
        conditionalChooser.addOption("False", false);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        autoChooser.addDefaultOption("None", new NoneAuto());
        autoChooser.addOption("ExampleAuto", new ExampleAuto(drive));
        autoChooser.addOption("BranchingAuto",
            new BranchingAuto(drive, () -> conditionalChooser.get()));

        autoChooser.onChange(auto -> {
            autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
        });

        autoChooser.addOption("Drive Wheel Radius Characterization",
            new WheelCharacterizationAuto(drive));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        controller
            .a()
            .whileTrue(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                    () -> drive.setPose(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                    .ignoringDisable(true));

        // Pathfind to Pose when the Y button is pressed
        controller.y().onTrue(
            DriveCommands.pathFindToPose(() -> drive.getPose(), new Pose2d(3, 3, Rotation2d.kZero),
                PathConstants.ON_THE_FLY_PATH_CONSTRAINTS, 0.0,
                PathConstants.PATHGENERATION_DRIVE_TOLERANCE));

        // On-the-fly path with waypoints while the Right Bumper is held
        controller.rightBumper().whileTrue(
            new OnTheFlyPathCommand(drive, () -> drive.getPose(), new ArrayList<>(Arrays.asList()), // List
                                                                                                    // of
                                                                                                    // waypoints
                new Pose2d(6, 6, Rotation2d.k180deg), PathConstants.ON_THE_FLY_PATH_CONSTRAINTS,
                0.0, false, PathConstants.PATHGENERATION_DRIVE_TOLERANCE,
                PathConstants.PATHGENERATION_ROT_TOLERANCE_DEGREES));

        SmartDashboard.putData("Linear: Stow", linear.goToSetpoint(Elevator.Setpoint.STOW));
        SmartDashboard.putData("Linear: Raised", linear.goToSetpoint(Elevator.Setpoint.RAISED));
        SmartDashboard.putData("Linear: Home", linear.homeCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoChooser.get();
    }
}
