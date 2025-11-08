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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.commands.DriveToPoseBase;
import frc.lib.commands.AlignToPoseBase.AlignMode;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOPhotonVision;
import frc.lib.io.vision.VisionIOPhotonVisionSim;
import frc.lib.util.LoggedDashboardChooser;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoggedTuneableProfiledPID;
import frc.lib.util.AutoCommand;
import frc.lib.util.CommandXboxControllerExtended;
import frc.lib.util.GamePieceVisualizer;
import frc.robot.Constants.PathConstants;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.OnTheFlyPathCommand;
import frc.robot.commands.PointTo2DTarget;
import frc.robot.commands.autos.NoneAuto;
import frc.robot.commands.autos.WheelCharacterizationAuto;
import frc.robot.commands.autos.WheelSlipAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.linear.Linear;
import frc.robot.subsystems.linear.LinearConstants;
import frc.robot.subsystems.rotary.Rotary;
import frc.robot.subsystems.rotary.RotaryConstants;
import frc.robot.subsystems.rotary.Rotary.Setpoint;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.objectDetector.ObjectDetector;
import frc.robot.subsystems.objectDetector.ObjectDetectorConstants;
import frc.robot.subsystems.lasercan1.LaserCAN1;
import frc.robot.subsystems.lasercan1.LaserCAN1Constants;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

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
    private final LaserCAN1 laserCAN1;
    private final Linear linear;
    private final Vision vision;
    private final Rotary rotary;
    private final ObjectDetector objectDetector;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoCommand> autoChooser;
    public static Field2d autoPreviewField = new Field2d();



    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer()
    {
        drive = DriveConstants.get();
        laserCAN1 = LaserCAN1Constants.get();
        linear = LinearConstants.get();
        rotary = RotaryConstants.get();
        vision = VisionConstants.get(drive); // TODO: Will be refactored in the future to RobotState
        objectDetector = ObjectDetectorConstants.get(drive);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        autoChooser.addDefaultOption("None", new NoneAuto());

        autoChooser.addOption("Drive Wheel Radius Characterization",
            new WheelCharacterizationAuto(drive));

        autoChooser.addOption("Wheel Slip Characterization", new WheelSlipAuto(drive));

        // Configure the button bindings
        configureButtonBindings();
    }

    private AlignToPose joystickApproach(Supplier<Pose2d> approachPose)
    {
        return new AlignToPose(
            drive,
            approachPose,
            AlignMode.APPROACH,
            () -> controller.getLeftY());
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

        // Driver Right Bumper and Coral Mode: Approach Nearest Right-Side Reef Branch
        controller
            .rightBumper()
            .whileTrue(joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));

        // Driver Left Bumper and Coral Mode: Approach Nearest Left-Side Reef Branch
        controller
            .leftBumper()
            .whileTrue(joystickApproach(
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));

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
