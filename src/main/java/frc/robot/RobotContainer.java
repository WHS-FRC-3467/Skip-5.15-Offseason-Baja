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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.clawroller.ClawRoller;
import frc.robot.subsystems.clawroller.ClawRollerConstants;
import frc.robot.subsystems.clawrollerlasercan.ClawRollerLaserCAN;
import frc.robot.subsystems.clawrollerlasercan.ClawRollerLaserCANConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
    private final Drive drive;
    private final Elevator elevator;
    private final Arm arm;
    private final ClawRoller clawroller;
    private final ClawRollerLaserCAN clawLaserCAN;
    private final Tounge tounge;
    private final Climber climber;
    private final LEDs leds;
    private final Vision vision;

    // Controller
    private final CommandXboxControllerExtended controller = new CommandXboxControllerExtended(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoCommand> autoChooser;
    public static Field2d autoPreviewField = new Field2d();

    // Trigger for algae/coral mode switching
    @AutoLogOutput
    private Trigger isCoralMode;

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
                elevator = new Elevator(ElevatorConstants.getReal());
                arm = new Arm(ArmConstants.getReal());
                clawroller = new ClawRoller(ClawRollerConstants.getReal());
                clawLaserCAN = new ClawRollerLaserCAN(ClawRollerLaserCANConstants.getReal());
                tounge = new Tounge(ToungeConstants.getReal());
                climber = new Climber(ClimberConstants.getReal());
                leds = new LEDs(LEDsConstants.getReal());
                vision = new Vision(
                    drive::addVisionMeasurement,
                    () -> drive.getTimestampedHeading(),
                    new VisionIOPhotonVision(
                        VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0,
                        VisionConstants.aprilTagLayout,
                        PoseStrategy.CONSTRAINED_SOLVEPNP),
                    new VisionIOPhotonVision(
                        VisionConstants.camera1Name,
                        VisionConstants.robotToCamera1,
                        VisionConstants.aprilTagLayout,
                        PoseStrategy.CONSTRAINED_SOLVEPNP));
            }

            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(DriveConstants.FrontLeft),
                    new ModuleIOSim(DriveConstants.FrontRight),
                    new ModuleIOSim(DriveConstants.BackLeft),
                    new ModuleIOSim(DriveConstants.BackRight));
                elevator = new Elevator(ElevatorConstants.getSim());
                arm = new Arm(ArmConstants.getSim());
                clawroller = new ClawRoller(ClawRollerConstants.getSim());
                clawLaserCAN = new ClawRollerLaserCAN(ClawRollerLaserCANConstants.getSim());
                tounge = new Tounge(ToungeConstants.getSim());
                climber = new Climber(ClimberConstants.getSim());
                leds = new LEDs(LEDsConstants.getSim());
                vision = new Vision(
                    drive::addVisionMeasurement,
                    () -> drive.getTimestampedHeading(),
                    new VisionIOPhotonVisionSim(
                        () -> drive.getPose(),
                        VisionConstants.camera0Name,
                        VisionConstants.robotToCamera0,
                        VisionConstants.aprilTagLayout,
                        PoseStrategy.CONSTRAINED_SOLVEPNP),
                    new VisionIOPhotonVisionSim(
                        () -> drive.getPose(),
                        VisionConstants.camera1Name,
                        VisionConstants.robotToCamera1,
                        VisionConstants.aprilTagLayout,
                        PoseStrategy.CONSTRAINED_SOLVEPNP));
            }

            default -> {
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
                elevator = new Elevator(ElevatorConstants.getReplay());
                arm = new Arm(ArmConstants.getReplay());
                clawroller = new ClawRoller(ClawRollerConstants.getReplay());
                clawLaserCAN = new ClawRollerLaserCAN(ClawRollerLaserCANConstants.getReplay());
                tounge = new Tounge(ToungeConstants.getReplay());
                climber = new Climber(ClimberConstants.getReplay());
                leds = new LEDs(LEDsConstants.getReplay());
                vision = new Vision(
                    drive::addVisionMeasurement,
                    () -> drive.getTimestampedHeading(),
                    new VisionIO() {},
                    new VisionIO() {});
            }
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        SmartDashboard.putData("Auto Preview", autoPreviewField);

        autoChooser.addDefaultOption("None", new NoneAuto());
        autoChooser.addOption("ExampleAuto", new ExampleAuto(drive));

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
        drive.setDefaultCommand(joystickDrive());

        // Align Right
        controller
            .rightBumper()
            .and(isCoralMode)
            .whileTrue(null);

        // Align left
        controller
            .leftBumper()
            .and(isCoralMode)
            .whileTrue(null);

        // Descore Algae
        controller
            .leftBumper()
            .and(controller.rightBumper())
            .and(isCoralMode.negate())
            .whileTrue(null);

        // Score L1 left
        controller
            .leftBumper()
            .and(controller.a())
            .whileTrue(null);

        // Score L1 right
        controller
            .rightBumper()
            .and(controller.a())
            .whileTrue(null);

        // Prep for L1 Score, or ground algae intake
        controller
            .a()
            .onTrue(null)
            .whileTrue(null);

        // Algae Descore to Lower Claw - Processor
        controller
            .start()
            .whileTrue(null);

        // L2 Coral
        controller
            .x()
            .and(isCoralMode)
            .onTrue(
                superStructureCommand(
                    Arm.Setpoint.LEVEL_2,
                    Elevator.Setpoint.LEVEL_2));

        // Algae Lolipop Collect
        controller
            .x()
            .and(isCoralMode.negate())
            .onTrue(
                Commands.sequence(
                    superStructureCommand(
                        Arm.Setpoint.PROCESSOR_SCORE,
                        Elevator.Setpoint.ALGAE_LOLLIPOP),
                    clawroller.algaeForward(),
                    Commands.waitUntil(clawroller.stalled),
                    superStructureCommand(
                        Arm.Setpoint.STOW,
                        Elevator.Setpoint.STOW)))
            .whileTrue(null); // Face driverstation

        // L3 Coral
        controller
            .b()
            .and(isCoralMode)
            .onTrue(
                superStructureCommand(
                    Arm.Setpoint.LEVEL_3,
                    Elevator.Setpoint.LEVEL_3));

        // Processor Score
        controller
            .b()
            .and(isCoralMode.negate())
            .onTrue(
                superStructureCommand(
                    Arm.Setpoint.PROCESSOR_SCORE,
                    Elevator.Setpoint.STOW))
            .whileTrue(null); // Align to processor

        // L4 Coral
        controller
            .y()
            .and(isCoralMode)
            .onTrue(
                superStructureCommand(
                    Arm.Setpoint.LEVEL_4,
                    Elevator.Setpoint.LEVEL_4));

        // Algae Barge
        controller
            .y()
            .and(isCoralMode.negate())
            .onTrue(null);

        // Score Coral or Algae
        controller
            .rightTrigger()
            .and(controller.a().negate())
            .onTrue(
                Commands.either(
                    Commands.sequence(
                        clawroller.score(),
                        Commands.waitUntil(clawLaserCAN.triggered.negate()),
                        Commands.waitSeconds(0.2),
                        clawroller.stop(),
                        superStructureCommand(
                            Arm.Setpoint.STOW,
                            Elevator.Setpoint.STOW)),

                    Commands.sequence(null),

                    isCoralMode));

        // Coral Intake
        controller
            .leftTrigger()
            .whileTrue(null)
            .onFalse(null);

        // Climb Sequence
        controller
            .back()
            .onTrue(null);

        // Elevator Stow Override
        controller
            .povLeft()
            .onTrue(
                Commands.sequence(
                    elevator.setSetpoint(Elevator.Setpoint.STOW),
                    tounge.setSetpoint(Tounge.Setpoint.DOWN),
                    clawroller.score()))
            .onFalse(
                Commands.parallel(
                    clawroller.stop(),
                    tounge.setSetpoint(Tounge.Setpoint.STOW)));

        // Climber Sequence Reset
        controller
            .povRight()
            .onTrue(null);

        // Unjam
        controller
            .povUp()
            .onTrue(
                Commands.parallel(
                    arm.setSetpoint(Arm.Setpoint.LEVEL_2),
                    elevator.setSetpoint(Elevator.Setpoint.LEVEL_3)));

        // Elevator Homing
        controller.povDown()
            .onTrue(
                Commands.sequence(
                    arm.setpointCommandWithWait(Arm.Setpoint.STOW),
                    elevator.homeCommand()));
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

    public Command superStructureCommand(Arm.Setpoint armSetpoint,
        Elevator.Setpoint elevatorSetpoint)
    {
        return Commands.sequence(
            // Always move Arm to STOW position before moving Elevator
            arm.setpointCommandWithWait(Arm.Setpoint.STOW),
            // Move Elevator to new position
            elevator.setpointCommandWithWait(elevatorSetpoint),
            // Reposition Arm to new position
            arm.setpointCommandWithWait(armSetpoint));
    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX());
    }

    // private JoystickApproachCommand joystickApproach(Supplier<Pose2d> approachPose)
    // {
    // return new JoystickApproachCommand(
    // m_drive,
    // () -> m_driver.getLeftY(),
    // approachPose);
    // }
}
