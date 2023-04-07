package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.BalanceRoutine;
import frc.robot.auto.MobilityRoutine;
import frc.robot.claw.ClawConstants;
import frc.robot.claw.ClawSubsystem;
import frc.robot.claw.commands.SetClawAngleCommand;
import frc.robot.claw.commands.ShootCommand;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.SetElevatorCommand;
import frc.robot.elevator.commands.ZeroElevatorCommand;
import frc.robot.routines.AutoShootRoutine;
import frc.robot.routines.TagAlignments;

public class Robot extends TimedRobot {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final IntegerSubscriber tidL = NetworkTableInstance.getDefault().getTable("limelight-tags").getIntegerTopic("tid").subscribe(-1);
  private final IntegerSubscriber tidR = NetworkTableInstance.getDefault().getTable("limelight-pieces").getIntegerTopic("tid").subscribe(-1);

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    CameraServer.addCamera(new HttpCamera("limelightL", "http://10.80.80.11:5800"));
    CameraServer.addCamera(new HttpCamera("limelightR", "http://10.80.80.12:5800"));
    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog(), false);
    DriverStation.silenceJoystickConnectionWarning(true);

    SmartDashboard.putStringArray("Auto List", AutoConstants.ROUTINES);

    PortForwarder.add(5800, "10.80.80.11", 5800);
    PortForwarder.add(5801, "10.80.80.11", 5801);
    PortForwarder.add(5802, "10.80.80.12", 5800);
    PortForwarder.add(5803, "10.80.80.12", 5801);

    driveSubsystem.reset();
    driveSubsystem.zero();

    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Gyro", driveSubsystem.getGyro().getDegrees());
    SmartDashboard.putString("DB/String 0", "OX: " + (int)(driveSubsystem.getOdometryPose().getX() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 1", "OY: " + (int)(driveSubsystem.getOdometryPose().getY() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 2", "Y: " + (int)(driveSubsystem.getGyro().getDegrees() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 3", "P: " + (int)(driveSubsystem.getPitch() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 4", "R: " + (int)(driveSubsystem.getRoll() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 5", "TID: " + getPrimaryApriltag().map(x -> "" + x).orElse("N/A"));
    SmartDashboard.putString("DB/String 6", "CA: " + (int)(clawSubsystem.getAngle() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 7", "EP: " + (int)(elevatorSubsystem.getMotorPosition() * 100.0) / 100.0);

    SmartDashboard.putBoolean("DB/LED 0", driveSubsystem.getCurrentCommand() != null && !(driveSubsystem.getCurrentCommand() instanceof DriveCommand));
    SmartDashboard.putBoolean("DB/LED 1", isInUse(clawSubsystem));
    SmartDashboard.putBoolean("DB/LED 2", isInUse(clawSubsystem));
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().enable();
    driveSubsystem.reset();
    Command autoCommand;
    switch (SmartDashboard.getString("Auto Selector", AutoConstants.ROUTINE_NOTHING)) {
      case AutoConstants.ROUTINE_MOBILITY:
        autoCommand = new MobilityRoutine(driveSubsystem, false);
        break;
      case AutoConstants.ROUTINE_BALANCE:
        autoCommand = new BalanceRoutine(driveSubsystem, false);
        break;
      case AutoConstants.ROUTINE_HIGH_CUBE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.HIGH_CUBE.toCommand(elevatorSubsystem, clawSubsystem);
        break;
      case AutoConstants.ROUTINE_MID_CUBE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.MID_CUBE.toCommand(elevatorSubsystem, clawSubsystem);
        break;
      case AutoConstants.ROUTINE_HIGH_CUBE_BALANCE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.HIGH_CUBE.toCommand(elevatorSubsystem, clawSubsystem)
          .andThen(new BalanceRoutine(driveSubsystem, false));
        break;
      case AutoConstants.ROUTINE_MID_CUBE_BALANCE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.MID_CUBE.toCommand(elevatorSubsystem, clawSubsystem)
          .andThen(new BalanceRoutine(driveSubsystem, false));
        break;
      case AutoConstants.ROUTINE_HIGH_CUBE_MOBILITY_SHORT_SIDE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.HIGH_CUBE.toCommand(elevatorSubsystem, clawSubsystem)
          .andThen(TagAlignments.SHORT_SIDE.toCommand(driveSubsystem))
          .andThen(new MobilityRoutine(driveSubsystem, false));
        break;
      case AutoConstants.ROUTINE_MID_CUBE_MOBILITY_SHORT_SIDE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.MID_CUBE.toCommand(elevatorSubsystem, clawSubsystem)
          .andThen(TagAlignments.SHORT_SIDE.toCommand(driveSubsystem))
          .andThen(new MobilityRoutine(driveSubsystem, false));
        break;
      case AutoConstants.ROUTINE_HIGH_CUBE_MOBILITY_LONG_SIDE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.HIGH_CUBE.toCommand(elevatorSubsystem, clawSubsystem)
          .andThen(TagAlignments.LONG_SIDE.toCommand(driveSubsystem))
          .andThen(new MobilityRoutine(driveSubsystem, false));
        break;
      case AutoConstants.ROUTINE_MID_CUBE_MOBILITY_LONG_SIDE:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = AutoShootRoutine.MID_CUBE.toCommand(elevatorSubsystem, clawSubsystem)
          .andThen(TagAlignments.LONG_SIDE.toCommand(driveSubsystem))
          .andThen(new MobilityRoutine(driveSubsystem, false));
        break;
      default:
        autoCommand = new InstantCommand();
        break;
    }
    CommandScheduler.getInstance().schedule(new WaitCommand(0.5).andThen(() -> {
      CommandScheduler.getInstance().schedule(autoCommand);
    }));
  }

  boolean lastUp = false, lastDown = false;

  @Override
  public void teleopPeriodic() {
    // Elevator
    if (!isInUse(elevatorSubsystem)) {
      if (Controls.getDriver2ManualElevatorUp() && !Controls.getDriver2ManualElevatorDown()) {
        elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_UP_SPEED);
      } else if (Controls.getDriver2ManualElevatorDown() && !Controls.getDriver2ManualElevatorUp()) {
        elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_DOWN_SPEED);
      } else {
        elevatorSubsystem.stop();
      }
    }

    // Claw
    if (!isInUse(clawSubsystem)) {
      if (Controls.getDriver2ManualClawUp() && !Controls.getDriver2ManualClawDown()) {
        clawSubsystem.setWrist(ClawConstants.WRIST_MANUAL_UP_SPEED);
      } else if (Controls.getDriver2ManualClawDown() && !Controls.getDriver2ManualClawUp()) {
        clawSubsystem.setWrist(ClawConstants.WRIST_MANUAL_DOWN_SPEED);
      } else {
        clawSubsystem.setWrist(0);
      }

      if (Controls.getDriver2Intake()) {
        clawSubsystem.setIntake(ClawConstants.INTAKE_SPEED);
      } else if (Controls.getDriver2Outtake()) {
        clawSubsystem.setIntake(-ClawConstants.INTAKE_SPEED); 
      } else {
        clawSubsystem.setIntake(0);
      }

      if (Controls.INSTANCE.driver2.getRawButtonPressed(1)) {
        CommandScheduler.getInstance().schedule(new ShootCommand(-0.3, clawSubsystem));
      }

      if (Controls.getDriver2ManualShootMid()) {
        CommandScheduler.getInstance().schedule(new ShootCommand(-0.165, clawSubsystem));
      }

      if (Controls.getDriver2ManualShootHigh()) {
        CommandScheduler.getInstance().schedule(new ShootCommand(-0.25, clawSubsystem));
      }
    }

    // Special Routines
    if (!isInUse(clawSubsystem) && !isInUse(elevatorSubsystem)) {
      if (Controls.getDriver2HighCube()) {
        CommandScheduler.getInstance().schedule(TagAlignments.CUBE.toCommand(driveSubsystem)
          .andThen(new WaitCommand(0.1)
          .andThen(AutoShootRoutine.HIGH_CUBE.toCommand(elevatorSubsystem, clawSubsystem))));
      }

      if (Controls.getDriver2MidCube()) {
        CommandScheduler.getInstance().schedule(TagAlignments.CUBE.toCommand(driveSubsystem)
          .andThen(new WaitCommand(0.1)
          .andThen(AutoShootRoutine.MID_CUBE.toCommand(elevatorSubsystem, clawSubsystem))));
      }

      if (Controls.getDriver2HighCone()) {
        CommandScheduler.getInstance().schedule(TagAlignments.CONE.toCommand(driveSubsystem)
        .andThen(new WaitCommand(0.1))
        .andThen(AutoShootRoutine.HIGH_CONE.toCommand(elevatorSubsystem, clawSubsystem)));
      }

      if (Controls.getDriver2MidCone()) {
        CommandScheduler.getInstance().schedule(TagAlignments.CONE.toCommand(driveSubsystem)
        .andThen(new WaitCommand(0.1))
        .andThen(AutoShootRoutine.MID_CONE.toCommand(elevatorSubsystem, clawSubsystem)));
      }

      if (Controls.getDriver2ToSubStationHeight()) {
        CommandScheduler.getInstance().schedule(new SetElevatorCommand(ElevatorConstants.SUBSTATION_HEIGHT, elevatorSubsystem)
          .alongWith(new SetClawAngleCommand(15.93, clawSubsystem)));
      }
    }

    // Cancel
    if (Controls.getDriver1Cancel() || Controls.getDriver2Cancel()) {
      CommandScheduler.getInstance().cancelAll();
      elevatorSubsystem.stop();
      clawSubsystem.stop();
    }
  }

  @Override
  public void teleopExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
      CommandScheduler.getInstance().enable();
      CommandScheduler.getInstance().schedule(new ZeroElevatorCommand(elevatorSubsystem));
      driveSubsystem.reset();
      if (SmartDashboard.getBoolean("DB/Button 0", false)) {
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
      }
  }

  private boolean isInUse(SubsystemBase subsystem) {
    return subsystem.getCurrentCommand() != null;
  }

  private Optional<Integer> getPrimaryApriltag() {
    int l = (int)tidL.get();
    int r = (int)tidR.get();
    if (l >= 1 && l <= 8) {
      return Optional.of(l);
    }
    if (r >= 1 && r <= 8) {
      return Optional.of(r);
    }
    return Optional.empty();
  }
}