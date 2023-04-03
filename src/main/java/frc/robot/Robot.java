package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.Balance2Routine;
import frc.robot.auto.BalanceRoutine;
import frc.robot.auto.HighCubeRoutine;
import frc.robot.auto.MobilityRoutine;
import frc.robot.claw.ClawConstants;
import frc.robot.claw.ClawSubsystem;
import frc.robot.claw.commands.SetClawAngleCommand;
import frc.robot.claw.commands.ShootCommand;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.commands.AlignToTXCommand;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorHeight;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.SetElevatorCommand;

public class Robot extends TimedRobot {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog(), false);
    DriverStation.silenceJoystickConnectionWarning(true);

    SmartDashboard.putStringArray("Auto List", AutoConstants.ROUTINES);

    driveSubsystem.reset();
    driveSubsystem.zero();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Gyro", driveSubsystem.getGyro().getDegrees());
    SmartDashboard.putString("DB/String 0", "OX: " + (int)(driveSubsystem.getPose().getX() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 1", "OY: " + (int)(driveSubsystem.getPose().getY() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 2", "Y: " + (int)(driveSubsystem.getGyro().getDegrees() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 3", "P: " + (int)(driveSubsystem.getPitch() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 4", "R: " + (int)(driveSubsystem.getRoll() * 100.0) / 100.0);
    SmartDashboard.putString("DB/String 5", "TID: " + driveSubsystem.getPrimaryApriltag().map(x -> "" + x).orElse("N/A"));
  }

  @Override
  public void teleopInit() {
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().enable();
    driveSubsystem.zero();
    Command autoCommand;
    switch (SmartDashboard.getString("Auto Selector", AutoConstants.ROUTINE_NOTHING)) {
      case AutoConstants.ROUTINE_MOBILITY:
        autoCommand = new MobilityRoutine(driveSubsystem, false);
        break;
      case AutoConstants.ROUTINE_BALANCE:
        autoCommand = new BalanceRoutine(driveSubsystem, false);
        break;
      case AutoConstants.ROUTINE_BALANCE_INVERTED:
        driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = new BalanceRoutine(driveSubsystem, false);
        break;
      case AutoConstants.ROUTINE_BALANCE2:
        autoCommand = new Balance2Routine(driveSubsystem);
        break;
      case AutoConstants.ROUTINE_DUMP:
      driveSubsystem.calibrateGyro(Rotation2d.fromDegrees(180));
        autoCommand = new HighCubeRoutine(elevatorSubsystem, clawSubsystem)
        .andThen(new ScheduleCommand(new MobilityRoutine(driveSubsystem, true)));
        break;
      default:
        autoCommand = new InstantCommand();
        break;
    }
    CommandScheduler.getInstance().schedule(new WaitCommand(1).andThen(() -> {
      CommandScheduler.getInstance().schedule(autoCommand);
    }));
  }

  boolean lastUp = false, lastDown = false;

  @Override
  public void teleopPeriodic() {
    // Elevator
    if (elevatorSubsystem.getCurrentCommand() == null) {
      if (Controls.getDriver2ManualElevatorUp() && !Controls.getDriver2ManualElevatorDown()) {
        elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_UP_SPEED);
      } else if (Controls.getDriver2ManualElevatorDown() && !Controls.getDriver2ManualElevatorUp()) {
        elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_DOWN_SPEED);
      } else {
        elevatorSubsystem.stop();
      }

      if (Controls.INSTANCE.driver2.getRawButtonPressed(11)) {
        CommandScheduler.getInstance().schedule(new SetElevatorCommand(40.08, elevatorSubsystem));
      }
      if (Controls.INSTANCE.driver2.getRawButtonPressed(12)) {
        CommandScheduler.getInstance().schedule(new SetClawAngleCommand(45, clawSubsystem));
      }
    }

    if (Controls.INSTANCE.driver2.getRawButtonPressed(10)) {
      CommandScheduler.getInstance().schedule(new AlignToTXCommand(7, driveSubsystem)
      .andThen(new SetElevatorCommand(20, elevatorSubsystem))
      .andThen(new WaitCommand(1))
      .andThen(new SetClawAngleCommand(57.86, clawSubsystem))
      .andThen(new WaitCommand(1))
      .andThen(new ShootCommand(-0.4, clawSubsystem)));
    }

    if (Controls.INSTANCE.driver2.getRawButtonPressed(7)) {
      CommandScheduler.getInstance().schedule(new ShootCommand(-0.225, clawSubsystem));
    }
    if (Controls.INSTANCE.driver2.getRawButtonPressed(8)) {
      CommandScheduler.getInstance().schedule(new ShootCommand(-0.4, clawSubsystem));
    }

    // Claw
    if (clawSubsystem.getCurrentCommand() == null) {
      if (Controls.getDriver2ManualClawUp() && !Controls.getDriver2ManualClawDown()) {
        clawSubsystem.setWrist(ClawConstants.WRIST_MANUAL_UP_SPEED);
      } else if (Controls.getDriver2ManualClawDown() && !Controls.getDriver2ManualClawUp()) {
        clawSubsystem.setWrist(ClawConstants.WRIST_MANUAL_DOWN_SPEED);
      } else {
        clawSubsystem.setWrist(0);
      }

      if (Controls.getDriver2Shoot()) {
        CommandScheduler.getInstance().schedule(new ShootCommand(-1, clawSubsystem));
      }

      if (Controls.getDriver2Intake()) {
        clawSubsystem.setIntake(ClawConstants.INTAKE_SPEED);
      } else {
        clawSubsystem.setIntake(0);
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
    driveSubsystem.removeDefaultCommand();
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
      driveSubsystem.reset();
  }
}