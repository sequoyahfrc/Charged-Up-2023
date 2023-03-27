package frc.robot.elevator;

public enum ElevatorHeight {
    FLOOR(0),
    MIDDLE(12.5),
    TOP(25);

    private static final ElevatorHeight[] VALUES = ElevatorHeight.values();
    private final double position;

    private ElevatorHeight(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }

    public ElevatorHeight next() {
        return VALUES[Math.min(ordinal() + 1, VALUES.length - 1)];
    }

    public ElevatorHeight prev() {
        return VALUES[Math.max(ordinal() - 1, 0)];
    }
}
