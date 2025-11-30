package frc.robot.subsystems.utils;

public enum armPositions {
    
    DOWN(6.0),
    MEDIUM(12.0),
    HIGH(24.0);

    private final double position;

    armPositions(double position) {
        this.position = position;
    }

    public double getPosition() {
        return position;
    }
}
