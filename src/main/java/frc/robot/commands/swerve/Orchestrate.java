package frc.robot.commands.swerve;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Orchestrate extends Command{
    public CommandSwerveDrivetrain drivetrain;
    public Orchestra orchestrate;
    public String midiString;

    public Orchestrate(CommandSwerveDrivetrain drivetrain, Orchestra orchestrate, String midiString){
        this.drivetrain = drivetrain;
        this.orchestrate = orchestrate;
        this.midiString = midiString;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        orchestrate.loadMusic(midiString);
        orchestrate.addInstrument(drivetrain.getModule(0).getDriveMotor(), 1);
        orchestrate.addInstrument(drivetrain.getModule(1).getDriveMotor(), 2);
        orchestrate.addInstrument(drivetrain.getModule(2).getDriveMotor(), 3);
        orchestrate.addInstrument(drivetrain.getModule(3).getDriveMotor(), 4);

        orchestrate.addInstrument(drivetrain.getModule(0).getSteerMotor(), 5);
        orchestrate.addInstrument(drivetrain.getModule(1).getSteerMotor(), 6);
        orchestrate.addInstrument(drivetrain.getModule(2).getSteerMotor(), 7);
        orchestrate.addInstrument(drivetrain.getModule(3).getSteerMotor(), 8);

        orchestrate.play();
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted){
            orchestrate.stop();
        }
    }

    @Override
    public boolean isFinished(){
        return !orchestrate.isPlaying();
    }
}
