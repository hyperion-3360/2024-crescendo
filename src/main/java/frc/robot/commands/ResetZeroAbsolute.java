// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.Swerve;

// public class ResetZeroAbsolute extends Command {
//   private Swerve s_Swerve;
//   private boolean m_done;

//   public ResetZeroAbsolute(Swerve s_Swerve) {
//     this.s_Swerve = s_Swerve;
//     addRequirements(s_Swerve);
//     m_done = true;
//   }

//   @Override
//   public void execute() {
//     /* Drive */
//     m_done = false;
//     System.out.println("Rsetting to zero value");
//     s_Swerve.resetModulesToAbsolute();
//     m_done = true;
//   }

//   @Override
//   public boolean isFinished() {
//     return m_done;
//   }
// }
