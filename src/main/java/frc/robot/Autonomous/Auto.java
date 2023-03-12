package frc.robot.Autonomous;

/*
    Class to store autonomous sequences, including sequences such as intake
*/

public class Auto {
    public enum Selection {
        MOVEARM(1), MOVEWRIST(2), OTHER(3);
        public int val;
        private Selection(int val) {
            this.val = val;
        }
    }

    public Selection selection;

    public Auto(Selection selection) {
        this.selection = selection;
    }
}