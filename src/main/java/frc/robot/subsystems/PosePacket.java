package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

import java.nio.ByteBuffer;
import java.time.Instant;

public record PosePacket(Rotation3d rotation, Vector<N3> position, Instant time) {
    //                         Timestamp  | Position Vector  | Rotation Matrix
    public static int BYTES = Long.BYTES + 3 * Double.BYTES + 3 * 3 * Double.BYTES;

    public static PosePacket fromBuf(ByteBuffer buf) {
        long milliTimestamp = buf.getLong();
        Instant time = Instant.ofEpochMilli(milliTimestamp);

        Vector<N3> position = new Vector<>(Nat.N3());
        for(int i=0;i<position.getNumRows();i++) {
            position.set(i,0,buf.getDouble());
        }

        Matrix<N3,N3> rotation = new Matrix<>(Nat.N3(),Nat.N3());
        for(int i=0;i<rotation.getNumRows();i++) {
            for(int j=0;j<rotation.getNumCols();j++) {
                rotation.set(i,j,buf.getDouble());
            }
        }

        return new PosePacket(new Rotation3d(rotation),position,time);
    }
}
