package frc.robot.poseEstimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N4;

import java.nio.ByteBuffer;
import java.time.Instant;

/// As a buffer,
/// a pose packet consists of
///  - a 4x4 matrix written in left to right top to bottom order
///  - then the translational error as a double
///  - the rotational error in radians as a double
///  - the current time as a unix timestamp in milliseconds (milliseconds since 1970/01/01)
///
/// NOTE: time may not actually be calibrated on the RoboRIO??? I mean I don't know how it would sync???
/// If so, a shared other timestamp could be found and used
public record PosePacket(Pose3d pose, double translationErr, double rotationErr, Instant time) {
    //                   Rotation Quaternion| Position Vector       | Time of measurement
    public static int BYTES = Long.BYTES + 3 * Double.BYTES + 3 * 3 * Double.BYTES;

    public static PosePacket fromBuf(ByteBuffer buf) {
        double[] matrixValues = new double[4 * 4];
        // reading order (left to right then top to bottom)
        for(int i=0;i<4;i++) {
            for(int j=0;j<4;j++) {
                matrixValues[j * 4 + i] = buf.getDouble();
            }
        }
        Matrix<N4,N4> matrix = new Matrix<>(Nat.N4(), Nat.N4(), matrixValues);

        long milliTimestamp = buf.getLong();
        Instant time = Instant.ofEpochMilli(milliTimestamp);

        double translationErr = buf.getDouble();
        double rotationErr = buf.getDouble();

        return new PosePacket(new Pose3d(matrix),translationErr,rotationErr,time);
    }
}
