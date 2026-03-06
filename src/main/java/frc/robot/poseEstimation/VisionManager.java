package frc.robot.poseEstimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.LinkedList;
import java.util.stream.Stream;

public class VisionManager {
    private final ServerSocketChannel channel;
    private final LinkedList<BufferedFixedLengthChannel> channels = new LinkedList<>();
    private static final int PORT = 8080;
    private final MecanumDrivePoseEstimator3d poseEstimator3d;

    public VisionManager(MecanumDrivePoseEstimator3d poseEstimator3d) {
        this.poseEstimator3d = poseEstimator3d;
        try {
            this.channel = ServerSocketChannel.open();
            this.channel.configureBlocking(false);
            this.channel.bind(new InetSocketAddress(PORT));

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void acceptNewConnections() {
        SocketChannel channel;
        try {
            while ((channel = this.channel.accept()) != null) {
                channel.configureBlocking(false);
                this.channels.add(
                    new BufferedFixedLengthChannel(
                        channel,
                        PosePacket.BYTES
                    )
                );
                System.out.println("added new channel");
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private Stream<PosePacket> pollForPose(BufferedFixedLengthChannel channel) {
        try {
            return channel.pollFullBuffers().map(PosePacket::fromBuf);
        } catch (Exception e) {
            System.out.println("Error reading vision packets: ");
            e.printStackTrace();
            return Stream.empty();
        }
        
    }

    private Stream<PosePacket> poll() {
        acceptNewConnections();
        Stream<PosePacket> packetStream = Stream.empty();
        for(BufferedFixedLengthChannel channel : channels) {
            if(!channel.isOpen()) {
                System.out.println("channel closed");
            }
            packetStream = Stream.concat(packetStream,pollForPose(channel));
        }
        return packetStream;
    }

    private static final double TRANSLATION_ERR_CONSTANT = 0.05;
    private static final double ROTATION_ERR_CONSTANT = 0.05;

    private double firstVisionTimestampMyTime = -1;
    private double firstVisionTimestampPITime = -1;
    private long lastVisionPoseUpdatePITime = -1;

    public void update() {
        poll().forEach((posePacket) -> {
            if(firstVisionTimestampMyTime == -1) {
                firstVisionTimestampMyTime = Timer.getFPGATimestamp();
                firstVisionTimestampPITime = posePacket.time().toEpochMilli() / 1000.0;
            }
            poseEstimator3d.addVisionMeasurement(
                posePacket.pose(),
                posePacket.time().toEpochMilli() / 1000.0 - firstVisionTimestampPITime + firstVisionTimestampMyTime,
                VecBuilder.fill(
                    posePacket.translationErr() * TRANSLATION_ERR_CONSTANT,
                    posePacket.translationErr() * TRANSLATION_ERR_CONSTANT,
                    posePacket.translationErr() * TRANSLATION_ERR_CONSTANT,
                    ROTATION_ERR_CONSTANT
                )
            );
            this.lastVisionPoseUpdatePITime = posePacket.time().toEpochMilli();
        }
        );

        SmartDashboard.putNumber("LastVisionUpdateTimeStamp", lastVisionPoseUpdatePITime);
    }
}
