package frc.robot.poseEstimation;

public class NullTerminatedPacketChannel {
    private ByteBuffer incompleteBuffer;
    private final SocketChannel channel;
    private final int packetLength;

    public NullTerminatedPacketChannel(SocketChannel channel) {
        this.channel = channel;
    }

    public Stream<ByteBuffer> pollFullBuffers() {
        return Stream.iterate(
            ByteBuffer.allocate(1024),
            Objects::nonNull,
            (lastBuf) -> this.pollBuffer()
        );
    }

    public ByteBuffer pollBuffer() {
        if(this.incompleteBuffer == null) {
            this.incompleteBuffer = ByteBuffer.allocate(1024);
        }

        try {
            this.channel.read(incompleteBuffer);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        if(incompleteBuffer.position() < this.packetLength) {
            return null;
        }

        this.incompleteBuffer.flip();
        ByteBuffer justGoneBuffer = this.incompleteBuffer;
        this.incompleteBuffer = null;
        return justGoneBuffer;
    }

    public boolean isOpen() {
        return channel.isOpen();
    }
}