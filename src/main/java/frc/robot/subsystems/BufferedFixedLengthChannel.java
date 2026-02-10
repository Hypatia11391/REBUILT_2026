package frc.robot.subsystems;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.SocketChannel;
import java.util.stream.*;

public class BufferedFixedLengthChannel {
    private ByteBuffer incompleteBuffer;
    private final SocketChannel channel;
    private final int packetLength;

    public BufferedFixedLengthChannel(SocketChannel channel, int packetLength) {
        this.packetLength = packetLength;
        this.channel = channel;
    }

    public Stream<ByteBuffer> pollFullBuffers() {
        return Stream.iterate(this.pollBuffer(),(lastBuf) -> this.pollBuffer());
    }

    public ByteBuffer pollBuffer() {
        if(this.incompleteBuffer == null) {
            this.incompleteBuffer = ByteBuffer.allocate(packetLength);
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
        return this.incompleteBuffer;
    }
}

