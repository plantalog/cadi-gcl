package cadi.core;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

// this is the API for interaction with transport. 
// Contains methods for sending packets to streams and reading them from streams.
public final class Transport {
    private static final int EOF = -1;
    private static final int PACKET_TYPE_CONFIGURATION = 1;
    private static final int PACKET_TYPE_COMMAND = 2;

    private static final int PAYLOAD_LENGTH_CONFIGURATION = 400;
    
    public static final int COMMAND_GET_WATER = 1;
    
    // returns true if next packet was received and handled; false otherwise (eof has been reached)
    public boolean receive(InputStream is, PacketReceiveCallback callback) throws IOException {
        InputStream pis = PacketStreamUtil.inputStream(is);
        int packetType = pis.read();
        switch (packetType) {
        case EOF:
            return false;
        case PACKET_TYPE_CONFIGURATION:
            receiveConfiguration(pis, callback);
            break;
        case PACKET_TYPE_COMMAND:
            receiveCommand(pis, callback);
            break;
        default:
            throw new IllegalStateException("Unsupported packet type " + packetType);
        }
        return true;
    }
    
    private void receiveCommand(InputStream pis, PacketReceiveCallback callback) throws IOException {
        int command = read(pis);
        byte[] payload = new byte[commandPayloadLength(command)];
        for (int i = 0; i < payload.length; i++) {
            payload[i] = (byte) read(pis);
        }
        validateChecksum(new Checksum().put(command).put(payload).get(), read(pis));
        callback.onCommand(command, payload);
    }

    private void receiveConfiguration(InputStream pis, PacketReceiveCallback callback) throws IOException {
        byte[] payload = new byte[PAYLOAD_LENGTH_CONFIGURATION];
        for (int i = 0; i < payload.length; i++) {
            payload[i] = (byte) read(pis);
        }
        validateChecksum(new Checksum().put(payload).get(), read(pis));
        callback.onConfiguration(payload);
    }

    public void sendConfiguration(OutputStream os, byte[] payload) throws IOException {
        validateLength(payload, PAYLOAD_LENGTH_CONFIGURATION);
        OutputStream pos = PacketStreamUtil.outputStream(PACKET_TYPE_CONFIGURATION, os);
        pos.write(payload);
        pos.write(new Checksum().put(payload).get());
    }

    public void sendCommand(OutputStream os, int command, byte[] params) throws IOException {
        validateLength(params, commandPayloadLength(command));
        OutputStream pos = PacketStreamUtil.outputStream(PACKET_TYPE_COMMAND, os);
        pos.write(command);
        pos.write(params);
        pos.write(new Checksum().put(command).put(params).get());
    }
    
    // read and return byte from the stream; fail if EOF is reached
    private int read(InputStream pis) throws IOException {
        int result = pis.read();
        if (result == EOF) {
            throw new IOException("Unexpected EOF detected in the middle of the packet");
        }
        return result;
    }

    private void validateLength(byte[] payload, int expectedLength) {
        if (payload.length != expectedLength) {
            throw new IllegalArgumentException("Length of parameter payload " 
                    + payload.length + " bytes does not match expected " + expectedLength);
        }
    }

    // validate checksum and fail if expected crc not equals to actual
    private void validateChecksum(int expected, int actual) {
        if (expected != actual) {
            throw new IllegalStateException("Received checksum " + actual + " does not match expected " + expected);
        }
    }

    // this method returns length in bytes for each particular command
    private int commandPayloadLength(int command) {
        switch (command) {
        case COMMAND_GET_WATER:
            return 4;
        default:
            new IllegalArgumentException("Unsupported command type " + command);
        }
        return 0;
    }
    
    // callback interface
    public interface PacketReceiveCallback {
        void onConfiguration(byte[] payload);
        void onCommand(int command, byte[] payload);
    }
    
    // checksum helping class
    private static class Checksum {
        private static final int UNSIGNED_BYTE_MASK = 0xFF;
        private int result;

        public Checksum put(byte[] val) {
            for (byte b : val) {
                result ^= b & UNSIGNED_BYTE_MASK;
            }
            return this;
        }

        public Checksum put(int val) {
            result ^= val & UNSIGNED_BYTE_MASK;
            return this;
        }
        
        public int get() {
            return result;
        }
    }
}
