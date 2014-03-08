package cadi.portable;

import java.util.Arrays;

/**
 * This is declarative stateful receiver written in imperative 
 * style that can easily be ported to c.
 */
public class DeclarativeReceiver {
    // constants for state machine
    private static final int STATE_NOTHING = 0;
    private static final int STATE_CONFIGURATION_PAYLOAD = 1;
    private static final int STATE_CONFIGURATION_CRC = 2;
    private static final int STATE_COMMAND_TYPE = 3;
    private static final int STATE_COMMAND_PAYLOAD = 4;
    private static final int STATE_COMMAND_CRC = 5;
    
    private int prefixDetectionIdx; // counter for length of the sequence matching the packet start prefix

    private int state = STATE_NOTHING; // state for stat machine

    private byte[] payload; // payload buffer for payload reading states
    private int payloadIdx; // payload index for payload reading states
    
    private int command; // command for command reading
    private int crc; // crc byte
    
    // this is the only public method we have to use; just feed all received bytes to this method
    public void onByteReceived(int b) {
        if (b < 0) { 
            throw new IllegalArgumentException(); // this makes sense for java only, prevent feeding EOF
        }
        // first we apply packet prefix detection logic
        if (b == 'Z') {
            prefixDetectionIdx = 1; // ready...
        } else if (b == 'X' && prefixDetectionIdx == 1) {
            prefixDetectionIdx = 2; // steady...
        } else if (prefixDetectionIdx == 2) { // we got something after prefix
            if (b == 0) { 
                // escaping zero, just skip it
            } else if (state == STATE_NOTHING) {
                onPacketType(b); // ... go!(?)
            } else {
                throw new IllegalStateException("Unquoted packet prefix found inside packet stream");
            }
            prefixDetectionIdx = 0;
            return;
        }

        // here is the main portion of state machine, which proceeds packet bytes
        switch (state) {
        case STATE_CONFIGURATION_PAYLOAD:
            onPayloadByte(b, STATE_CONFIGURATION_CRC);
            break;
        case STATE_COMMAND_TYPE:
            onCommandTypeByte(b);
            break;
        case STATE_COMMAND_PAYLOAD:
            onPayloadByte(b, STATE_COMMAND_CRC);
            break;
        case STATE_CONFIGURATION_CRC: 
        case STATE_COMMAND_CRC:
            onCrcByte(b, state);
            break;
        default:
            break;
        }
    }

    private void onCrcByte(int b, int crcType) {
        // when we receive crc byte - must validate crc match to the one calculated from read payload
        if ((b & 0xFF) != crc) {
            throw new IllegalStateException("CRC mismatch");
        }
        // if everything is fine - invoke packet receive callback methods
        switch (crcType) {
        case STATE_CONFIGURATION_CRC:
            configurationReceived(payload);
            break;
        case STATE_COMMAND_CRC:
            commandReceived(command, payload);
            break;
        default:
            break;
        }
        state = STATE_NOTHING;
    }

    // this callback method is invoked if command packet is successfully received
    private void commandReceived(int command, byte[] payload) {
        System.out.println(String.format("command(%d, %s)", command, Arrays.toString(payload)));
    }

    // this callback method is invoked if configuration packet is successfully received
    private void configurationReceived(byte[] payload) {
        System.out.println(String.format("configuration(%s)", Arrays.toString(payload)));
    }

    private void onPacketType(int b) {
        // depending on packet type we transit to next state
        switch (b) {
        case 1: // for configuration packet type...
            payload = new byte[400]; 
            payloadIdx = 0;
            state = STATE_CONFIGURATION_PAYLOAD; // ... expect for 400 bytes payload
            break;
        case 2: // for command packet type...
            state = STATE_COMMAND_TYPE; // ...expect command type to be read next
            break;
        default:
            throw new IllegalStateException("Unsupported packet type read " + b);
        }
        crc = 0;
    }
    
    private void onPayloadByte(int b, int nextState) {
        // on each payload byte - fill payload buffer and calculate crc
        payload[payloadIdx++] = (byte) b;
        if (payloadIdx == payload.length) {
            state = nextState;
        }
        crc(b);
    }

    private void onCommandTypeByte(int b) {
        // here we have to allocate memory buffer for each command arguments (payload)...
        switch (b) {
        case 1: // command_get_water...
            payload = new byte[4]; // ...has 4 byte payload
            break;
        default:
            throw new IllegalStateException("Unknown command type " + b);
        }
        command = b;
        payloadIdx = 0;
        crc(b);
        state = STATE_COMMAND_PAYLOAD;
    }
    
    // calculate crc
    private void crc(int b) {
        crc ^= b;
    }
    
    // test
    public static void main(String[] args) {
        // bytes from Main
        byte[] data = new byte[] {90, 123, 1, 0, 90, 88, 2, 1, 0, 1, 2, 3, 1, 90, 88, 2, 1, 123, 127, -128, 0, -123, 12, 90, 88, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 102};
        DeclarativeReceiver r = new DeclarativeReceiver();
        for (int i = 0; i < data.length; i++) {
            byte b = data[i];
            r.onByteReceived(unsigned(b));
        }
    }
    
    // this only makes sense for java
    private static final int unsigned(int b) {
        return b & 0xFF;
    }
}
