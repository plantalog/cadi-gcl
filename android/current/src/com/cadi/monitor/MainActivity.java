package com.cadi.monitor;


import java.io.IOException;
import java.io.OutputStream;
import java.io.InputStream;
import java.io.BufferedInputStream;
import java.util.List;
import java.util.UUID;
import java.util.Date;
import java.text.SimpleDateFormat;

import com.cadi.monitor.DeviceListActivity;
import com.cadi.monitor.BluetoothSerialService;
// import com.cadi.monitor.R;
import com.cadi.graphbuilder.Draw2D;

import android.util.Log;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.DialogInterface.OnClickListener;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.KeyEvent;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.app.Activity;
import android.view.Menu;
import android.view.inputmethod.InputMethodManager;

public class MainActivity extends Activity {
    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE = 1;
    private static final int REQUEST_ENABLE_BT = 2;

    // Name of the connected device
    private String mConnectedDeviceName = null;

    /**
     * Set to true to add debugging code and logging.
     */
    public static final boolean DEBUG = true;

    /**
     * Set to true to log each character received from the remote process to the
     * android log, which makes it easier to debug some kinds of problems with
     * emulating escape sequences and control codes.
     */
    public static final boolean LOG_CHARACTERS_FLAG = DEBUG && false;

    /**
     * Set to true to log unknown escape sequences.
     */
    public static final boolean LOG_UNKNOWN_ESCAPE_SEQUENCES = DEBUG && false;

  

    // Message types sent from the BluetoothReadService Handler
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;	

    // Key names received from the BluetoothChatService Handler
    public static final String DEVICE_NAME = "device_name";
    public static final String TOAST = "toast";
	
	private BluetoothAdapter mBluetoothAdapter = null;	
	
    private static BluetoothSerialService mSerialService = null;
	
//	private boolean mEnablingBT;
    private boolean mLocalEcho = false;

	
    private MenuItem mMenuItemConnect;
    private Button buttonConnect;
    private TextView outText;
    private TextView outText2;
    private String tmpBuff;
    private TextView rHum;
    private TextView temp;
    private TextView plugStates;
    private TextView curTime;
    private Draw2D temp_graph;
    
	private int cadiMode = 0;
	
    
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);	//	eta i sledujushaja stroki musthave
		setContentView(R.layout.activity_main);
		temp_graph = (Draw2D) findViewById(R.id.temp_graph);	// assign to class variables the element in xml layout
        outText = (TextView) findViewById(R.id.textview_output);
        outText2 = (TextView) findViewById(R.id.textview_output2);
        rHum = (TextView) findViewById(R.id.val_humidity);
        temp = (TextView) findViewById(R.id.val_temperature);
        curTime = (TextView) findViewById(R.id.val_curtime);
        plugStates = (TextView) findViewById(R.id.val_plug_states);
        buttonConnect = (Button) findViewById(R.id.button_connect);
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		mSerialService = new BluetoothSerialService(this, mHandlerBT, outText);
		
		tmpBuff = new String();
		
	    buttonConnect.setOnClickListener(new View.OnClickListener() {
 			
 			@Override
 			public void onClick(View v) {
				//if (connectStat) {
					// Attempt to disconnect from the device
					// disconnect();
				//}else{
					// Attempt to connect to the device
					connect();
				//}
 			}
 		});
	}
	
	public void onStart() {
		super.onStart();
	//	if (DEBUG)
	//	mEnablingBT = false;
	}
	
	@Override
	public void onDestroy() {
		super.onDestroy();
		if (DEBUG)
		
        if (mSerialService != null)
        	mSerialService.stop();
        
	}
	
	public int getConnectionState() {
		return mSerialService.getState();
	}
	
	// some copy paste from BlueTerm, without some lines
	private final Handler mHandlerBT = new Handler() {
	        @Override
	        public void handleMessage(Message msg) {        	
	            switch (msg.what) {
	            case MESSAGE_STATE_CHANGE:
	                switch (msg.arg1) {
	                case BluetoothSerialService.STATE_CONNECTED:
	                	if (mMenuItemConnect != null) {
	                		mMenuItemConnect.setIcon(android.R.drawable.ic_menu_close_clear_cancel);
	                		mMenuItemConnect.setTitle(R.string.disconnect);
	                	}
	                    break;
	                    
	                case BluetoothSerialService.STATE_CONNECTING:
	                    break;
	                    
	                case BluetoothSerialService.STATE_LISTEN:
	                case BluetoothSerialService.STATE_NONE:
	                	if (mMenuItemConnect != null) {
	                		mMenuItemConnect.setIcon(android.R.drawable.ic_menu_search);
	                		mMenuItemConnect.setTitle(R.string.connect);
	                	}
	                    break;
	                }
	                break;
	            case MESSAGE_WRITE:
	            	if (mLocalEcho) {
	            		byte[] writeBuf = (byte[]) msg.obj;
	            	}
	                
	                break;
	                
	            case MESSAGE_READ:

	                byte[] readBuf = (byte[]) msg.obj;
	                String readMessage = new String(readBuf, 0, msg.arg1);
	                
	            	if (cadiMode==0){
	            		log_process(readMessage, msg.arg1);
	            	}
	            	
	            	if (cadiMode==1){
	            		get_settings(readBuf, msg.arg1);
	            	}
	            	
	            	if (cadiMode==0){
	            		send_settings(readBuf);
	            	}
	                
	       
	                
	                break;
	                
	            case MESSAGE_DEVICE_NAME:
	                // save the connected device's name
	                mConnectedDeviceName = msg.getData().getString(DEVICE_NAME);
	                Toast.makeText(getApplicationContext(), "Connected to "
	                               + mConnectedDeviceName, Toast.LENGTH_SHORT).show();
	                break;
	            case MESSAGE_TOAST:
	                Toast.makeText(getApplicationContext(), msg.getData().getString(TOAST),
	                               Toast.LENGTH_SHORT).show();
	                break;
	            }
	        }
	    };
	    
	    public void get_settings(byte[] data, int size){
	    	
	    }
	    
	    public void send_settings(byte[] data){
	    	
	    }
	   
	    public void log_process(String readMessage, int count){
	         if (tmpBuff.length()<190) {
             	tmpBuff += readMessage;
             }
             else {
             	String[] outLines = tmpBuff.split("U");	// split the buffer to get the log string
             	int crc = 0;
             	for (int i=0; i<200; i++) {
//             		if (readBuf[i]==85 && i<120 && readBuf[i+1]==49 && readBuf[i+64]==85){	// if found U with distance of 63bytes between them
//             			i=200;
//             		}
//             		else {
//             		}
             	}
             	int cntd_crc=0;	// init value for counted crc
             	
             	if (outLines[1].length()>62) {	// count crc of data received
             		crc = outLines[1].charAt(62);
             		for (int i=0; i<62;i++) {
	                		cntd_crc = cntd_crc ^ (int)outLines[1].charAt(i); // here we use XOR (^) algo to get the checksum
	                	}
             	}                	
             	
             	// display counte and received CRCs
             	String outStr1 = Integer.toString(cntd_crc);
             	String outStr2 = Integer.toString(crc);
             	outText2.setText(outStr1); 
             	outText2.append(" == ");
             	outText2.append(outStr2);
             	
             	// check th string for right CRC and dots and commas positions
             	if (crc==cntd_crc && outLines[1].length()>61 && outLines[1].length()<64 
             				&& outLines[1].charAt(10)==','
             				&& outLines[1].charAt(15)==','
             				&& outLines[1].charAt(20)==','
 	                		&& outLines[1].charAt(24)==','
 	                		&& outLines[1].charAt(29)==','
 	                		&& outLines[1].charAt(34)==','
 	                		&& outLines[1].charAt(38)==','
 	    	                && outLines[1].charAt(42)==','
 	    	                && outLines[1].charAt(46)==','
 	    	    	        && outLines[1].charAt(50)==','
 	    	    	        && outLines[1].charAt(54)==','
 	    	    	        && outLines[1].charAt(58)==','
 	    	    	                && outLines[1].charAt(13)=='.'
 	    	    	    	        && outLines[1].charAt(17)=='.'
 	    	    	    	        && outLines[1].charAt(27)=='.'
 	    	    	    	        && outLines[1].charAt(32)=='.'				    	                		
             				) {
	                	// outText.setText(outLines[1]);
	                	String[] vals = outLines[1].split(",");	// split the string to get the values
	                    long time = Integer.parseInt(vals[0].toString());
	                    Date date = new Date(time*1000);
	                    SimpleDateFormat format = new SimpleDateFormat("EEE MMM dd HH:mm:ss zzz yyyy a");	// setting the date-time format for output
	                    curTime.setText(format.format(date));	// display date
	                	temp.setText(vals[4]);	// and other values
	                	rHum.setText(vals[5]);
	                	plugStates.setText(vals[3]);
	                	float number = Float.valueOf(vals[4].toString());	// prepare values to put on graph
	                	int temp = (int)((number*10)/2-100);
	                	number = Float.valueOf(vals[5].toString());
	                	int rh = (int)(number*10)/10;
	                	int ph = 20;
	                	int ec = 25;
		                temp_graph.pushValues(temp, rh, ph, ec);	// and push then into draw object
	                }
	                tmpBuff=readMessage;
             }
             
             
          //   mEmulatorView.write(readBuf, msg.arg1);
	    }
	    
	    public void onActivityResult(int requestCode, int resultCode, Intent data) {
	        // if(DEBUG) Log.d(LOG_TAG, "onActivityResult " + resultCode);
	        switch (requestCode) {
	        
	        case REQUEST_CONNECT_DEVICE:

	            // When DeviceListActivity returns with a device to connect
	            if (resultCode == Activity.RESULT_OK) {
	                // Get the device MAC address
	                String address = data.getExtras()
	                                     .getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
	                // Get the BLuetoothDevice object
	                BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
	                // Attempt to connect to the device
	                mSerialService.connect(device);                
	            }
	            break;

	        case REQUEST_ENABLE_BT:
	            // When the request to enable Bluetooth returns
	            if (resultCode == Activity.RESULT_OK) {
	                // Log.d(LOG_TAG, "BT not enabled");
	                
	                //finishDialogNoBluetooth();                
	            }
	        }
	    }
	    
	    public void connect() {
	    	 // Launch the DeviceListActivity to see devices and do scan
	         Intent serverIntent = new Intent(this, DeviceListActivity.class);
	         startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
	    }
	     

}