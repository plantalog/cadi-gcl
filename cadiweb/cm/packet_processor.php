<?php

// read dump file settings from btd.conf (shared with BTDaemon)
if (($handle = fopen("btds/btd.conf", "r")) !== FALSE) {
	$data = fgetcsv($handle, 1000, ",");
	$settings_filesize = $data[7];	// File settings dump file size
	$settings_startaddr = $data[8];	// Settings start address
	fclose($handle);
}

function crc_block_ord($input, $ord_arr, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input = 111;
	}
	return $input;
}

function crc_block($input, $packet, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input ^= ord($packet[$i]);
	}
	return $input;
} 

// extracts 16byte block of settings dump file to send to Cadi
function dump2block($addr){
	global $settings_startaddr;
	$fp = fopen('cadi_settings_dump', 'rb'); // open in binary read mode.
	fseek($fp, (($addr-$settings_startaddr)*2),0); //point to file start
	$settings_dump = array();
	$settings_dump = fread($fp,32); // read settings dump data
	fclose($fp); // close the file
	$settings_hex = '';
	for ($i=0; $i<(strlen($block_data)/2);$i++) {
		// swap
		$temp = $settings_dump[$i*2];
		$settings_dump[$i]=$settings_dump[($i*2+1)];
		$settings_dump[($i*2+1)] = $temp;
		// GDE-TO TUT :)))))	
	}
	return $settings_dump;
}

unset($packet);
if (isset($_POST['cmd'])) {
	$packet_pref = "ZX2";
	$packet.=$packet_pref;
	$cmd = $_POST['cmd'];
	switch($cmd) {
		case 0:		// get_water(8,8,16)
			$packet .= chr(6);	// packet payload size
			$packet .= chr(0);
			$packet .= chr($_POST['valve']);		// add valve_id
			$packet .= chr($_POST['counter']);		// counter_id
			$packet .= chr(intval($_POST['gw_amount']/256));	// first byte of uint16_t amount (of water)
			$packet .= chr($_POST['gw_amount']%256);		// second byte

			break;
		case 1:		// plugStateSet(8,8)
			$packet .= chr(4);	// packet payload size (Payload starts here)
			$packet .= chr(1);	//  Command ID
			$packet .= chr($_POST['plug']);		 // Plug ID  (command arguments)
			$packet .= chr($_POST['state']);	// new Plug State (Payload ends here)
			break;
		case 2:		// Direct drive enable
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(2);	// command
			break;
		case 3:		// Direct drive disable
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(3);	// command
			break;
		case 4:		// open_valve()
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(4);	// command
			$packet .= chr($_POST['valve']);	// valve_id
			break;
		case 5:		// close_valve()
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(5);	// command
			$packet .= chr($_POST['valve']);	// valve_id
			break;
		case 6:		// get_settings_block()
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(6);	// command
			$packet .= chr($_POST['block_id']);	// SETTINGS block id
			break;

		case 8:		// set auto_flags
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(8);	// command
			$packet .= chr($_POST['flags']);	// new auto_flags byte
			break;
/*		case 9:		// dosing pump enable/disable
			$packet .= chr(4);	// packet payload size (including this size byte)
			$packet .= chr(9);	// command
			$packet .= chr($_POST['pump_id']);	// pumpId for doser
			$packet .= chr($_POST['state']);	// new state for doser
			break;	*/
		case 9:		// dosing pump fertilizer intake in seconds
			$packet .= chr(5);	// packet payload size (including this size byte)
			$packet .= chr(9);	// command
			$packet .= chr($_POST['pump_id']);	// pumpId for doser
			$packet .= chr($_POST['amount']);	// in seconds
			$packet .= chr($_POST['speed']);	// in %, 1 - enabled full
			break;
		case 10:		// set new valve_failed 8bit value
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(10);	// command
			break;
		case 11:		// stop all processes and force manual control
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(11);	// command
			break;
		case 12:		// set time
			$curtime = time();
			$packet .= chr(6);	// packet payload size (including this size byte)
			$packet .= chr(12);	// command
			$packet .= chr(floor($curtime/16777216));	// command
			$packet .= chr((($curtime%16777216)/65536));	// command
			$packet .= chr((($curtime%65536)/256));	// command
			$packet .= chr($curtime%256);	// command
			echo $curtime;
			break;
		case 13:		// get_status_block();
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(13);	// command
			break;
		case 15:		// rx_ee((RxBuffer[2]&(RxBuffer[3]<<8)), 1); writes 16bit variable into STM32's Emulated EEPROM
			$packet .= chr(6);	// packet payload size (including this size byte)
			$packet .= chr(15);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			$packet .= chr($_POST['value']%256);	// value lower byte
			$packet .= chr(floor($_POST['value']/256));	// higher byte
			break;
		case 16:		// rx_ee((RxBuffer[2]&(RxBuffer[3]<<8)), 16); writes 32 byte (16 vars) block into STM32's Emulated EEPROM
			$packet .= chr(20);	// packet payload size (including this size byte)
			$packet .= chr(16);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			$settings_block_data = dump2block($_POST['addr']);
			$packet .= $settings_block_data;
			break;
		case 19:		// reload settings
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(19);	// command
			break;

// Above 50th there are commands that send successfull execution confirmation "ZX7"
		case 51:		// get_status_block();
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(51);	// command
			$packet .= chr($_POST['block_id']);	// STATUS block_id
			break;
		case 58:		// EE_ReadVariable(uint32_t addr)
			$packet .= chr(4);	// packet payload size (including this size byte)
			$packet .= chr(58);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			break;
		case 59:		// EE_ReadVariable(uint32_t addr)
			$packet .= chr(4);	// packet payload size (including this size byte)
			$packet .= chr(59);	// command
			$packet .= chr($_POST['addr']%256);	// address lower byte
			$packet .= chr(floor($_POST['addr']/256));	// higher byte
			break;

	}

	// SOME HARDCODE
	$packet[3] = chr(ord($packet[3])+1);	// increase packet length byte for CRC use (TEMPORARY solution, use fixed values when stable)
//	$packet[3] = chr(ord($packet[3])+1);	// increase packet length byte for packet ID use (same story as before)
								// Packet ID is being generated and added to the end of packet within Cadi BTDaemon

	// CRC
	$curcrc = chr(crc_block(0, $packet, (strlen($packet))));	// calculate XOR CRC checksum for the packet
	$packet .= $curcrc;	// add CRC to packet
	

	// convert packet into "echo -e" Linux command usable format
	unset($arguments);
	for ($i=0; $i<strlen($packet);$i++) {
		$arguments .= sprintf("\\x%02x",ord($packet[$i]));
	}
	unset($packet);
	$packet = $arguments;
	$cadi_packet = $arguments;

	$packet_crc = crcs($packet);
	$out = $packet_pref.$packet.$packet_crc;
}

else {
	$cadi_packet = "empty packet";
}


function crcs($s){	// generate xor crc checksum
	$curxor = 0;
	for ($i=0;$i<strlen($s);$i++){
		$curxor ^ $s[$i];
	}
	return $curxor;
}






?>
