<?php

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

unset($packet);
if (isset($_POST['cmd'])) {
	$packet_pref = "ZX2";
	$packet.=$packet_pref;
	$cmd = $_POST['cmd'];
	switch($cmd) {
		case 0:		// get_water(8,8,16)
			$packet .= chr(5);	// packet payload size
			$packet .= chr(0);
			$packet .= chr($_POST['valve']);		// add valve_id
			$packet .= chr($_POST['counter']);		// counter_id
			$packet .= chr(intval($_POST['gw_amount']/256));	// first byte of uint16_t amount (of water)
			$packet .= chr($_POST['gw_amount']%256);		// second byte
			$packet .= chr($_POST['gw_amount']%256);		// second byte

			break;
		case 1:		// plugStateSet(8,8)
			$packet .= chr(4);	// packet payload size
			$packet .= chr(1);	// 
			$packet .= chr($_POST['plug']); 
			$packet .= chr($_POST['state']);
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
		case 7:		// get_status_block();
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(7);	// command
			$packet .= chr($_POST['block_id']);	// STATUS block_id
			break;
		case 8:		// set auto_flags
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(8);	// command
			$packet .= chr($_POST['flags']);	// new auto_flags byte
			break;
		case 10:		// get_status_block();
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(10);	// command
			break;
		case 11:		// get_status_block();
			$packet .= chr(2);	// packet payload size (including this size byte)
			$packet .= chr(11);	// command
			break;
		case 12:		// get_status_block();
			$packet .= chr(6);	// packet payload size (including this size byte)
			$packet .= chr(12);	// command
			$curtime = time();
			$packet .= chr(floor($curtime/16777216));	// command
			$packet .= chr((($curtime%16777216)/65536));	// command
			$packet .= chr((($curtime%65536)/256));	// command
			$packet .= chr($curtime%256);	// command
			echo $curtime;
			break;
	}

	$packet[3] = chr(ord($packet[3])+1);	// increase packet length byte for CRC use (TEMPORARY solution, use fixed values when stable)
	$curcrc = chr(crc_block(0, $packet, (strlen($packet))));
	$packet .= $curcrc;	// add CRC to packet
	

	unset($arguments);
	for ($i=0; $i<=strlen($packet);$i++) {
		$arguments .= sprintf("\\x%02x",ord($packet[$i]));
	}
	unset($packet);
	$packet = $arguments;
	$cadi_packet = $arguments;
}

else {
	$cadi_packet = "empty packet";
}

$packet_crc = crcs($packet);
$out = $packet_pref.$packet.$packet_crc;

function crcs($s){	// generate xor crc checksum
	$curxor = 0;
	for ($i=0;$i<strlen($s);$i++){
		$curxor ^ $s[$i];
	}
	return $curxor;
}






?>
