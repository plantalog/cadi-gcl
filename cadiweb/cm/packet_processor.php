<?php

unset($packet);

// echo "packet processor included";
if (isset($_POST['cmd'])) {
	$packet_pref = "ZX2";
	$packet.=$packet_pref;
//	print_r($_POST);
	$cmd = $_POST['cmd'];
	switch($cmd) {
		case 0:		// get_water(8,8,16)
			$packet .= chr(5);	// packet payload size
			$packet .= chr(0);
			$packet .= chr($_POST['valve']);		// add valve_id
			$packet .= chr($_POST['counter']);		// counter_id
			$packet .= chr(intval($_POST['gw_amount']/256));	// first byte of uint16_t amount (of water)
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
			$packet .= chr(5);	// command
			$packet .= chr($_POST['block_id']);	// SETTINGS block id
			break;
		case 7:		// get_status_block();
			$packet .= chr(3);	// packet payload size (including this size byte)
			$packet .= chr(7);	// command
			$packet .= chr($_POST['block_id']);	// STATUS block_id
			break;
	}


	unset($arguments);
	for ($i=0; $i<=strlen($packet);$i++) {
		$arguments .= sprintf("\\x%02x",ord($packet[$i]));
	}
	unset($packet);
	$packet = $arguments;
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
