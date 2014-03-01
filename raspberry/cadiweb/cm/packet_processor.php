<?php




$cmd = $_POST['cmd'];
$packet_pref = "ZX2";

switch($cmd) {
	case 48:		// get_water(8,8,16)
		$_POST['gw_amount'][0]*256+
		$packet .= "0";
		$packet .= chr($_POST['valve']);		// add valve_id
		$packet .= chr($_POST['counter']);		// counter_id
		$packet .= chr(intval($_POST['gw_amount']/256));	// first byte of uint16_t amount (of water)
		$packet .= chr($_POST['gw_amount']%256);		// second byte
	break;
}

$packet_crc = crcs($packet);
$out = $packet_pref.$packet.$packet_crc;

exec("echo '".$out."' > /dev/rfcomm0");		// HARDCODE!!

function crcs($s){	// generate xor crc checksum
	$curxor = 0;
	for ($i=0;$i<strlen($s);$i++){
		$curxor ^ $s[$i];
	}
	return $curxor;
}






?>
