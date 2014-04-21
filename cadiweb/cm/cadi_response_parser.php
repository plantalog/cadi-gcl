<?php

// echo 'CRParser included';
$fp = fopen('serialresp.out', 'rb');
fseek($fp, -50, SEEK_END); // It needs to be negative
$data = fread($fp, 50);
$packets_arr = explode('ZX', $data);
$last_packet_id = count($packets_arr)-1;
// echo 'Count='.$last_packet_id;
$last_packet = $packets_arr[$last_packet_id];

$packet_type = $last_packet[0];

$packet_size = ord($last_packet[1]);

switch ($packet_type) {
	case 0:
		// screening
	break;
	case 1:
		// settings
	break;
	case 2:
		// command
	break;
	case 3:
		// STATUS data provider
		$block_id = ord($last_packet[12]);
		echo 'something--separator--';
		switch ($block_id) {
			case 1:
				$comm_state = ord($last_packet[2]);
				$timerStateFlags = decbin(ord($last_packet[3]));
				$cTimerStateFlags = decbin(ord($last_packet[4]));
//				$valveFlags = decbin(ord($last_packet[5]));
//				$plugStateFlags = decbin(ord($last_packet[6]));
				$wpStateFlags = ord($last_packet[7]);
				$dht[0] = ord($last_packet[8]);
				$dht[1] = ord($last_packet[9]);
				$dht[2] = ord($last_packet[10]);
				$dht[3] = ord($last_packet[11]);
				$crc = ord($last_packet[13]);
				$counted_crc = crc_block(2, $last_packet, 13);	// 90 xor 88 = 2, 14 - the rest of 16byte "ZX3..." packet
				if ($counted_crc==$crc) {
					$_SESSION['cadi_status']['dht']['temp'] = ($dht[2]*256+$dht[3])/10;	// DHT temperature
					$_SESSION['cadi_status']['dht']['rh'] = ($dht[0]*256+$dht[1])/10;	// DHT sensor relative humidity
					$_SESSION['cadi_status']['timerStateFlags'] = decbin(ord($last_packet[3]));
					$_SESSION['cadi_status']['cTimerStateFlags'] = decbin(ord($last_packet[4]));
					$_SESSION['cadi_status']['valves'] = decbin(ord($last_packet[5]));
					$_SESSION['cadi_status']['plugs'] = decbin(ord($last_packet[6]));
					$_SESSION['cadi_status']['wpStateFlags'] = decbin(ord($last_packet[7]));
					$_SESSION['cadi_status']['dosingPumpsFlags'] = '&nbsp;';
				}
				else {
					// wrong CRC reports broken packet
					/*
					$_SESSION['cadi_status']['dht']['temp'] = ($dht[2]*256+$dht[3])/10;	// DHT temperature
					$_SESSION['cadi_status']['dht']['rh'] = ($dht[0]*256+$dht[1])/10;	// DHT sensor relative humidity
					$_SESSION['cadi_status']['dosingPumpsFlags'] = "BAD DATA. Packet CRC=".$crc.' while Counted CRC='.$counted_crc.' #';
					$_SESSION['cadi_status']['cTimerStateFlags'] = decbin(ord($last_packet[4]));
					$_SESSION['cadi_status']['valves'] = decbin(ord($last_packet[5]));
					$_SESSION['cadi_status']['plugs'] = decbin(ord($last_packet[6]));
					$_SESSION['cadi_status']['wpStateFlags'] = decbin(ord($last_packet[7]));

					for ($i=0; $i<=strlen($last_packet);$i++) {
						$hexpacket .= sprintf("\\x%02x",ord($last_packet[$i]));
					}
					echo $hexpacket;
					*/
				}
			break;
		}
	break;
}

?>
