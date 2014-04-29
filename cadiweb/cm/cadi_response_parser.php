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
		$block_id = ord($last_packet[36]);
		echo 'something--separator--';
		switch ($block_id) {
			case 1:
				$comm_state = ord($last_packet[2]);		// TxBuffer[4] in STM32 firmware
				$timerStateFlags = decbin(ord($last_packet[3]));
				$cTimerStateFlags = decbin(ord($last_packet[4]));
//				$valveFlags = decbin(ord($last_packet[5]));
//				$plugStateFlags = decbin(ord($last_packet[6]));
				$wpStateFlags = ord($last_packet[7]);
				$dht[0] = ord($last_packet[8]);
				$dht[1] = ord($last_packet[9]);
				$dht[2] = ord($last_packet[10]);
				$dht[3] = ord($last_packet[11]);
				// Sonar data
				$sonar_read[0] = ord($last_packet[16])+ord($last_packet[17])*265;		// First sonar lower and higher bytes
				$sonar_read[1] = ord($last_packet[18])+ord($last_packet[19])*265;		// First sonar lower and higher bytes



				$cadi_ta[2] = ord($last_packet[12]);		// TxBuffer[14] = (uint8_t)(RTC->CNTH&(0xFF));
				$cadi_ta[3] = ord($last_packet[13]);		// TxBuffer[15] = (uint8_t)((RTC->CNTH>>8)&(0xFF));
				$cadi_ta[0] = ord($last_packet[14]);		// TxBuffer[16] = (uint8_t)(RTC->CNTL&(0xFF));
				$cadi_ta[1] = ord($last_packet[15]);		// TxBuffer[17] = (uint8_t)(((RTC->CNTL)>>8)&(0xFF));
				$cadi_time = $cadi_ta[3]*16777216+$cadi_ta[2]*65536+$cadi_ta[1]*256+$cadi_ta[0];

				$adc_avg[0] = ord($last_packet[20])+ord($last_packet[21])*256;
				$adc_avg[1] = ord($last_packet[22])+ord($last_packet[23])*256;
				$adc_avg[2] = ord($last_packet[24])+ord($last_packet[25])*256;
				$adc_avg[3] = ord($last_packet[26])+ord($last_packet[27])*256;

				$crc = ord($last_packet[($packet_size-3)]);
				$counted_crc = crc_block(2, $last_packet, ($packet_size-3));	// 90 xor 88 = 2
				if ($counted_crc==$crc) {
					// echo 'CRC OK';
					$_SESSION['cadi_status']['dht']['temp'] = ($dht[2]*256+$dht[3])/10;	// DHT temperature
					$_SESSION['cadi_status']['dht']['rh'] = ($dht[0]*256+$dht[1])/10;	// DHT sensor relative humidity
					$_SESSION['cadi_status']['timerStateFlags'] = decbin(ord($last_packet[3]));
					$_SESSION['cadi_status']['cTimerStateFlags'] = decbin(ord($last_packet[4]));
					$_SESSION['cadi_status']['valves'] = decbin(ord($last_packet[5]));
					$_SESSION['cadi_status']['plugs'] = decbin(ord($last_packet[6]));
					$_SESSION['cadi_status']['wpStateFlags'] = decbin(ord($last_packet[7]));
					$_SESSION['cadi_status']['dosingPumpsFlags'] = '&nbsp;';
					$_SESSION['cadi_status']['sonar_read'][0] = $sonar_read[0];	// sonar1 distance
					$_SESSION['cadi_status']['sonar_read'][1] = $sonar_read[1];	// sonar2 distance
					$_SESSION['cadi_status']['time'] = $cadi_time;
					$_SESSION['cadi_status']['adc_avg'][0] = $adc_avg[0];	// ADC average value
					$_SESSION['cadi_status']['adc_avg'][1] = $adc_avg[1];	// ADC average value
					$_SESSION['cadi_status']['adc_avg'][2] = $adc_avg[2];	// ADC average value
					$_SESSION['cadi_status']['adc_avg'][3] = $adc_avg[3];	// ADC average value
					$_SESSION['cadi_status']['psi'] = round((($adc_avg[2]-230)/470), 2);
				}
				else {
					echo 'CRC broken';
					echo 'Counted CRC='.$counted_crc.'<br>';
					echo 'PacketCRCByte='.$crc.'<br>';
					// wrong CRC reports broken packet
					/*
					$_SESSION['cadi_status']['dht']['temp'] = ($dht[2]*256+$dht[3])/10;	// DHT temperature
					$_SESSION['cadi_status']['dht']['rh'] = ($dht[0]*256+$dht[1])/10;	// DHT sensor relative humidity
					$_SESSION['cadi_status']['dosingPumpsFlags'] = "BAD DATA. Packet CRC=".$crc.' while Counted CRC='.$counted_crc.' #';
					$_SESSION['cadi_status']['cTimerStateFlags'] = decbin(ord($last_packet[4]));
					$_SESSION['cadi_status']['valves'] = decbin(ord($last_packet[5]));
					$_SESSION['cadi_status']['plugs'] = decbin(ord($last_packet[6]));
					$_SESSION['cadi_status']['wpStateFlags'] = decbin(ord($last_packet[7]));


					*/
				}
			///	echo 'aaaaaaaa';

			break;

			case 2:
				// extract Cadi time 4 bytes array


				// Extract ADC average values
				$adc_avg[0] = ord($last_packet[6])+ord($last_packet[7])*256;
				$adc_avg[1] = ord($last_packet[8])+ord($last_packet[9])*256;
				$adc_avg[2] = ord($last_packet[10])+ord($last_packet[11])*256;

				$crc = ord($last_packet[($packet_sizde-3)]);
				$counted_crc = crc_block(2, $last_packet, ($packet_sizde-3));	// 90 xor 88 = 2, 14 - the rest of 16byte "ZX3..." packet
				if ($counted_crc==$crc) {
					$_SESSION['cadi_status']['time'] = $cadi_time;	// Cadi UNIXTIME
					$_SESSION['cadi_status']['adc_avg'][0] = $adc_avg[0];	// ADC average value
					$_SESSION['cadi_status']['adc_avg'][1] = $adc_avg[1];	// ADC average value
					$_SESSION['cadi_status']['adc_avg'][2] = $adc_avg[2];	// ADC average value
				}
				else {
					// wrong CRC reports broken packet
				}
			break;

			case 5:

/*
		TxBuffer[4] = (uint8_t)(sonar_read[0]&(0xFF));		// First sonar lower byte
		TxBuffer[5] = (uint8_t)((sonar_read[0]>>8)&(0xFF));	// first sonar higher byte
		TxBuffer[6] = (uint8_t)(sonar_read[1]&(0xFF));		// second sonar
		TxBuffer[7] = (uint8_t)((sonar_read[1]>>8)&(0xFF));
		TxBuffer[8] = (uint8_t)(water_counter[0]&(0xFF));		// first wfm counter
		TxBuffer[9] = (uint8_t)((water_counter[0]>>8)&(0xFF));
		TxBuffer[10] = (uint8_t)((water_counter[0]>>16)&(0xFF));
		TxBuffer[11] = (uint8_t)((water_counter[0]>>24)&(0xFF));
		TxBuffer[12] = valve_failed;		//
		TxBuffer[13] = waterSensorStateFlags;
		TxBuffer[14] = blockId;
*/
				
				// Sonar data
				$sonar_read[0] = ord($last_packet[2])+ord($last_packet[3])*265;		// First sonar lower and higher bytes
				$sonar_read[1] = ord($last_packet[4])+ord($last_packet[5])*265;		// First sonar lower and higher bytes

				// 32 bit water_counter[0]
				$water_counter[0] = ord($last_packet[6])+ord($last_packet[7])*2^8+ord($last_packet[8])*2^16+ord($last_packet[9])*2^24;

				// valve_failed flags
				$valve_failed = decbin(ord($last_packet[10]));

				$crc = ord($last_packet[($packet_sizde-3)]);
				$counted_crc = crc_block(2, $last_packet, ($packet_sizde-3));	// 90 xor 88 = 2, 14 - the rest of 16byte "ZX3..." packet
//				echo '     __________cntd_crc='.$counted_crc;
				if ($counted_crc==$crc) {
					$_SESSION['cadi_status']['valve_failed'] = $valve_failed;	// failed valves flags
					$_SESSION['cadi_status']['water_counter'][0] = $water_counter[0];
				}
				else {
					// wrong CRC reports broken packet
				}
		/*			for ($i=0; $i<=strlen($last_packet);$i++) {

						$hexpacket .= sprintf("\\x%02x",ord($last_packet[$i]));

					}

					echo $hexpacket; */
			break;


			case 4:
/*

		TxBuffer[6] = phUnderOver+ecUnderOver*4;
		TxBuffer[7] = (uint8_t)(phWindowTop&(0xFF));
		TxBuffer[8] = (uint8_t)((phWindowBottom>>8)&(0xFF));
		TxBuffer[9] = 00;		// EMPTY
		TxBuffer[10] = dosingPumpStateFlags;
		TxBuffer[11] = (uint8_t)(wfCalArray[0]&(0xFF));
		TxBuffer[12] = (uint8_t)((wfCalArray[0]>>8)&(0xFF));
		TxBuffer[13] = waterSensorStateFlags;
		TxBuffer[14] = blockId;

*/
				$current_ec = ord($last_packet[2]);		// TxBuffer[4] = currentEc;
				$current_ph = ord($last_packet[3]);		// TxBuffer[5] = currentPh;
	
				$crc = ord($last_packet[($packet_sizde-3)]);
				$counted_crc = crc_block(2, $last_packet, ($packet_sizde-3));	// 90 xor 88 = 2, 14 - the rest of 16byte "ZX3..." packet
				if ($counted_crc==$crc) {
			//		$_SESSION['cadi_status']['time'] = $cadi_time;	// Cadi UNIXTIME

				}
				else {
					// wrong CRC reports broken packet

				}
			break;
		}
				for ($i=0; $i<=strlen($last_packet);$i++) {
						$hexpacket .= sprintf("\\x%02x",ord($last_packet[$i]));
					}
					//echo $hexpacket; 					
	break;
}

?>
