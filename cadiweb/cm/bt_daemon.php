<?php




echo "Bluetooth daemon for Cadi started";
$btd_cmd_file = 'daemon_cmd';

// initial Cadi BTDaemon settings load
file_put_contents($btd_cmd_file,'reload_settings,');

$cycle_counter = 0;
$video = 0;
$status_stream_enabled=0;		// shows if status stream enabled
while(1){
	$cycle_counter++;
	$command = file_get_contents($btd_cmd_file);
	unset($execmd);
	if(!empty($command)){
		$cmd_arr = explode(",", $command);
		print_r($cmd_arr);
		echo '('.date(DATE_RFC2822).') Got CMD ';
		file_put_contents($btd_cmd_file,'');
		switch ($cmd_arr[0]) {
			case 'bind':
				
			//	proc_close(proc_open('rfcomm -r connect /dev/'.$cmd_arr[1].' '.$cmd_arr[2].' > /dev/null &', array(), $pipe));
			//	$execmd = 'rfcomm -r connect /dev/'.$cmd_arr[1].' '.$cmd_arr[2].' > /dev/null &';
				$execmd = 'rfcomm -r bind /dev/'.$cmd_arr[1].' '.$cmd_arr[2].' 1';
				exec($execmd);
				$execmd = 'ln -s /dev/'.$cmd_arr[1].' /dev/cadi';
				break;
			case 'disconnect':
				$execmd = "'kill -9 $(pidof rfcomm)'";
				break;
			case 'stream':
				$execmd = 'rfcomm -r bind /dev/'.$cmd_arr[1].' '.$cmd_arr[2].' 1';
	//			exec($execmd);
				$execmd = "cat /dev/".$cmd_arr[1]." > serialresp.out &";
				break;
			case 'release':
				$execmd = 'rfcomm release 0 ; rm -rf /dev/'.$cmd_arr[1].' ; rm -rf /dev/cadi';
				exec($execmd);
				$execmd = 'service bluetooth restart';	// Ubuntu 12.04 LTS
				exec($execmd);
				break;
			case 'kill':
				$execmd = "'kill -9 $(pidof rfcomm)'";
				break;
			case 'restart':
				$execmd = "'./bt_restart.sh'";
				break;
			case 'change_video':
				$execmd = "echo";
				$video=$cmd_arr[1];
				break;
			case 'tx':		// send packet
				$execmd = "/bin/echo -e '".$cmd_arr[2]."' >> /dev/".$cmd_arr[1];
			break;
			case 'reboot':
				$execmd = "eboot no";
			//	echo $execmd;
				break;
			case 'stream_status':
				$status_stream_enabled = $cmdarr[1];
				break;
			case 'reload_settings':	// read settings file fetching the daemon settings
				echo PHP_EOL.' reloading Cadi BTDaemon settings'.PHP_EOL;
				if (($handle = fopen("btds/btd.conf", "r")) !== FALSE) {
				    	$data = fgetcsv($handle, 1000, ",");
				    	$csd_value = $data[1];
				    	$photo_divider  = $data[0];
				    	$fsd_value  = $data[2];	// File Size Difference in bytes to start parsing
				    	$srtrs_value  = $data[5];	// Serial response tail read size
					if (!($srtrs_value>40 && $srtrs_value<1000)) {
						$srtrs_value = 100;	// force default size if not in range [40..1000] bytes
					}
					echo PHP_EOL.' new CSD value ='.$csd_value;
					echo PHP_EOL.' new Photoshot divider value ='.$photo_divider.PHP_EOL;
					echo PHP_EOL.' new FileSizeDifference value ='.$fsd_value.PHP_EOL;
					echo PHP_EOL.' new Serial response tail read size value ='.$srtrs_value.PHP_EOL;
				    fclose($handle);
				}
				else {
					echo PHP_EOL.'WARNING: btds/btd.conf not found!'.PHP_EOL;
					// using default values
					$photo_divider = 80;
					$csd_value = 25000;
				}
				$execmd = "echo";
				break;
		}

		echo $execmd.PHP_EOL;
		exec($execmd);
	}

	usleep(25000);
	if ($cycle_counter%$photo_divider==0) {	// photo_divider needed for making photo NOT every while() iteration
		exec('fswebcam -d /dev/video'.$video.' -r 640x480 --jpeg 85 ../img/curimage.jpeg >> /dev/null &');
	}
	if ($cycle_counter>9999999){
		$cycle_counter=0;
	}
	$curespfs = filesize('serialresp.out');	// current response file size
	clearstatcache();				// refresh file size
	if ($curespfs>($respfs+$fsd_value)) {	// compare new and old values if they match. if not, file changed (35 for minimum file difference)
		$respfs = $curespfs;		// update old file size value
		parse_response($srtrs_value);		// parse response if change detected
	}

//	exec('streamer -f jpeg -o ../img/curimage.jpeg');
}

function parse_response($srtrs){
	echo PHP_EOL.'Attempting to parse response'.PHP_EOL;
	// create file pointer
	$fp = fopen('serialresp.out', 'rb');
	fseek($fp, ($srtrs*(-1)), SEEK_END); // It needs to be negative to seek from the file end
	$data = fread($fp, $srtrs);
	fclose($fp);


	$packets_arr = explode('ZX', $data);
	$last_packet_id = count($packets_arr)-1;
	// echo 'Count='.$last_packet_id;
	$last_packet = $packets_arr[$last_packet_id];

/*	for ($i=0; $i<=strlen($last_packet);$i++) {
		$hexpacket .= sprintf("\\x%02x",ord($last_packet[$i]));
	}

	echo $hexpacket.PHP_EOL; */


	$packet_type = $last_packet[0];

	$packet_size = ord($last_packet[1]);

	if ($packet_type==3) {
		// STATUS data provider
		print_r($last_packet);
		$block_id = ord($last_packet[36]);
		if ($block_id==1) {
			$comm_state = ord($last_packet[2]);		// TxBuffer[4] in STM32 firmware
			$timerStateFlags = decbin(ord($last_packet[3]));
			$cTimerStateFlags = decbin(ord($last_packet[4]));
	//		$valveFlags = decbin(ord($last_packet[5]));
	//		$plugStateFlags = decbin(ord($last_packet[6]));
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
				$outstr = '';
				// echo 'CRC OK';
				$statarr['dht']['temp'] = ($dht[2]*256+$dht[3])/10;	// DHT temperature
				$statarr['dht']['rh'] = ($dht[0]*256+$dht[1])/10;	// DHT sensor relative humidity
				$statarr['timerStateFlags'] = str_pad(decbin(ord($last_packet[3])), 4, "0", STR_PAD_LEFT);
				$statarr['cTimerStateFlags'] = str_pad(decbin(ord($last_packet[4])), 4, "0", STR_PAD_LEFT);
				$statarr['valves'] = str_pad(decbin(ord($last_packet[5])), 4, "0", STR_PAD_LEFT);
				$statarr['plugs'] = str_pad(decbin(ord($last_packet[6])), 4, "0", STR_PAD_LEFT);
				$statarr['wpStateFlags'] = decbin(ord($last_packet[7]));
				$statarr['dosingPumpsFlags'] = '&nbsp;';
				$statarr['sonar_read'][0] = $sonar_read[0];	// sonar1 distance
				$statarr['sonar_read'][1] = $sonar_read[1];	// sonar2 distance
				$statarr['time'] = $cadi_time;
				$statarr['adc_avg'][0] = $adc_avg[0];	// ADC average value
				$statarr['adc_avg'][1] = $adc_avg[1];	// ADC average value
				$statarr['adc_avg'][2] = $adc_avg[2];	// ADC average value
				$statarr['adc_avg'][3] = $adc_avg[3];	// ADC average value
				$statarr['psi'] = round((($adc_avg[2]-600)/470), 2); 
			
				

				$statstr = 'PHP time: '.date("Y-m-d H:i:s", time()).'
					<table>
					<tr>
						<td>Cadi Time</td>
						<td>'.date("Y-m-d H:i:s", $statarr['time']).'</td>
					</tr>
					<tr>
						<td>220V Plugs</td>
						<input type="hidden" value="'.$statarr['plugs'].'" />
						<td>'.$statarr['plugs'].'</td>
					</tr>
					<tr>
						<td>12V plugs</td>
						<td>'.$statarr['dosingPumpsFlags'].'</td>
					</tr>
					<tr>
						<td>Valve states</td>
						<td>'.$statarr['valves'].'</td>
					</tr>
					<tr>
						<td>Water levels</td>
						<td>'.$statarr['sonar_read'][0].' / '.$statarr['sonar_read'][1].'</td>
					</tr>
					<tr>
						<td>Temperature</td>
						<td>'.$statarr['dht']['temp'].' C</td>
					</tr>
					<tr>
						<td>Humidity</td>
						<td>'.$statarr['dht']['rh'].' %</td>
					</tr>
					<tr>
						<td>Pressure</td>
						<td>'.$statarr['psi'].' bar</td>
					</tr>
					<tr>
						<td>ADC1</td>
						<td>'.$statarr['adc_avg'][0].'</td>
					</tr>
					<tr>
						<td>ADC2</td>
						<td>'.$statarr['adc_avg'][1].'</td>
					</tr>
					<tr>
						<td>ADC3</td>
						<td>'.$statarr['adc_avg'][2].'</td>
					</tr>
					<tr>
						<td>ADC4</td>
						<td>'.$statarr['adc_avg'][3].'</td>
					</tr>
					</table>
			';
			file_put_contents('status_view_1.php', '');
			file_put_contents('status_view_1.php',$statstr);
				
			$statstr = build_svg($statarr);
	//		echo "STARTSRT".$statstr.PHP_EOL;
			file_put_contents('status_view_2.php', '');
			file_put_contents('status_view_2.php',$statstr);

			}
			else {
				echo 'CRC broken';
				echo 'Counted CRC='.$counted_crc.'<br>';
				echo 'PacketCRCByte='.$crc.'<br>';
			}
		}
	}
}

// Makes an SVG image string to be layered over the Cadi water circuit diagram as status layer
function build_svg($statarr){
	//$statarr['sonar_read'][1];
	$valves = str_pad($statarr['valves'], 4, "0", STR_PAD_LEFT);
	for ($i=0;$i<4;$i++) {
		if ($valves[(3-$i)] == 0){
			$valve_fill[$i] = "red";
		}
		else {
			$valve_fill[$i] = "green";
		}
	}
	$tank4_height = (70*$statarr['sonar_read'][0])/10;
	$svg = '';
	$svg .= '<div style="float:left;"><img src="img/cadi_watering_hptl(high_pressure_two_lines).jpg" /></div>';
	$svg .= '<div id="svg_container" style="display:block; float:left; position: absolute; border:1px solid red;">
			<svg opacity="0.5" width="850px" height="400px" viewBox="0 0 2000 1200" xmlns="http://www.w3.org/2000/svg" version="1.1">
				<circle cx="2085" cy="635" r="60" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[0].'" />
				<circle cx="1670" cy="130" r="60" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[1].'" />
				<circle cx="1345" cy="990" r="40" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[2].'" />
				<circle cx="1345" cy="910" r="40" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[3].'" />
				<text style="font-size:50px; font-weight:bold;" x="760" y="1180" fill="red" >Pressure@7.2 = '.$statarr['psi'].' bar</text>
				<text style="font-size:50px; font-weight:bold;" x="760" y="300" fill="red" >Temperature: '.$statarr['dht']['temp'].' C</text>
				<text style="font-size:50px; font-weight:bold;" x="760" y="240" fill="red" >Humidity: '.$statarr['dht']['temp'].' C</text>
				<polygon fill="blue" stroke="blue" stroke-width="10" points="1920,'.(200+8*$statarr['sonar_read'][1]).'  2000,'.(60+8*$statarr['sonar_read'][1]).'  2300,'.(60+8*$statarr['sonar_read'][1]).'  2300,420  2200,580 1920,580" />
				<polygon fill="blue" stroke="blue" stroke-width="10" points="1490,'.(700+$tank4_height).'  1570,'.(560+$tank4_height).'  1850,'.(560+$tank4_height).'  1850,1050  1770,1170  1490,1170" />

			</svg>


		</div>';
	return $svg;
}

function crc_block($input, $packet, $length){	// $input is XORin init
	for ($i=0; $i<$length; $i++) {
		$input ^= ord($packet[$i]);
	}
	return $input;
} 

//exec("rfcomm -r connect /dev/rfcomm0 20:13:07:18:09:12 1");

?>
