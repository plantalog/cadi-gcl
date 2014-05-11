<?php

echo "Bluetooth daemon for Cadi started";
$btd_cmd_file = 'daemon_cmd';

// initial Cadi BTDaemon settings load
file_put_contents($btd_cmd_file,'reload_settings,');
$respfs = 0;
$ping_delay = 0;
$cycle_counter = 0;
$video = 0;
$status_stream_enabled=0;		// shows if status stream enabled
$tank = '';

// load SVG display settings
	if (($handle = fopen("btds/svg_layer.conf", "r")) !== FALSE) {
	    	$data = fgetcsv($handle, 1000, ",");
	    	$csd_value = $data[1];
	    	$photo_divider  = $data[0];
	    	$fsd_value  = $data[2];	// File Size Difference in bytes to start parsing
	    	$srtrs_value  = $data[5];	// Serial response tail read size
		$sppd_value = $data[6];	// Status packet ping divider

		// setting tank water polygon parameters (loaded from file)
		$GLOBALS['tank'][3]['top'] = $data[1];	// Maximum water level (distance in cm to sonar installed on top of the water tank)
		$GLOBALS['tank'][3]['bottom'] = $data[2];	// minimum water level
		$GLOBALS['tank'][3]['svg']['height'] = $data[3];	// SVG water polygon height
		$GLOBALS['tank'][3]['svg']['tank_top'] = $data[4];	// SVG water polygon top offset

		// next line
		$data = fgetcsv($handle, 1000, ",");
		$GLOBALS['tank'][4]['top'] = $data[1];
		$GLOBALS['tank'][4]['bottom'] = $data[2];
		$GLOBALS['tank'][4]['svg']['height'] = $data[3];
		$GLOBALS['tank'][4]['svg']['tank_top'] = $data[4];

		fclose($handle);
	}
	else {
		echo 'Loading SVG tank defaults'.PHP_EOL;
		// setting tank water polygon parameters (defaults) SVG
		$GLOBALS['tank'][3]['top'] = 14;	// Maximum water level (distance in cm to sonar installed on top of the water tank)
		$GLOBALS['tank'][3]['bottom'] = 100;	// minimum water level
		$GLOBALS['tank'][3]['svg']['height'] = 400;	// SVG water polygon height
		$GLOBALS['tank'][3]['svg']['tank_top'] = 20;	// SVG water polygon top offset

		$GLOBALS['tank'][4]['top'] = 7;
		$GLOBALS['tank'][4]['bottom'] = 47;
		$GLOBALS['tank'][4]['svg']['height'] = 430;
		$GLOBALS['tank'][4]['svg']['tank_top'] = 300;
		print_r($tank);
	}

print_r($tank);

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
				$respfs = 0;
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
				if ($status_stream_enabled==1) {
					$ping_delay = 10;
					usleep($csd_value*$sppd_value);
				}
				$execmd = "/bin/echo -e '".$cmd_arr[2]."' >> /dev/".$cmd_arr[1];
			break;
			case 'reboot':
				$execmd = "reboot now";
			//	echo $execmd;
				break;
			case 'stream_status':
				echo 'Going to stream status locally';
				$status_stream_enabled = $cmd_arr[1];
				break;
			case 'reload_settings':	// read settings file fetching the daemon settings
				echo PHP_EOL.' reloading Cadi BTDaemon settings'.PHP_EOL;
				if (($handle = fopen("btds/btd.conf", "r")) !== FALSE) {
				    	$data = fgetcsv($handle, 1000, ",");
				    	$csd_value = $data[1];
				    	$photo_divider  = $data[0];
				    	$fsd_value  = $data[2];	// File Size Difference in bytes to start parsing
				    	$srtrs_value  = $data[5];	// Serial response tail read size
					$sppd_value = $data[6];	// Status packet ping divider
					if (!($srtrs_value>40 && $srtrs_value<1000)) {
						$srtrs_value = 100;	// force default size if not in range [40..1000] bytes
					}
					echo PHP_EOL.' new CSD value ='.$csd_value;
					echo PHP_EOL.' new Photoshot divider value ='.$photo_divider.PHP_EOL;
					echo PHP_EOL.' new FileSizeDifference value ='.$fsd_value.PHP_EOL;
					echo PHP_EOL.' new Serial response tail read size value ='.$srtrs_value.PHP_EOL;
					echo PHP_EOL.' new Status packet ping divider ='.$sppd_value.PHP_EOL;
				    fclose($handle);
				}
				else {
					echo PHP_EOL.'WARNING: btds/btd.conf not found!'.PHP_EOL;
					// using default values
					$photo_divider = 80;
					$csd_value = 25000;
					$sppd_value = 80;
				}
				$execmd = "echo";
				break;
		}

		echo $execmd.PHP_EOL;
		exec($execmd);
	}

	usleep($csd_value);
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

	if ($cycle_counter%$sppd_value==0 && $ping_delay==0 && $status_stream_enabled==1) {
		echo '#CadiBTD# Cadi, Ping!';
		$ping_packet = "\x5a\x58\x32\x04\x07\x01\x32\x00";
		$command = "/bin/echo -e '";
		for ($i=0; $i<strlen($ping_packet);$i++) {
			$command .= sprintf("\\x%02x",ord($ping_packet[$i]));
		}
		// $command .= sprintf("\\x%02x",ord($ping_packet));
		$command .= "' >> /dev/cadi";
//		echo $ping_packet.PHP_EOL.$command.PHP_EOL;
		exec($command);
	}
	
	if ($ping_delay>0) {
		$ping_delay--;
	}

//	exec('streamer -f jpeg -o ../img/curimage.jpeg');
}

function parse_response($srtrs){
	echo PHP_EOL.'Trying parse response'.PHP_EOL;
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

	if ($packet_type==3 && strlen($last_packet)>37) {
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
				$statarr['dosingPumpsFlags'] = str_pad(decbin(ord($last_packet[32])), 4, "0", STR_PAD_LEFT);
				$statarr['sonar_read'][0] = $sonar_read[0];	// sonar1 distance
				$statarr['sonar_read'][1] = $sonar_read[1];	// sonar2 distance
				$statarr['time'] = $cadi_time;
				$statarr['adc_avg'][0] = $adc_avg[0];	// ADC average value
				$statarr['adc_avg'][1] = $adc_avg[1];	// ADC average value
				$statarr['adc_avg'][2] = $adc_avg[2];	// ADC average value
				$statarr['adc_avg'][3] = $adc_avg[3];	// ADC average value
				$statarr['psi'] = round((($adc_avg[2]-600)/470), 2); 
				$statarr['comm_state'] = ord($last_packet[2]);

				$tofile[0] = $cadi_time;
				$tofile[1] = $statarr['dht']['temp'];
				$tofile[2] = $statarr['dht']['rh'];
				$tofile[3] = $statarr['timerStateFlags'];
				$tofile[4] = $statarr['cTimerStateFlags'];
				$tofile[5] = $statarr['valves'];
				$tofile[6] = $statarr['plugs'];
				$tofile[7] = $statarr['wpStateFlags'];
				$tofile[8] = $statarr['sonar_read'][0];
				$tofile[9] = $statarr['sonar_read'][1];
				$tofile[10] = $statarr['adc_avg'][0];
				$tofile[11] = $statarr['adc_avg'][1];
				$tofile[12] = $statarr['adc_avg'][2];
				$tofile[13] = $statarr['adc_avg'][3];
				$tofile[14] = $statarr['psi'];
				$tofile[15] = $statarr['comm_state'];
				$tofile[16] = $statarr['dosingPumpsFlags'];
				
				$csv_string = implode(",", $tofile);
				file_put_contents('cadi_status.csv', '');
				file_put_contents('cadi_status.csv',$csv_string);

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



	// Volume: 0% - 100cm, 100% - 14cm
	// Range: 86cm.Resolution:1cm
	// if SVG has height of 400units (let's call this value - $tank['svg']['height'] ), then 1cm = 400/86 ~= 4.65.
	// To draw SVG polygon we use the shifting of 3 top points up/down depending on sonar_read value, displaying current tank volume
	// X - fixed, Y = svg_tank_top_y_coord + (400/86)*(sonar_read - 14)
	// now let's write an equation for this calculation:
	// Y = svg_tank_top_y_coord + ($tank['svg']['height']/($GLOBALS['tank'][3]['bottom']-$GLOBALS['tank'][3]['top']))*(sonar_read - $tank['top'])

	$GLOBALS['tank'][3]['svg']['y'] = $GLOBALS['tank'][3]['svg']['tank_top']+($GLOBALS['tank'][3]['svg']['height']/($GLOBALS['tank'][3]['bottom']-$GLOBALS['tank'][3]['top']))*($statarr['sonar_read'][0] - $GLOBALS['tank'][3]['top']);
	$GLOBALS['tank'][4]['svg']['y'] = $GLOBALS['tank'][4]['svg']['tank_top']+($GLOBALS['tank'][4]['svg']['height']/($GLOBALS['tank'][4]['bottom']-$GLOBALS['tank'][4]['top']))*($statarr['sonar_read'][1] - $GLOBALS['tank'][4]['top']);



	if ($statarr['sonar_read'][0]>$GLOBALS['tank'][3]['bottom']) {
		$GLOBALS['tank'][3]['txt_color'] = "red";
	}
	else {
		$GLOBALS['tank'][3]['txt_color'] = "white";
	}

	$GLOBALS['tank'][4]['txt_color'] = "white";

	for ($i=0;$i<4;$i++) {
		if ($statarr['plugs'][(3-$i)] == 0){
			$plugs['svg']['colors'][$i] = "red";
		}
		else {
			$plugs['svg']['colors'][$i] = "green";
		}
	}

	for ($i=1;$i<4;$i++) {
		if ($statarr['dosingPumpsFlags'][(3-$i)] == 0){
			$dosers['svg']['colors'][$i] = "red";
		}
		else {
			$dosers['svg']['colors'][$i] = "green";
		}
	}

	if ($statarr['comm_state']==51) {	// CDD enabled
		$cdd_string = "CDD Enabled";
		$cdd_color = "green";
	}
	else {	// CDD enabled
		$cdd_string = "CDD Disabled";
		$cdd_color = "red";
	}

	$plugs['svg']['x'] = 1850;
	$plugs['svg']['y'] = 1100;

	// dosingPumpsFlags
	$dosers['svg']['x'] = 1585;
	$dosers['svg']['y'] = 502;
	$dosers['svg']['square_size'] = 36;


	$svg = '';
	$svg .= '<div style="float:left;"><img src="img/cadi_watering_hptl(high_pressure_two_lines).jpg" /></div>';
	$svg .= '<div id="svg_container" style="display:block; float:left; position: absolute; border:1px solid red;">
			<svg opacity="0.5" width="850px" height="400px" viewBox="0 0 2000 1200" xmlns="http://www.w3.org/2000/svg" version="1.1">
				<circle cx="2085" cy="635" r="60" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[1].'" />
				<circle cx="1670" cy="130" r="60" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[0].'" />
				<circle cx="1345" cy="990" r="40" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[2].'" />
				<circle cx="1345" cy="910" r="40" stroke="black" stroke-width="3" opacity="0.7" fill="'.$valve_fill[3].'" />

				<text style="font-size:50px; font-weight:bold;" x="760" y="300" fill="green" >Temp: '.$statarr['dht']['temp'].' C</text>
				<text style="font-size:50px; font-weight:bold;" x="760" y="240" fill="blue" >Humidity: '.$statarr['dht']['rh'].' %</text>
				<text style="font-size:50px; font-weight:bold;" x="750" y="1190" fill="red" >Pressure@7.2 = : '.$statarr['psi'].' bar</text>

				<text style="font-size:50px; font-weight:bold;" x="400" y="70" fill="red" >Cadi time: '.(date("Y-m-d H:i:s", $statarr['time'])).'</text>


				<polygon fill="blue" stroke="blue" stroke-width="10" points="1920,'.(140+$GLOBALS['tank'][3]['svg']['tank_top']+$GLOBALS['tank'][3]['svg']['y']).'  2000,'.($GLOBALS['tank'][3]['svg']['tank_top']+$GLOBALS['tank'][3]['svg']['y']).'  2300,'.($GLOBALS['tank'][3]['svg']['tank_top']+$GLOBALS['tank'][3]['svg']['y']).'  2300,420  2200,580 1920,580" />

				<polygon fill="blue" stroke="blue" stroke-width="10" points="1490,'.(140+$GLOBALS['tank'][4]['svg']['tank_top']+$GLOBALS['tank'][4]['svg']['y']).'  1570,'.($GLOBALS['tank'][4]['svg']['tank_top']+$GLOBALS['tank'][4]['svg']['y']).'  1850,'.($GLOBALS['tank'][4]['svg']['tank_top']+$GLOBALS['tank'][4]['svg']['y']).'  1850,1050  1770,1170  1490,1170" />




			<text style="font-size:52px; font-weight:bold;" x="'.($plugs['svg']['x']+45).'" y="'.($plugs['svg']['y']-10).'" fill="red" >Loads</text>

			<text style="font-size:52px; font-weight:bold;" x="400" y="140" fill="'.$cdd_color.'" >'.$cdd_string.'</text>

			<polygon fill="'.$plugs['svg']['colors'][0].'" stroke="blue" stroke-width="1" points="
				'.$plugs['svg']['x'].','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+100).','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+100).','.($plugs['svg']['y']+100).' 
				'.($plugs['svg']['x']).','.($plugs['svg']['y']+100).' 
			" />
				
			<polygon fill="'.$plugs['svg']['colors'][1].'" stroke="blue" stroke-width="1" points="
				'.($plugs['svg']['x']+105).','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+205).','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+205).','.($plugs['svg']['y']+100).' 
				'.($plugs['svg']['x']+105).','.($plugs['svg']['y']+100).' 
			" />

		<polygon fill="'.$plugs['svg']['colors'][2].'" stroke="blue" stroke-width="1" points="
				'.($plugs['svg']['x']+210).','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+310).','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+310).','.($plugs['svg']['y']+100).' 
				'.($plugs['svg']['x']+210).','.($plugs['svg']['y']+100).' 
			" />

		<polygon fill="'.$plugs['svg']['colors'][3].'" stroke="blue" stroke-width="1" points="
				'.($plugs['svg']['x']+315).','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+415).','.$plugs['svg']['y'].' 
				'.($plugs['svg']['x']+415).','.($plugs['svg']['y']+100).' 
				'.($plugs['svg']['x']+315).','.($plugs['svg']['y']+100).' 
			" />

			<polygon fill="'.$dosers['svg']['colors'][1].'" stroke="blue" stroke-width="1" points="
				'.$dosers['svg']['x'].','.$dosers['svg']['y'].' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']).','.$dosers['svg']['y'].' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']).','.($dosers['svg']['y']+$dosers['svg']['square_size']).' 
				'.($dosers['svg']['x']).','.($dosers['svg']['y']+$dosers['svg']['square_size']).' 
			" />

			<polygon fill="'.$dosers['svg']['colors'][2].'" stroke="blue" stroke-width="1" points="
				'.($dosers['svg']['x']+$dosers['svg']['square_size']+2).','.$dosers['svg']['y'].' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']*2+2).','.$dosers['svg']['y'].' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']*2+2).','.($dosers['svg']['y']+$dosers['svg']['square_size']).' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']+2).','.($dosers['svg']['y']+$dosers['svg']['square_size']).' 
			" />

			<polygon fill="'.$dosers['svg']['colors'][3].'" stroke="blue" stroke-width="1" points="
				'.($dosers['svg']['x']+$dosers['svg']['square_size']*2+2*2).','.$dosers['svg']['y'].' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']*3+2*2).','.$dosers['svg']['y'].' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']*3+2*2).','.($dosers['svg']['y']+$dosers['svg']['square_size']).' 
				'.($dosers['svg']['x']+$dosers['svg']['square_size']*2+2*2).','.($dosers['svg']['y']+$dosers['svg']['square_size']).' 
			" />

				
		


				<text style="font-size:50px; font-weight:bold;" x="1495" y="900" fill="'.$GLOBALS['tank'][4]['txt_color'].'" stroke="black" stroke-width="3"  title="Distance to sonar installed on top" >2Top: '.$statarr['sonar_read'][1].' cm</text>

				<text style="font-size:50px; font-weight:bold;" x="1915" y="520" fill="'.$GLOBALS['tank'][3]['txt_color'].'" stroke="black" stroke-width="3"  title="Distance to sonar installed on top" >2Top: '.$statarr['sonar_read'][0].' cm</text>

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
