<?php
session_start();
// cs - for Cadi Settings
$_SESSION['cs'] = array();

$_SESSION['cs']['timers'][0]['description'] = 'default timer description';
$_SESSION['cs']['timers'][0]['on'] = 0;
$_SESSION['cs']['timers'][0]['off'] = 0;
$_SESSION['cs']['timers'][0]['enabled'] = 0;
$_SESSION['cs']['plugs']['0']['description'] = 'default plug description';
$_SESSION['cs']['plugs']['0']['program_link'] = '0';	// timer, ctimer... id

$_SESSION['cs']['valves']['0']['description'] = 'default plug description';
$_SESSION['cs']['valves']['0']['program_link'] = '0';	// timer, ctimer... id
$_SESSION['csbin_filename'] = 'csbin_local';	// cadi settings binary file name
$_SESSION['csblksize'] = 32;	// 16 variables of 16bit each.

	// read dump file settings from btd.conf (shared with BTDaemon)
	if (($handle = fopen("btds/btd.conf", "r")) !== FALSE) {
		$data = fgetcsv($handle, 1000, ",");
		$settings_filesize = $data[7];	// File settings dump file size
		$settings_startaddr = $data[8];	// Settings start address
		fclose($handle);
	}




// injects Cadi settings config file into dump file
function sync_conf2dump(){
	global $settings_startaddr;
	if (($handle = fopen("cadi_settings_conf.csv", "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$sca[$row++] = $data;
	    }
	    fclose($handle);
	}


	// read dump file into variable
	$curfsize = filesize('cadi_settings_dump');	// current response file size
	clearstatcache();				// refresh file size
	$fp = fopen('cadi_settings_dump', 'rb'); // open in binary read mode.
	fseek($fp, 0,0); //point to file start
	$settings_dump = array();
	$settings_dump = fread($fp,$curfsize); // read settings dump data
	fclose($fp); // close the file




	// pack corresponding (from conf file adresses and types) values to settings dump file
	foreach ($sca as $key=>$row) {
		if ($row[1]==1) {
			// pack 8 bit value
			$addr = substr($row[0],0,5);
			echo 'packing 8 bit at '.$addr.PHP_EOL;
			$parity = substr($row[0],5,1);	// 0 - higher byte, 1 - lower byte
			$pointer = ($addr-$settings_startaddr)*2;
			if ($parity!="1") {	//
				$pointer++;	
			}
			$settings_dump[$pointer] = chr($sca[$key][2]);
		}
		if ($row[1]==2) {
			// 16 bit value pack
			$addr = $row[0];
			echo 'packing 16 bit at '.$addr.PHP_EOL;
			$pointer = ($addr-$settings_startaddr)*2;
			$settings_dump[$pointer++] = chr(floor($sca[$key][2]/256));
			$settings_dump[$pointer] = chr($sca[$key][2]%256);
		}
		if ($row[1]==3) {
			// 32 bit

			$addr = 0;
			$addr = $row[0];
			$val = 0;
			$val = floor($sca[$key][2]/65536);
			echo 'packing 16 (1/2) bit at '.$addr.PHP_EOL;
			$pointer = ($addr-$settings_startaddr)*2;
			$settings_dump[$pointer++] = chr(floor($val/256));
			$settings_dump[$pointer++] = chr($val%256);

			$addr++;
			$val = 0;
			$val = $sca[$key][2]%65536;
			echo 'packing 16 (2/2) bit at '.$addr.PHP_EOL;
			$pointer = ($addr-$settings_startaddr)*2;
			$settings_dump[$pointer++] = chr(floor($val/256));
			$settings_dump[$pointer] = chr($val%256);

		}
	}

	// recreate new dump file
	$fp = fopen('cadi_settings_dump', 'w');
	fwrite($fp, $settings_dump);
	fclose($fp); // close the file

}

// extracts Cadi settings from dump file into config file
function sync_dump2conf(){
	// CSV file contains lines with fields:
	// 0. address
	// 1. type (1 - 8bit, 2 - 16bit, 3 - 32bit)
	// 2. value (plain text integer value)
	// 3. text name, explaining the meaning of the value, kind of comment
	global $settings_startaddr;
	if (($handle = fopen("cadi_settings_conf.csv", "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$sca[$row++] = $data;
	    }
	    fclose($handle);
	}


	// TEST BLOCK
	$curfsize = filesize('cadi_settings_dump');	// current response file size
	clearstatcache();				// refresh file size
	$fp = fopen('cadi_settings_dump', 'rb'); // open in binary read mode.
	fseek($fp, 0,0); //point to file start
	$settings_dump = array();
	$settings_dump = fread($fp,$curfsize); // read settings dump data
	fclose($fp); // close the file
	$settings_file_hex='';
	for ($i=0; $i<strlen($settings_dump);$i++) {
		if (($i%10)==0) {
			$settings_file_hex .= PHP_EOL;
			$settings_file_hex .= ' '.($i/10).' ';
		}
		$settings_file_hex .= sprintf("\\x%02x",ord($settings_dump[$i]));
	}
//	echo PHP_EOL."settings file hex (first byte has STM32 EEPROM address ".$settings_startaddr."): ".PHP_EOL.$settings_file_hex.PHP_EOL;
	// EOF TEST BLOCK


	// extract corresponding (to conf file adresses and types) values from settings dump file
	foreach ($sca as $key=>$row) {
		if ($row[1]==1) {
			// extract 8 bit value
			$addr = substr($row[0],0,5);
			$parity = substr($row[0],5,1);	// 0 - lower byte, 1 - higher byte
			if (($fp = fopen("cadi_settings_dump", "rb")) !== FALSE) {
				$seekaddr = ($addr-$settings_startaddr)*2;
				if ($parity == '1'){
					$seekaddr++;
				}
				fseek($fp,$seekaddr,0); 
			    	$data = fread($fp, 1);
				$sca[$key][2] = ord($data);
				fclose($fp);
			}
		}
		if ($row[1]==2) {
			// 16 bit value extraction

			$addr = $row[0];
			//echo 'Extracting 16 bit val from addr '.$addr.PHP_EOL;
			if (($fp = fopen("cadi_settings_dump", "rb")) !== FALSE) {
				$seekaddr = ($addr-$settings_startaddr)*2;
				fseek($fp,$seekaddr,0); 
			    	$data = fread($fp, 2);
				$sca[$key][2] = ord($data[0])*256+ord($data[1]);
				fclose($fp);
			}
		}
		if ($row[1]==3) {
			// extract 32 bits from dump file
			$addr = substr($row[0],0,5);
			if (($fp = fopen("cadi_settings_dump", "rb")) !== FALSE) {
				$seekaddr = ($addr-$settings_startaddr)*2;
				fseek($fp,$seekaddr,0); 
			    	$data = fread($fp, 4);
				$sca[$key][2] = ord($data[0])*16777216+ord($data[1])*65536+ord($data[2])*256+ord($data[3]);
				fclose($fp);
			}
		}
	}

	// recreate new file with new CSV values of dumped binary
	if (($fp = fopen("cadi_settings_conf.csv", "w")) !== FALSE) {
		foreach ($sca as $key=>$row) {
			fputcsv($fp,$row);
		}
	fclose($fp);
	}

}



?>
