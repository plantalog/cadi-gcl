<?php




echo "Bluetooth daemon for Cadi started";
$photo_divider = 2;
$cycle_counter = 0;
while(1){
	$cycle_counter++;
	$command = file_get_contents('daemon_cmd');
	unset($execmd);
	if(!empty($command)){
		$cmd_arr = explode(",", $command);
		print_r($cmd_arr);
		echo 'Got CMD';
		file_put_contents('daemon_cmd','');
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
			case 'tx':		// send packet
				$execmd = "/bin/echo -e '".$cmd_arr[2]."' >> /dev/".$cmd_arr[1];
			break;
			case 'reboot':
				$execmd = "eboot no";
			//	echo $execmd;
				break;
		}

		echo $execmd;
		exec($execmd);
	}

	sleep(1);
	if ($cycle_counter%$photo_divider==0) {	// photo_divider needed for making photo NOT every while() iteration
		exec('fswebcam -d /dev/video1 -r 640x480 --jpeg 85 ../img/curimage.jpeg >> /dev/null &');
	}
	if ($cycle_counter>9999999){
		$cycle_counter=0;
	}

//	exec('streamer -f jpeg -o ../img/curimage.jpeg');
}



//exec("rfcomm -r connect /dev/rfcomm0 20:13:07:18:09:12 1");

?>
