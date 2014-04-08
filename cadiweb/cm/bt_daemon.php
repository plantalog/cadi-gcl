<?php

echo "Bluetooth daemon for Cadi started";


while(1){
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
}



//exec("rfcomm -r connect /dev/rfcomm0 20:13:07:18:09:12 1");

?>
