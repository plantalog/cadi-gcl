<?php

echo "Bluetooth daemon for Cadi started";


while(1){
	$command = file_get_contents('daemon_cmd');

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
				break;
			case 'disconnect':
				$execmd = "'kill -9 $(pidof rfcomm)'";
				break;
			case 'stream':
				$execmd = "cat /dev/".$cmd_arr[1]." > /srv/http/cm/serialresp.out &";
				break;
			case 'release':
				$execmd = 'rfcomm release 0';
				break;
			case 'kill':
				$execmd = "'kill -9 $(pidof rfcomm)'";
				break;
			case 'restart':
				$execmd = "'./bt_restart.sh'";
				break;
			case 'tx':		// send packet
				$execmd = "echo ".$cmd_arr[2]." > /dev/".$cmd_arr[1];
				echo $execmd;
				break;
		}

	//	echo $execmd;
		exec($execmd);
	}

	sleep(1);
}



//exec("rfcomm -r connect /dev/rfcomm0 20:13:07:18:09:12 1");

?>
