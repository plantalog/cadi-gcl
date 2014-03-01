<?php 
session_start();

if (isset($_GET['action'])) {
	$action = $_GET['action'];
}
else {
	$action = $_POST['action'];
}


switch ($action) {
	case 'bt_connect':
		// rfcomm_bind($_POST['mac'], $_POST['channel'], $_POST['name']);
		// rfcomm_bind($_POST['rfcomm'], $_POST['mac'], 1);
	//	sleep(1);
		$toput = "release,".$_POST['rfcomm'].", ";
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		$toput = "bind,".$_POST['rfcomm'].",".$_POST['mac'];
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		$toput = "stream,".$_POST['rfcomm'].", ";
		file_put_contents('daemon_cmd', $toput);
		break;
	case 'bt_disconnect':
		$toput = "release,".$_POST['rfcomm'].", ";
		echo $toput;
		file_put_contents('daemon_cmd', $toput);
		sleep(1);
		break;
	case 'rfcomm_scan':
		rfcomm_scan();
		break;
	case 'rfcomm_list_binded':
		rfcomm_list_binded();
		break;
	case 'stop_serial_read':
		stop_serial_read($_POST['process']);
		break;
	case 'tail_serial_log':
		tail_serial_log($_POST['amount']);
		break;
	case 'command_send':
		command_send($_POST['command'], $_POST['mac']);
		break;
}

function command_send($command, $mac){
	$out = array();
	$cmd = 'sudo echo '.time().': '.$command.' >> cadi_input';
//	$out = exec('sudo python test_btserialclient2.py '.$mac.' '.$command);
	exec($cmd, $out);
	echo $cmd;
}

function tail_serial_log($amount){
	$filename = 'serialresp.out';
	if (1) {
		# Processing
		
		$i = 0;
		$arrr = read_file('serialresp.out', 10);
		echo "---separator---";
		$cadi_str_arr = explode(',', $arrr[8]);
		$cadi_time = date('l jS \of F Y h:i:s A', $cadi_str_arr[0]);
		
		
		for ($i=0; $i<3; $i++) {
			$curflag = substr($cadi_str_arr[3], $i, 1);
			if ($curflag == '1'){
				$plugout .= '<b style="background:green; color:white;">&nbsp;'.$i.'&nbsp;</b>';
			}
			else {
                       		$plugout .= '<b style="background:red; color:white;">&nbsp;'.$i.'&nbsp;</b>';
			}
		}

		for ($i=0; $i<3; $i++) {
			$curflag = substr($cadi_str_arr[7], $i, 1);
			if ($curflag == '1'){
				$ctimerout .= '<b class="flgenb">'.$i.'</b>';
			}
			else {
                       		$ctimerout .= '<b class="flgdsb">'.$i.'</b>';
			}
		}
		
		
		$timer_States='';
		/* echo '<table>
			<tr>
				<td>Time:</td>
				<td>'.$cadi_str_arr[0].'<td>
			</tr>
			<tr>
				<td>Temp:</td>
				<td>'.$cadi_str_arr[4].'<td>
			</tr>
			<tr>
				<td>rH:</td>
				<td>'.$cadi_str_arr[5].'<td>
			</tr>
			<tr>
				<td>Plug states:</td>
				<td>'.$plugout.'<td>
			</tr>
			<tr>
				<td>CTimer states:</td>
				<td>'.$ctimerout.'<td>
			</tr>



		</table>'; */
		echo $arrr[8];
	}
		

}

function stop_serial_read($psid){
	$out = array();
	exec('kill -9 '.$psid, $out);
	echo 'kill -9 '.$psid;
}

function rfcomm_scan(){
	$out = array();
	$out2 = array();
	exec('hcitool scan', $out);

	for ($i=1; $i<sizeof($out); $i++) {
	
		$out2[$i][0] = substr($out[$i], 1, 17);
	    	$out2[$i][1] = substr($out[$i], 18, strlen($out[$i])-18);
		echo '<option value="'.$out2[$i][0].'">';
		echo $out2[$i][1].' ('.$out2[$i][0].')';
		echo '</option>';
	}
}


function rfcomm_list_binded(){
	echo 'Binded RFCOMMs list:';
	$out = array();
	// get "ls -la" to list existing bindings
//	exec('ls /dev/ |grep rfcomm', $out);
	exec('rfcomm -a', $out);
	echo '<ul>';
	for ($i=0; $i<sizeof($out); $i++) {
		$rfend = strpos($out[$i], ':');
		$rfcomm_name = substr($out[$i], 0, $rfend);
		echo '<li>'.$out[$i].'&nbsp;&nbsp;&nbsp;<div style="display:inline; border:1px solid red;" onClick=bt_disconnect("'.$rfcomm_name.'");>Disconnect</div></li>';
	}
	echo '</ul>';

	unset($out);
	$cmd = "ps -ef |grep rfcomm";
	exec($cmd, $out);
	echo $cmd;
	echo '<ul>';

	for ($i=0; $i<sizeof($out); $i++) {
		$position = strpos($out[$i], 'rfcomm');
		if (strpos($out[$i], 'oot')>0 && $position==0) {
			$startpos = digit_offset($out[$i]);
			$psid = substr($out[$i], $startpos,5);
			echo '<li>'.$out[$i].' <div style="border:1px solid red; display:inline;" onClick=stopSerialRead('.$psid.')>Kill '.$psid.'</div></li>';
		}
	}
	echo '</ul>';
//	echo '<div style="display:inline; border:1px solid red;" onClick="cadi_list_rfcomms();">Refresh binds</div>';
}

function digit_offset($text){
    preg_match('/^\D*(?=\d)/', $text, $m);
    return strlen($m[0]);
}


function rfcomm_bind($name, $mac, $channel){
	$out = array();
	$out = exec('rfcomm -r bind /dev/'.$name.' '.$mac.' '.$channel);
}

function rfcomm_release($name){
	$out = array();
	$out = exec('rfcomm release /dev/'.$name);	
}


function rfcomm_connect($rfcomm_name, $mac){
	$out = array();
//	$cmd = 'rfcomm -r connect /dev/'.$rfcomm_name.' '.$mac.' 1 > /dev/null &';
//	$cmd = 'rfcomm -r bind /dev/'.$rfcomm_name.' '.$mac.' 1 > /dev/null &';
	$cmd = './bt_expect.exp '.$rfcomm_name.' '.$mac.' > /dev/null &';
	$cmd = 'id';
//	pclose(popen("rfcomm -r connect /dev/rfcomm0 20:13:07:18:09:12 > /dev/null &","r"));
	$out = exec($cmd);
//	$cmd = "rfcomm connect /dev/".$rfcomm_name." ".$mac." > /dev/null & /dev/null 2> /dev/null &";
//	proc_open($cmd, array(), $pipes);
	echo $cmd.'<br>';
//	$out .= exec("cat /dev/".$rfcomm_name." > serialresp.out");
	print_r($out);
}

// kill the process with name "rfcomm"
function rfcomm_kill(){
	$out = array();
	exec('kill -9 $(pidof rfcomm)');
	print_r($out);
}


function read_file($file, $lines) {
       $handle = fopen($file, "r");
       $linecounter = $lines;
       $pos = -2;
       $beginning = false;
       $text = array();
       while ($linecounter > 0) {
         $t = " ";
         while ($t != "\n") {
           if(fseek($handle, $pos, SEEK_END) == -1) {
		$beginning = true;
		break;
	   }
           $t = fgetc($handle);
           $pos --;
         }
         $linecounter --;
         if($beginning) {
		rewind($handle);
	 }
         $text[$lines-$linecounter-1] = fgets($handle);
         if($beginning) {
		break;
	 }
       }
       fclose ($handle);
       return array_reverse($text); // array_reverse is optional: you can also just return the $text array which consists of the file's lines.
}

?>
