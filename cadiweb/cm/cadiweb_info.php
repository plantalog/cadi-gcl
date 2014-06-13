<?php

//	session_start();
	$out = array();
	exec('lsb_release -a', $out, $retvals);
	for ($i=0; $i<sizeof($out);$i++) {
		$server_version.=$out[$i].'<br>';
	}

	exec('ls -la /var/www/', $out);
	for ($i=0; $i<sizeof($out);$i++) {
		$cadiweb_date.=$out[$i].'<br>';
	}


//	echo $server_version


?>

<ul>
	<li><b>SERVER: </b><br>
		<?php echo $server_version; ?>
	</li>

	<li><b>Cadiweb version:</b>
		<?php echo $_SESSION['cadiweb_version']; ?>
	</li>

	<li></li>
	<li></li>
</ul>


