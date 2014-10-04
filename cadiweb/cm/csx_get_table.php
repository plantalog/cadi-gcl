	<?php
	include_once('cadi_settings.php');
	sync_dump2conf();
	unset($cs_arr);
	$cs_arr = array();
	if (($handle = fopen("cadi_settings_conf.csv", "r")) !== FALSE) {
	    $sca = array();	// settings csv array
	    $row = 0;
	    while (($data = fgetcsv($handle, 1000, ",")) !== FALSE) {
		if (sizeof($data)>1) {		// skip empty lines if found
			$cs_arr[$row++] = $data;		// aka $sca
		}
	    }
	    fclose($handle);
	}

		echo '<table>';

		for ($i=0;$i<sizeof($cs_arr);$i++) {
			$idpref = 'csx_'.$cs_arr[$i][0].'_';
			echo '<tr>';
				echo '<td>'.$cs_arr[$i][0].'</td>';	// Address
				echo '<td><select name="'.$idpref.'type">';
					for ($i2=1; $i2<4;$i2++) {
						$selected[$i2]='';
						if ($i2==$cs_arr[$i][1]) {
							$selected[$i2]='selected';
						}
						else {
							$selected[$i2]='';
						}
					}
					echo '
						<option value="1" '.$selected[1].'>8 bit</option>
						<option value="2" '.$selected[2].'>16 bit</option>
						<option value="3" '.$selected[3].'>32 bit</option>
					</select></td>';				// type
				echo '<td>'.$cs_arr[$i][1].'</td>';				// value
				echo '<td><input type="text" onChange="rx_ee('.(intval(substr($idpref,4,5))).')" name="'.$idpref.'value" value="'.$cs_arr[$i][2].'" /></td>';				// value
				echo '<td><input style="width:600px" type="text" name="'.$idpref.'text" value="'.$cs_arr[$i][3].'" /></td>';				// text description
			echo '</tr>';
				

		}


		echo '</table>';
	?>
