<?php

include_once('cm/cadi_response_parser.php');
include_once('cadi_response_parser.php');
	echo '
	<p>Cadi status</p>
	<table>
	<tr>
		<td>Cadi Time</td>
		<td>'.$_SESSION['cadi_status']['time'].'</td>
	</tr>
	<tr>
		<td>220V Plugs</td>
		<input type="hidden" value="'.$_SESSION['cadi_status']['plugs'].'" />
		<td>'.$_SESSION['cadi_status']['plugs'].'</td>
	</tr>
	<tr>
		<td>12V plugs</td>
		<td>'.$_SESSION['cadi_status']['dosingPumpsFlags'].'</td>
	</tr>
	<tr>
		<td>Water levels</td>
		<td>sonar data</td>
	</tr>
	<tr>
		<td>Temperature</td>
		<td>'.(($dht[2]*256+$dht[3])/10).' C</td>
	</tr>
	<tr>
		<td>Humidity</td>
		<td>'.$_SESSION['cadi_status']['dht']['rh'].' %</td>
	</tr>
	<tr>
		<td>pH</td>
		<td></td>
	</tr>
	<tr>
		<td>EC</td>
		<td></td>
	</tr>
	</table>';

//	echo '<img id="cadi_img" src="img/curimage.jpeg?'.time().'" />';

?>
