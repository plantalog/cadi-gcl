<?php

include_once('cm/cadi_response_parser.php');
include_once('cadi_response_parser.php');
	echo '
	PHP time: '.date("Y-m-d H:i:s", time()).'
	<p>Cadi status</p>
	<table>
	<tr>
		<td>Cadi Time</td>
		<td>'.date("Y-m-d H:i:s", $_SESSION['cadi_status']['time']).'</td>
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
		<td>Valve states</td>
		<td>'.$_SESSION['cadi_status']['valves'].'</td>
	</tr>
	<tr>
		<td>Water levels</td>
		<td>'.$_SESSION['cadi_status']['sonar_read'][0].' / '.$_SESSION['cadi_status']['sonar_read'][1].'</td>
	</tr>
	<tr>
		<td>Temperature</td>
		<td>'.$_SESSION['cadi_status']['dht']['temp'].' C</td>
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
