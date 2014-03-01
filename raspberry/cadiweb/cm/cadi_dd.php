<script>
$(function() {
	$( ".btn_" ).button();
	$( ".radio_" ).buttonset();
});

function cadi_send_command(){
	var mac = $('#bind_mac').val();
	var command = $('#cadi_command').val();
//	alert('going to send '+command+' to '+mac);
	$.post('cm/cadi_bt_processor.php', {action: 'command_send', command:command, mac:mac}, function(data){
		
//		cadi_list_rfcomms();
		$('#main_output').html(data);
//		cadi_bt_stream('rfcomm0');
//		alert('command sent');
	});
}

function cadi_send_cmd(cmd){
	var mac = $('#bind_mac').val();
	
//	alert('going to send '+command+' to '+mac);
	$.post('cm/cadi_bt_processor.php', {action: 'command_send', command:cmd, mac:mac}, function(data){
		
//		cadi_list_rfcomms();
		$('#main_output').html(data);
//		cadi_bt_stream('rfcomm0');
//		alert('command sent');
	});
}

</script>

<div id="cadi_dd_accordion">
<h3>Direct drive</h3>
<div>

<table>
<?php
$plug_amount = 4;
for ($i=0; $i<$plug_amount; $i++) {
	echo '
	<tr>
		<td>Lights zone 1</td>
		<td>
			<form class="c_inline">
			<div id="radio_plug1" class="radio_">
			<input type="radio" id="plug'.$i.'_radio1" name="plug'.$i.'_radio"><label for="plug'.$i.'_radio1" onClick="cadi_send_cmd('.($i+49).')" >ON</label>
			<input type="radio" id="plug'.$i.'_radio2" name="plug'.$i.'_radio" checked="checked"><label for="plug'.$i.'_radio2" onClick="cadi_send_cmd('.($i+50).')">OFF</label>
			</div>
			</form>
		</td>
		<td></td>
	</tr>';
}
?>

</table>

<table>
<?php
$valves_amount = 4;
for ($i=0; $i<$plug_amount; $i++) {
	echo '
	<tr>
		<td>Valve '.$i.'</td>
		<td>
			<form class="c_inline">
			<div id="radio_plug1" class="radio_">
			<input type="radio" id="valve'.$i.'_radio1" name="valve'.$i.'_radio"><label for="valve'.$i.'_radio1">Open</label>
			<input type="radio" id="valve'.$i.'_radio2" name="valve'.$i.'_radio" checked="checked"><label for="valve'.$i.'_radio2">Close</label>
			</div>
			</form>
		</td>
		<td></td>
	</tr>';
}
?>

</table>

</div>

</div>
