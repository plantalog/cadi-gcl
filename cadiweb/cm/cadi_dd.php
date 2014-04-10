<script>
$(function() {
	$( ".btn_" ).button();
	$( ".radio_" ).buttonset();
});

function bt_setdd(state) {
	if (state==1) {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '2'}, function(data){
			cadi_list_rfcomms();
		});
	}
	else {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '3'}, function(data){
				cadi_list_rfcomms();
		});
	}
} 

function open_valve(valve){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 4, valve:valve}, function(data){
		$('#main_output').html(data);
	});
}

function close_valve(valve){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 5, valve:valve}, function(data){
		$('#main_output').html(data);
	});
}

function cadi_send_command(){
	var mac = $('#bind_mac').val();
	var command = $('#cadi_command').val();
	$.post('cm/cadi_bt_processor.php', {action: 'command_send', command:command, mac:mac}, function(data){
		$('#main_output').html(data);
	});
}

function cadi_send_cmd(cmd){
	var mac = $('#bind_mac').val();
	$.post('cm/cadi_bt_processor.php', {action: 'command_send', command:cmd, mac:mac}, function(data){
		$('#main_output').html(data);
	});
}

function cdd_toggle(){
	var state = $('#cdd_enabled').is(':checked');
	if (state==1) {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '2'}, function(data){
			cadi_list_rfcomms();
		});  
	}
	else {
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '3'}, function(data){
			cadi_list_rfcomms();
		});  
	}
//	alert(data);
}

function plugStateSet(plug, state){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '1', plug:plug, state:state}, function(data){
		cadi_list_rfcomms();
//		alert(data);
	});  
}

</script>

<div id="cadi_dd_accordion">
<h3>Direct drive</h3>
	<div>	
	<input type="checkbox" id="cdd_enabled" onClick="cdd_toggle(this)"> Cadi Direct Drive enabled
	<table>
	<?php
	$plug_amount = 4;
	for ($i=0; $i<$plug_amount; $i++) {
		echo '
		<tr>
			<td>Lights zone '.$i.'</td>
			<td>
				<form class="c_inline">
				<div id="radio_plug1" class="radio_">
				<input type="radio" id="plug'.$i.'_radio1" name="plug'.$i.'_radio"><label for="plug'.$i.'_radio1" onClick="plugStateSet('.$i.',1)" >ON</label>
				<input type="radio" id="plug'.$i.'_radio2" name="plug'.$i.'_radio" checked="checked"><label for="plug'.$i.'_radio2" onClick="plugStateSet('.$i.',0)">OFF</label>
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
				<input type="radio" id="valve'.$i.'_radio1" name="valve'.$i.'_radio"><label for="valve'.$i.'_radio1" onClick="open_valve('.$i.')">Open</label>
				<input type="radio" id="valve'.$i.'_radio2" name="valve'.$i.'_radio" checked="checked"><label for="valve'.$i.'_radio2" onClick="close_valve('.$i.')">Close</label>
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

<div>
	<h1>Get water</h1>
Valve: 	<select id="f_gw_valve_id">
		<option value="0">0</option>
		<option value="1">1</option>
		<option value="2">2</option>
		<option value="3">3</option>
	</select>

Counter: 	<select id="f_gw_counter_id">
		<option value="0">0</option>
		<option value="1">1</option>
		<option value="2">2</option>
		<option value="3">3</option>
	</select>
<br>
<input type="text" title="enter amount of water you want to get (in CL, integer value from 0 to 65535)" id="f_gw_amount" />
<button>Get water</button>
</div>

