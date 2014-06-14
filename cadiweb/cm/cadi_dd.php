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

function auto_flags(flags){	// set new auto_flags byte
	if (flags == 256) {
		flags = $('#auto_flags_input').val();
		// alert('Custom flags='+flags+' ('+flags.toString(2)+')');
	}
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 8, flags:flags}, function(data){
		$('#main_output').html(data);
	});
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
function reset_valve_fails(){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 10}, function(data){
		$('#main_output').html(data);
	});
}
function cadi_txcmd(cmd){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: cmd}, function(data){
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
}

function plugStateSet(plug, state){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '1', plug:plug, state:state}, function(data){
		cadi_list_rfcomms();
	});  
}

function enable_dosing_pump(pumpId, state){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '9', pump_id:pumpId, state:state}, function(data){
		cadi_list_rfcomms();
//		alert(data);
	});  
}

function run_doser_for(){
	var doserId = $('#fert_selector').val();
	var amount = $('#fert_amount').val();
	alert('Going to add fertilizer '+doserId+' within '+amount+' seconds');
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '9', pump_id:doserId, amount:amount}, function(data){
//		alert(data);
	}); 
}

</script>


<div id="cadi_dd_accordion">
	<div>	
	<input type="checkbox" id="cdd_enabled" onClick="cdd_toggle(this)"> CDD: Cadi Direct Drive enabled
	<table>
	<?php

	$row = 1;
	if (($handle = fopen("cm/cadi_settings", "r")) !== FALSE) {
	    while (($_SESSION['settings_data'][$row] = fgetcsv($handle, 1000, ",")) !== FALSE) {
		$num = count($_SESSION['settings_data'][$row]);
		$row++;
	    }
	    fclose($handle);
	}


	$plug_amount = 4;

	echo '<tr><td><b>LOADS:</b></td></tr>';
	for ($i=0; $i<$plug_amount; $i++) {
		echo '
		<tr title="'.$_SESSION['settings_data'][2][$i+1].'">
			<td>'.$_SESSION['settings_data'][1][$i+1].'</td>
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

	
	echo '<tr><b>VALVES:</b></tr>';
	for ($i=0; $i<$valves_amount; $i++) {
		echo '
		<tr title="'.$_SESSION['settings_data'][4][$i+1].'">
			<td>'.$_SESSION['settings_data'][3][$i+1].'</td>
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

/*	$dosers_amount = 4;
	for ($i=0; $i<$dosers_amount; $i++) {
		echo '
		<tr title="'.$_SESSION['settings_data'][6][$i+1].'">
			<td>'.$_SESSION['settings_data'][5][$i+1].'</td>
			<td>
				<form class="c_inline">
				<div id="radio_doser'.$i.'" class="radio_">
				<input type="radio" id="doser'.$i.'_radio1" name="doser'.$i.'_radio"><label for="doser'.$i.'_radio1" onClick="enable_dosing_pump('.$i.','.$_SESSION['settings_data'][7][$i+1].')">Run</label>
				<input type="radio" id="doser'.$i.'_radio2" name="doser'.$i.'_radio" checked="checked"><label for="doser'.$i.'_radio2" onClick="enable_dosing_pump('.$i.',0)">Stop</label>
				</div>
				</form>
			</td>
			<td></td>
		</tr>';
	} */


	?>


	<tr><td colspan="2">
		<?php
			echo '<div style="border: 1px solid red;">';
			echo '<b style="color: red;">Fertilizer mixer</b><br>';
			echo '<select id="fert_selector">';

			$dosers_amount = 4;

			for ($i=0; $i<$dosers_amount; $i++) {
				echo '
					<option value="'.$i.'" title="'.$_SESSION['settings_data'][6][$i+1].'">'.$i.': '.$_SESSION['settings_data'][5][$i+1].'</option>';
			}
			echo '</select>';
			echo '<input
					id="fert_amount"
					style="width:40px;"
					type="text"
					value="0"
					title="how much seconds to run the dosing pump"
				/>';
			echo '<button onClick=run_doser_for()>Add</button>';
			echo '</div>';

		?>		

		<button class="btn_" onClick="reset_valve_fails()">Fails=0</button>
		<button class="btn_" onClick="auto_flags(0)">Auto=0</button><br>
		<button class="btn_" onClick="auto_flags(255)">Auto=255</button>
		<button class="btn_" onClick="cadi_txcmd(11)">Force DOWN</button><br>
		<button class="btn_" onClick="cadi_txcmd(12)">Set time</button>
		<br>

		<input type="text" value="254" id="auto_flags_input" />
		<button onClick=auto_flags(256)>Set auto flags</button>

	</td></tr>
	<tr><td></td></tr>
	<tr><td></td></tr>

	</table>

	</div>
</div>
<!--
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
-->
=====================
