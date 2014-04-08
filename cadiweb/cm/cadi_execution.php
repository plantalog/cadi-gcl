<?php
include('cadi_defines.php');
?>

<script>
$(function() {
	$( ".btn" ).button();
	$( "#cadi_execution_accordion" ).accordion();
});

function enableDev(boxId) {

	var boxidbuf = boxId;
	var arr  = boxId.split('_');
	boxType = arr[2];	// valve, lights, whatever
	boxId = arr[4];		// line number
	disel_name = arr[1]+'_'+arr[2]+'_name_'+arr[4];
	disel_name2 = arr[1]+'_'+arr[2]+'_dev_id_'+arr[4];
	disel_name3 = arr[1]+'_'+arr[2]+'_type_'+arr[4];
	if ($("#"+boxidbuf).prop('checked')==true) {
		seten(disel_name, true);
		seten(disel_name2, true);
		seten(disel_name3, true);
	}
	else {
		seten(disel_name, false);
		seten(disel_name2, false);
		seten(disel_name3, false);
	}
//	togen(disel_name);
}

function seten(id, flag){
	if (flag==true) {
		$("#"+id).prop( "disabled", false );
	}
	else {
		$("#"+id).prop( "disabled", true );
	}
}

function togen(id){
	if ($("#"+id).prop( "disabled")==true) {
		$("#"+id).prop( "disabled", false );
	}
	else {
		$("#"+id).prop( "disabled", true );
	}
}

</script>
<div id="cadi_execution_accordion">

<!---
There are normally 12 execution devices to be controlled in system
4 AC loads (used as dosing pump control with PWM function)
4 DC loads
4 valves
Each device has it's own ID
4 AC Loads 1 to 4 have numbers 0 to 3 correspondingly
4 DC Loads 1 to 4 have device ids 10 to 13 correspondingly
4 Valves have device ids from 20 to 23
So, first digit in id number shows the type of device and second it's id within this type's pool, having up to 10 devices of each type.
AC loads could be used to trigger different kinds of loads, (eg heaters, lights, pumps etc). Therefore the same device ids are displayed under different tabs (devices 0 to 9 are displayed under Lights, pH, Pumps, Temperature tabs simultaneously) which causes necessarity of double-checking that the same ids are not used twice. 

-->


<h3>Valves / Клапаны</h3>
<div>
	<table>
	<?php
		for ($i=0; $i<VALVES_DEV_AMOUNT;$i++) {
			echo '<tr>';
			echo '<td><input type="checkbox" onClick=enableDev(this.id) id="enable_exec_valve_name_'.$i.'" /></td>';
			echo '<td>'.$i.'&nbsp;<input disabled id="exec_valve_name_'.$i.'" title="input valve name here" type="text" /></td>';
			echo '<td>
				<select disabled id="exec_valve_dev_id_'.$i.'">';
				for ($i2=0; $i2<VALVES_DEV_AMOUNT;$i2++) {
					echo '<option>'.($i2+20).'</option>';
				}
			echo '</select>
			</td>';
			if ($i<FEEDBACKED_VALVES_AMOUNT) {
				echo '<td>
					<select disabled id="exec_valve_type_'.$i.'">
						<option>Spherical</option>
						<option>Solenoid</option>
					</select>
				</td>';
			}
			echo '<tr>';
		}

	?>
</table>
<button class="btn">Save</button>
</div>

<h3>Pumps / Насосы</h3>
<div>

	<table>
	<?php
		for ($i=0; $i<(AC_DEV_AMOUNT+DC_DEV_AMOUNT);$i++) {
			echo '<tr>';
			echo '<td>'.$i.'&nbsp;<input id="exec_pump_name_'.$i.'" title="input pump name here" type="text" /></td>';
			echo '<td>
				<select id="exec_pump_dev_id_"'.$i.'>';
				for ($i2=0; $i2<DC_DEV_AMOUNT;$i2++) {
					echo '<option>DC load:  '.($i2+10).'</option>';
				}
				for ($i2=0; $i2<AC_DEV_AMOUNT;$i2++) {
					echo '<option>AC load: '.($i2+0).'</option>';
				}
			echo '</select></td>';
			echo '<td><button class="btn" onClick="testDev(pump_'.$i.');">test</button></td>';
			echo '<tr>';
		}

	?>
</table>
<button class="btn">Save</button>

</div>
<h3>Lights / Свет</h3>
<div>
<table>
	<?php
		for ($i=0; $i<(AC_DEV_AMOUNT);$i++) {
			echo '<tr>';
			echo '<td>'.$i.'&nbsp;<input id="exec_light_name_'.$i.'" title="input lights name here" type="text" /></td>';
			echo '<td>
				<select id="exec_pump_dev_id_"'.$i.'>';
				for ($i2=0; $i2<AC_DEV_AMOUNT;$i2++) {
					echo '<option>AC load: '.($i2+0).'</option>';
				}
			echo '</select></td>';
			echo '<td><button class="btn" onClick="testDev(light_'.$i.');">test</button></td>';
			echo '<tr>';
		}
	?>
</table>
</div>

<h3>Humidity / Влажность</h3>
<div>
	<table>
	<?php
		for ($i=0; $i<(AC_DEV_AMOUNT+DC_DEV_AMOUNT);$i++) {
			echo '<tr>';
			echo '<td>'.$i.'&nbsp;<input id="exec_humid_name_'.$i.'" title="input humidifier/dehumidifier name here" type="text" /></td>';
			echo '<td>
				<select id="exec_humid_dev_id_"'.$i.'>';
				for ($i2=0; $i2<DC_DEV_AMOUNT;$i2++) {
					echo '<option>DC load:  '.($i2+10).'</option>';
				}
				for ($i2=0; $i2<AC_DEV_AMOUNT;$i2++) {
					echo '<option>AC load: '.($i2+0).'</option>';
				}
			echo '</select></td>';
			echo '<td><select id="humid_correction_direction">
					<option value="0">--</option>
					<option value="1">Up</option>
					<option value="2">Down</option>
				</select></td>
			';
			echo '<td><button class="btn" onClick="testDev(humid_'.$i.');">test</button></td>';
			echo '<tr>';
		}

	?>
</table>
<button class="btn">Save</button>

</div>

<h3>Temperature / Температура</h3>
<div>

</div>

<h3>pH / кислотность</h3>
<div>

</div>

<h3>CO2</h3>
<div>

</div>

<h3>Fertilizers / Удобрения</h3>
<div>

</div>

</div>

