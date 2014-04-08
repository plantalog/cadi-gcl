<?php 
$_SESSION['gcl']['settings']['valves']['amount'] = 3;
?>

<script>
$(function() {
	$( ".btn_" ).button();
	$( ".radio_" ).buttonset();
});
</script>

<b title="since the distance meter installed on top, reading the distance from up down to air/water border, distance is less = more water in tank">Tank Level Keeper</b>
<div>
	<table class="table_advanced">
		<tr>
			<td>Enable TLK</td>
			<td>
				<form class="c_inline">
				<div id="radio_plug1" class="radio_">
				<input type="radio" id="tlk_enable_radio1" name="tlk_enable_radio"><label for="tlk_enable_radio1">ON</label>
				<input type="radio" id="tlk_enable_radio2" name="tlk_enable_radio" checked="checked"><label for="tlk_enable_radio2">OFF</label>
				</div>
				</form>
			</td>
		</tr>
		<tr title="current distance to air/water border">
			<td><b>Current distance</b></td>
			<td>64</td>
		</tr>
		<tr>
			<td><b>Keep level</b></td>
			<td title="current distance to air/water border">50</td>
			<td><input title="manually set new TLK level value to keep" type="text" /><button>Set</button></td>
		</tr>
		<tr title="lowest possible water level sonar reading. could be calibrated or manually set">
			<td><b>Distance to bottom</b></td>
			<td>105</td>
			<td><input title="manually set new TLK level value to keep" type="text" /><button>Set</button></td>
		</tr>
		<tr title="the minimum distance between sensor and water border - the top tank level">
			<td><b>Distance to top</b></td>
			<td>20</td>
			<td><input title="manually set new TLK level value to keep" type="text" /><button>Set</button></td>
		</tr>
		<tr title="valve id operating the tank water supply">
			<td><b>Supply valve</b></td>
			<td>1</td>
			<td>
				<select>
<?php 
	for ($i=0; $i<$_SESSION['gcl']['settings']['valves']['amount']; $i++) {
		echo '<option value="'.$i.'">Valve '.$i.'</option>';
	}
?>				</select>
				<button>Set</button>
			</td>
		</tr>
	
	</table>
</div>
