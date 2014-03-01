<?php
	session_start();
	$stngs = $_SESSION['cadi_settings'];
?>

<script>
$(function() {
	$( ".btn_cadi_timer_enabled" ).button();
	$( "#cadi_timers_accordion" ).accordion();
});
</script>
<div id="cadi_timers_accordion">
<h3>Daily timers</h3>
<div>
<table>
<?php
for ($i=0; $i<10;$i++) {
	echo '
	<tr>
		<td>Timer '.$i.'</td>
		<td>ON
			<input type="text" value="'.$stngs['timers'][$i]['on'].'" />
		</td>
		<td>OFF
			<input type="text" value="'.$stngs['timers'][$i]['off'].'" />
		</td>
		<td>
			<input class="btn_cadi_timer_enabled" id="btn_cadi_timer_n'.$i.'" type="checkbox"><label for="btn_cadi_timer_n'.$i.'">Enable</label>
		</td>
	</tr>';
}

?>
</table>
</div>
<h3>Cyclic timers</h3>
<div>
<table>
<?php
for ($i=0; $i<10;$i++) {
	echo '
	<tr>
		<td>Cyclic Timer '.$i.'</td>
		<td>Duration
			<input type="text" />
		</td>
		<td>Interval
			<input type="text" />
		</td>
		<td>
			<button class="btn_cadi_timer" id="btn_cadi_ctimer_n'.$i.'" onClick="clk()">ENABLED</button>
		</td>
	</tr>';
}

?>
</table>
</div>

</div>

