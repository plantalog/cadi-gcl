<!-- 
This block gives user ability to tune the Cadi Bluetooth Daemon settings

-->

<script>
function apply_btd_settings(){
	var arr = [$("#cadi_set_pfd").val(),
			$("#cadi_set_csd").val(),
			$("#cadi_set_fsd").val(),
			$("#cadi_set_rpd").val(),
			$("#cadi_set_cn").val(),
			$("#cadi_set_srtrs").val()];
	settings = arr.join();
	alert(arr);
	$.post('cm/cadi_bt_processor.php', {action: 'btd_apply_settings', settings:settings}, function(data){
		alert(data);
	});
}

</script>

<?php
	$temp = file_get_contents('btds/btd.conf');
	$temparr = explode(",", $temp);
?>

Cadi Bluetooth daemon tuning

<ul>
	<li>Photoshot freq divider, N
		<input
			id="cadi_set_pfd"
			title="make a photoshot every N iteration of Command Scan Cycle (CSC)"
			type="text"
			value="<?php echo $temparr[0]; ?>" />
	</li>
	<li>Command scan delay, us
		<input
			id="cadi_set_csd"
			title="25000=25ms~=40scans per second"
			type="text"
			value="<?php echo $temparr[1]; ?>" />
	</li>
	<li>File size difference, X
		<input
			id="cadi_set_fsd"
			title="attmpt to parse response if filesize increased at least for X bytes"
			type="text"
			value="<?php echo $temparr[2]; ?>" />
	</li>
	<li>Response parser divider, N
		<input
			id="cadi_set_rpd"
			title="attmpt to parse response every N-th CSC"
			type="text"
			value="<?php echo $temparr[3]; ?>" />
	</li>
	<li>/dev/videoX camera number
		<input
			id="cadi_set_cn"
			title="attmpt to parse response every N-th CSC"
			type="text"
			value="<?php echo $temparr[4]; ?>" />
	</li>
	<li>Serial response tail read size
		<input
			id="cadi_set_srtrs"
			title="Data amount to be read from the end "
			type="text"
			value="<?php echo $temparr[5]; ?>" />
	</li>
</ul>
<button class="btn_" onClick="apply_btd_settings()">Apply new settings</button>
