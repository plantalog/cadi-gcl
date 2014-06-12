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
			$("#cadi_set_srtrs").val(),
			$("#cadi_set_sppd").val()];
	settings = arr.join();
//	alert(arr);
	$.post('cm/cadi_bt_processor.php', {action: 'btd_apply_settings', settings:settings}, function(data){
		 alert(data);
	});

	var arr = [
			$("#cadi_set_svg_t3top").val(),
			$("#cadi_set_svg_t3btm").val(),
			$("#cadi_set_svg_t3wx").val(),
			$("#cadi_set_svg_t3wy").val(),
			$("#cadi_set_svg_t4top").val(),
			$("#cadi_set_svg_t4btm").val(),
			$("#cadi_set_svg_t4wx").val(),
			$("#cadi_set_svg_t4wy").val()];
	settings = arr.join();
//	alert(arr);
	$.post('cm/cadi_bt_processor.php', {action: 'svg_apply_settings', settings:settings}, function(data){
		 alert(data);
	});

}

</script>

<?php
	$temp = file_get_contents('btds/btd.conf');
	$temparr = explode(",", $temp);
	$temp = file_get_contents('svg.conf');
	$svg_arr = explode(",", $temp);
?>

<b>Cadi Bluetooth daemon tuning</b>

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
	<li>Status packet ping divider, N
		<input
			id="cadi_set_sppd"
			title="Send status request packet to Cadi every N iteration of daemon main loop"
			type="text"
			value="<?php echo $temparr[6]; ?>" />
	</li>
</ul>



<b>SVG variables</b>

<ul>
	<li>Верхний уровень бака (3), см
		<input
			id="cadi_set_svg_t3top"
			title="The tank 3 water top level, cm / Верхний уровень бака (3), см"
			type="text"
			value="<?php echo $svg_arr[0]; ?>" />
	</li>
	<li>Дно бака (3), см
		<input
			id="cadi_set_svg_t3btm"
			title="Tank 3 water bottom level. cm / Нижний уровень бака (3), см"
			type="text"
			value="<?php echo $svg_arr[1]; ?>" />
	</li>
	<li>Расположение "воды" (бак (3)) на схеме, координата X
		<input
			id="cadi_set_svg_t3wx"
			title="SVG 'water' position on Map view for Tank (3), X axis"
			type="text"
			value="<?php echo $svg_arr[2]; ?>" />
	</li>
	<li>Расположение "воды" (бак (3)) на схеме, координата Y
		<input
			id="cadi_set_svg_t3wy"
			title="SVG 'water' position on Map view for Tank (3), Y axis"
			type="text"
			value="<?php echo $svg_arr[3]; ?>" />
	</li>
	<li>Верхний уровень бака (4), см
		<input
			id="cadi_set_svg_t4top"
			title="The tank 4 water top level, cm"
			type="text"
			value="<?php echo $svg_arr[4]; ?>" />
	</li>
	<li>Дно бака (4), см
		<input
			id="cadi_set_svg_t4btm"
			title="Tank 4 water bottom level. cm / Нижний уровень бака (4), см"
			type="text"
			value="<?php echo $svg_arr[5]; ?>" />
	</li>
	<li>Расположение "воды" (бак (4)) на схеме, координата X
		<input
			id="cadi_set_svg_t4wx"
			title="SVG 'water' position on Map view for Tank (4), X axis"
			type="text"
			value="<?php echo $svg_arr[6]; ?>" />
	</li>
	<li>Расположение "воды" (бак (4)) на схеме, координата Y
		<input
			id="cadi_set_svg_t4wy"
			title="SVG 'water' position on Map view for Tank (4), Y axis"
			type="text"
			value="<?php echo $svg_arr[7]; ?>" />
	</li>
</ul>



<button class="btn_" onClick="apply_btd_settings()">Apply new settings</button>
