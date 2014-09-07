<!-- 
This block gives user ability to tune the Cadi Bluetooth Daemon settings

-->

<script>
$(document).ready(function() {
	$('#csxform').submit(function (e) {
   	  $('#csxform').click();
	     e.preventDefault();
	});
});

function upload_csx(){
	var csx_data = $('#csxform').serialize();
	$.post('cm/cadi_bt_processor.php', {action: 'upload_csx', csx_data:csx_data}, function(data){
		
	});
}

function download_csx(){
	$.post('cm/cadi_bt_processor.php', {action: 'download_csx'}, function(data){
		var interval_csxdl = setInterval(function(){csx_dl_proc()},1000);	
		$('#csxdl_interval').val(interval_csxdl);	
	});
}

function csx_dl_proc(){
	var btdStateStr = $('#btd_state').html();
	var btdState = btdStateStr.charAt(0);
	$('#csxform').html('<b style="font-size: 2em;">LOADING NEW SETTINGS DATA!!!<b>');
	if (btdState == "1") {
		var interval_csxdl = $('#csxdl_interval').val();
		clearInterval(interval_csxdl);
		$.post('cm/csx_get_table.php', {action: 'download_csx'}, function(data){
			// alert('reloaded');
			$('#csxform').html(data);
		});
	}
}

function rx_ee(addr){
//	alert(addr);
	var csx_data = $('#csxform').serialize();
	$.post('cm/cadi_bt_processor.php', {action: 'upload_csx', csx_data:csx_data}, function(data){
		var number = $('#settings_number').val();	// amount of 16 bit variables to upload to cadi
		$.post('cm/cadi_bt_processor.php', {action: 'rx_ee', addr:addr, number:2}, function(data){});	// 2 vars - HARDCODE	
	});
}

</script>

<br><br>
<input type="hidden" value="0" id="csxdl_interval" />


<b>=== Cadi EEPROM Settings Tuning ===</b>

<button onClick="download_csx()">Download settings from Cadi</button>

<form id="csxform" name="csxform" action="">

<?php include('csx_get_table.php'); ?>


</form>
<br><br><br>


