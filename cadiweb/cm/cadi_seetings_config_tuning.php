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
		alert('Settings saved');
	});
}

function download_csx(){
	$.post('cm/cadi_bt_processor.php', {action: 'download_csx'}, function(data){
		var interval_csxdl = setInterval(function(){csx_download()},1000);	
		$('#csxdl_interval').val(interval_csxdl);	
	});
}

function csx_download(){
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


</script>

<br><br>
<input type="hidden" value="0" id="csxdl_interval" />


<b>=== Cadi EEPROM Settings Tuning ===</b>

<button onClick="download_csx()">Download settings from Cadi</button>

<form id="csxform" name="csxform" action="">

<?php include('csx_get_table.php'); ?>


</form>
<button onClick="upload_csx()">Upload new settings to Cadi</button>
<br><br><br>


