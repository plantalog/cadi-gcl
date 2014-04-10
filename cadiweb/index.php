<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Cadi web UI</title>
<!---
<link rel="stylesheet" href="http://code.jquery.com/ui/1.10.3/themes/smoothness/jquery-ui.css">
<script src="http://code.jquery.com/jquery-1.9.1.js"></script>
<script src="http://code.jquery.com/ui/1.10.3/jquery-ui.js"></script>
<link rel="stylesheet" href="/resources/demos/style.css">  -->
<link rel="stylesheet" href="css/resetcss.css">
<link rel="stylesheet" href="css/style.css">
<link rel="stylesheet" href="js/jquery-ui-1.10.3.custom/css/smoothness/jquery-ui-1.10.3.custom.css">
<script src="js/jquery-ui-1.10.3.custom/js/jquery-1.9.1.js"></script>
<script src="js/jquery-ui-1.10.3.custom/js/jquery-ui-1.10.3.custom.js"></script>


<script>
$(function() {
	$( ".btn_" ).button();
	$( "#cadi_tabs" ).tabs({
		beforeLoad: function( event, ui ) {
			ui.jqXHR.error(function() {
			ui.panel.html(
			"Couldn't load this tab. We'll try to fix this as soon as possible. " +
			"If this wouldn't be a demo." );
			});
		}
	});
});



function bt_connect(){
	var mac = $('#bind_mac').val();
	var rfcomm = "rfcomm"+$("#rfcomm_nr").val();
	alert('going to connect '+mac);
	$.post('cm/cadi_bt_processor.php', {action: 'bt_connect', mac:mac, rfcomm: rfcomm}, function(data){
		alert('connected'+data);
		cadi_list_rfcomms();
		$('#main_output').html(data);
	});
}

function bt_restart(){
	alert('restarting');
	$.post('cm/cadi_bt_processor.php', {action: 'bt_restart'}, function(data){
		alert('restarted');
		cadi_list_rfcomms();
		$('#main_output').html(data);
	});
}

function bt_disconnect(){
	var rfcomm = "rfcomm"+$("#rfcomm_nr").val();
	$.post('cm/cadi_bt_processor.php', {action: 'bt_disconnect', rfcomm:rfcomm}, function(data){
		alert('disconnected');
		cadi_list_rfcomms();
		$('#main_output').html(data);
	
	});
}

function get_status_block(blockId){
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 7, block_id:1}, function(data){
		// $('#status_block').html(data);
	});
	$.post('cm/cadi_bt_processor.php', {action: 'get_status'}, function(data){
		var n = data.indexOf("lock_id=1"); 
		if (n>0) {		
			out = data.split('lock_id=1')
			$('#status_block').html(out[1]);
		}
		d = new Date();
		$("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
	});
}

function cadi_bt_scan(){
//	$('#main_output').html('scanning..');
	$.post('cm/cadi_bt_processor.php', {action: 'rfcomm_scan'}, function(data){
		$('#bind_mac').html(data);
		alert('Scanning complete!');
		alert(data);
//		cadi_list_rfcomms();
	});  
}

function cw_reboot(){	// reboots the machine, running CadiWeb server
	$.post('cm/cadi_bt_processor.php', {action: 'reboot'}, function(data){
	});  
}

function cadi_list_rfcomms(){
//	alert('listing binds');
	$.post('cm/cadi_bt_processor.php', {action: 'rfcomm_list_binded'}, function(data){
		$('#binded_rfcomms').html(data);
	});
}

function stopSerialRead(psid){
//alert('call'+psid);
	$.post('cm/cadi_bt_processor.php', {action: 'stop_serial_read', process:psid}, function(data){
		cadi_list_rfcomms();
//		alert('kill -9 sent');
	});  
}

function bt_tx_packet() {
	var data = $('#tx_data_packet').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', data: data}, function(data){
		cadi_list_rfcomms();
		alert(data);
	});  
}


function bt_tx() {
	var data = $('#tx_data').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '1', plug_id:'1', state:data}, function(data){
		cadi_list_rfcomms();
//		alert('kill -9 sent');
	});  
}

function plugStateSet(plug, state){
//	alert('oga');
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '1', plug:plug, state:state}, function(data){
		cadi_list_rfcomms();
//		alert(data);
	});  
}

function bt_setdd() {
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: '2'}, function(data){
		cadi_list_rfcomms();
//		alert(data);
	});  
}

function cadi_status_stream(){
	var state = $('#flag_status_stream').is(':checked');
	if (state==1) {
		var interval = setInterval(function(){get_status_block(1)},1000);
		$('#status_stream_interval').val(interval);
	}
	if (state==0) {
		var  interval = $('#status_stream_interval').val();
//		alert(interval);
		clearInterval(interval);

	}

}
		

</script>
</head>
<body>
<input type="hidden" id="status_stream_interval" value="" />
<div id="cadi_tabs">
<ul>
<li><a href="#tabs-1">Status</a></li>
<li><a href="cm/cadi_timers.php">Timers</a></li>
<li><a href="cm/cadi_watering.php">Watering</a></li>
<li><a href="cm/cadi_fertilization.php">Fertilization</a></li>
<li><a href="cm/cadi_sensors.php">Sensors</a></li>
<li><a href="cm/cadi_execution.php">Execution</a></li>
<li><a href="cm/cadi_advanced.php">Advanced</a></li>
<li><a href="cm/cadi_dd.php">Direct drive</a></li>
</ul>
<div class="ral">
<button class="btn_ fr">Apply settings</button>
<button class="btn_ fr">Reload settings</button>
</div>
<div id="tabs-1">
<input type="checkbox" id="flag_status_stream" onClick="cadi_status_stream()" />Stream STATUS
<div id="status_block">Status appears here
	<?php include_once('cm/cadi_status.php'); ?>
</div>

	=================================================	
	<br>
	<div onclick="bt_restart();" style="display:inline; border: 1px solid red;">BTRestart</div><br>
	<div onclick="cadi_list_rfcomms();" style="display:inline; border: 1px solid red;">Refresh rfcomm list</div><br>
	<div onClick="cadi_bt_scan();" style="display:inline; border: 1px solid red;">Scan</div>
	<select id="bind_mac" name="bind_mac">
	<option>Scan to get the list</option>
	</select>
	<div onClick=bt_connect() style="display:inline; border: 1px solid red;">Connect</div>
	<div id="binded_rfcomms"></div>
	<div title="use this field to send the data to Cadi while connected" id="tx_form">
		<input type="text" name="tx_data" id="tx_data" />
		<div onClick="bt_tx_packet()">Send</div>	
	</div>


	<div title="send ZX2[1],[1],[1]" id="tx_form">
		<input type="text" name="tx_data_packet" id="tx_data_packet" />
		<div onClick="bt_tx()">Send packet</div>	
	</div>

	RFCOMM NUMBER:
	<input type="text" value="0" id="rfcomm_nr" />

<div onClick="bt_setdd()">Set DD</div>	

<div onClick="plugStateSet('1','1')">Enable P1</div>
<div onClick="plugStateSet('1','0')">Disable P1</div>
<button onClick="get_status_block('1')">Get Status</button>
</div>
</div>
</body>
</html>
