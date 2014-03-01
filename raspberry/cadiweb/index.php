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

/* function cadi_mac_stream(){
	var mac = $('#bind_mac').val();

	alert('going to stream '+mac);
	$.post('cm/cadi_bt_processor.php', {action: 'mac_stream', mac:mac}, function(data){
		alert('streaming');
		cadi_list_rfcomms();
		$('#main_output').html(data);
	});
} */

function bt_connect(){
	var mac = $('#bind_mac').val();
	var rfcomm = "rfcomm0";
	alert('going to connect '+mac);
	$.post('cm/cadi_bt_processor.php', {action: 'bt_connect', mac:mac, rfcomm: rfcomm}, function(data){
		alert('connected'+data);
		cadi_list_rfcomms();
		$('#main_output').html(data);
	});
}

function bt_disconnect(){
	$.post('cm/cadi_bt_processor.php', {action: 'bt_disconnect'}, function(data){
		alert('disconnected');
		cadi_list_rfcomms();
		$('#main_output').html(data);
	
	});
}

function cadi_bt_scan(){
//	$('#main_output').html('scanning..');
	$.post('cm/cadi_bt_processor.php', {action: 'rfcomm_scan'}, function(data){
		$('#bind_mac').html(data);
		alert('Scanning complete!');
		cadi_list_rfcomms();
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
	var data = $('#tx_data').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', data: data}, function(data){
		cadi_list_rfcomms();
//		alert('kill -9 sent');
	});  
}

</script>
</head>
<body>
<div id="cadi_tabs">
<ul>
<li><a href="#tabs-1">Status</a></li>
<li><a href="cm/cadi_timers.php">Timers</a></li>
<li><a href="cm/cadi_watering.php">Watering</a></li>
<li><a href="cm/cadi_fertilization.php">Fertilization</a></li>
<li><a href="cms">Sensors</a></li>
<li><a href="cm/cadi_advanced.php">Advanced</a></li>
<li><a href="cm/cadi_dd.php">Direct drive</a></li>
</ul>
<div class="ral">
<button class="btn_ fr">Apply settings</button>
<button class="btn_ fr">Reload settings</button>
</div>
<div id="tabs-1">
	<p>Cadi status</p>
	<table>
	<tr>
		<td>Cadi Time</td>
		<td></td>
	</tr>
	<tr>
		<td>220V Plugs</td>
		<td></td>
	</tr>
	<tr>
		<td>12V plugs</td>
		<td>%</td>
	</tr>
	<tr>
		<td>Water levels</td>
		<td></td>
	</tr>
	<tr>
		<td>Temperature</td>
		<td></td>
	</tr>
	<tr>
		<td>Humidity</td>
		<td></td>
	</tr>
	<tr>
		<td>pH</td>
		<td></td>
	</tr>
	<tr>
		<td>EC</td>
		<td></td>
	</tr>
	</table>
	=================================================	
	<br>
	<div onclick="cadi_list_rfcomms();" style="display:inline; border: 1px solid red;">Refresh rfcomm list</div>
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


</div>
</div>
</body>
</html>
