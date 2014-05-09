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
		$( "#cadi_tabs" ).tabs({
			beforeLoad: function( event, ui ) {
				ui.jqXHR.error(function() {
				ui.panel.html(
				"Couldn't load this tab. We'll try to fix this as soon as possible. " +
				"If this wouldn't be a demo." );
				});
			}
		});

		$( ".btn_" ).button();
		$("#system_view_1").hide();
		get_status_block();
		cadi_list_rfcomms();
	});

	function get_ip(){
		$.post('cm/cadi_bt_processor.php', {action: 'get_ip'}, function(data){
			alert(data);
		});
	}

	function change_video(){
		var new_video =$("#video_stream").val();
		$.post('cm/cadi_bt_processor.php', {action: 'change_video', new_video:new_video}, function(data){
			alert('new video='+new_video+' ----- '+data);
		});
	}

	function bt_connect(){
		var mac = $('#bind_mac').val();
		var rfcomm = "rfcomm"+$("#rfcomm_nr").val();
//		alert('going to connect '+mac);
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

	function rcmd(cmd){		// packet created directly in packet processor
		$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: cmd}, function(data){
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

	function get_status_block(){
		var blocks = $('#status_block_ids').val();
		var block_ids = blocks.split(',');
		for (i=0; i<block_ids.length;i++){
//			$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd: 7, block_id:block_ids[i]}, function(data){
//			});
			$.post('cm/cadi_bt_processor.php', {action: 'get_status'}, function(data){
				blocks = data.split('---socalledseparator---');
				$('#status_block').html(blocks[0]);
				$('#system_view_2').html(blocks[1]);
				d = new Date();
				$("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
			});
		}
	}

	function check_plug(){
		alert("checking!");
		$(function(){
			$('#radio_plug1').attr('checked','checked');
			$('#plug1_radio').attr('checked','checked');
			$('#plug1_radio1').attr('checked','true');
			$("radio_").buttonset("refresh");
			$("#radio_plug1").buttonset("refresh");
		});
	}

	function cadi_bt_scan(){
	//	$('#main_output').html('scanning..');
		$.post('cm/cadi_bt_processor.php', {action: 'rfcomm_scan'}, function(data){
			$('#bind_mac').html(data);
	//		alert('Scanning complete!');
	//		alert(data);
	//		cadi_list_rfcomms();
		});  
	}

	function cw_reboot(){	// reboots the machine, running CadiWeb server
		if (confirm('Reboot Cadi server?')) {
			$.post('cm/cadi_bt_processor.php', {action: 'reboot'}, function(data){
			}); 
		}
	}

	function cadi_reset(){
		if (confirm('Reset Cadi?')) {
			rcmd(13);
		}
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
			var interval = setInterval(function(){get_status_block()},1200);
			$('#status_stream_interval').val(interval);
		}
		if (state==0) {
			var  interval = $('#status_stream_interval').val();
	//		alert(interval);
			clearInterval(interval);
		}
	}

	
function cadi_view(viewId){
	switch (viewId) {
		case 1:
			$("#system_view_1").show();
			$("#system_view_2").hide();
			break;
		case 2:
			$("#system_view_2").show();
			$("#system_view_1").hide();
			break;
	} 

}

function btd_stream_status(newStatus){
	//	alert();
		$.post('cm/cadi_bt_processor.php', {action: 'btd_stream_start', status: newStatus}, function(data){
	//		cadi_list_rfcomms();
			alert('Streaming to server cache');
		});  
}

function eeRead() {
	var addr = $('#ee_addr').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', cmd:18, addr:addr}, function(data){
		alert('EEPROM value read');
		var out = data.split('seppy');
		$('ee16bit_value').val(out[0]);
		$('ee32bit_value').val(out[1]);
	});  
}

function eeWrite(dataType) {
	var addr = $('#ee_addr').val();
	var cmd=0;
	switch ($dataType) {
		case 1:
			var value = $('#ee16bit_value').val();
			cmd = 15;
			break;
		case 2:
			var value = $('#ee32bit_value').val();
			cmd = 16;
			break;
	}
	$.post('cm/cadi_bt_processor.php', {action: 'tx_packet', data_type:dataType, value:value, cmd:cmd}, function(data){
		alert('EEPROM value sent');
	});  
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
<!--<li><a href="cm/cadi_dd.php">Direct drive</a></li> -->
</ul>
<div class="ral">
<button class="btn_ fr" onClick="check_plug()">Apply settings</button>
<button class="btn_ fr">Reload settings</button>
</div>
<div id="tabs-1">
<input type="checkbox" id="flag_status_stream" onClick="cadi_status_stream()" />Stream STATUS (from server cache)
<button class="btn_" id="cadi_view_cam" onClick="cadi_view(1)">Cam</button>
<button class="btn_" id="cadi_view_map" onClick="cadi_view(2)">Map</button>
<br>
<table>
	<tr>
		<td style="vertical-align:top;">
			<div id="system_view_1">
				<div id="status_block" style="float:left;">
					<?php include_once('cm/status_view_1.php'); ?>
				</div>
				<img id="cadi_img" style="float:right;" src="img/curimage.jpeg?" />
			</div>
			<div id="system_view_2" style="border: 1px solid blue;">
				<?php include_once('cm/cadi_status.svg'); ?>
			</div>
		</td>
		<td>
			<div style="float:right; border: 1px solid blue;">
				<?php include_once('cm/cadi_dd.php'); ?>
			</div>
		</td>
	</tr>	
</table>

	=================================================	
	<br>
<!--	<div onclick="bt_restart();" style="display:inline; border: 1px solid red;">BTRestart</div>  -->
	<br> 
	<button onclick="cadi_list_rfcomms();" style="display:inline; border: 1px solid red;">Refresh rfcomm list</button><br>
	<button onClick="cadi_bt_scan();" style="display:inline; border: 1px solid red;">Scan</button>
	<select id="bind_mac" name="bind_mac">
	<option>Scan to get the list</option>
	</select>
	<button onClick=bt_connect() style="display:inline; border: 1px solid red;">Connect</button>
	<div id="binded_rfcomms"></div>
	<button class="btn_" onClick="btd_stream_status(1);">BTD status stream ON</button>
	<button class="btn_" onClick="btd_stream_status(0);">BTD status stream OFF</button>
<!--	<div title="use this field to send the data to Cadi while connected" id="tx_form">
		<input type="text" name="tx_data" id="tx_data" />
		<div onClick="bt_tx_packet()">Send</div>	
	</div> 


  	<div title="send ZX2[1],[1],[1]" id="tx_form">
		<input type="text" name="tx_data_packet" id="tx_data_packet" />
		<div onClick="bt_tx()">Send packet</div>	
	</div> -->

	RFCOMM NUMBER:
	<input type="text" value="0" id="rfcomm_nr" /><br>

	<input type="text" value="254" id="auto_flags" />
	<button onClick=auto_flags(256)>Set new flags</button>
	<br>

<!--  <div onClick="bt_setdd()">Set DD</div>	

<div onClick="plugStateSet('1','1')">Enable P1</div>
<div onClick="plugStateSet('1','0')">Disable P1</div>  -->
<br>
<input type="text" value="1" id="status_block_ids" />
<button onClick="get_status_block()">Get Status</button>
<br>
<button onClick="get_ip()">Get IP</button>
<button onClick="cw_reboot()">Reboot server</button><br>
<button onClick="cadi_reset()">Cadi reset</button><br>

<!-- 
EEPROM read/write section
-->

EEPROM<br>
Address: <input type="text" id="ee_addr" /><br>
16bit:
<input type="text" id="ee16bit_value" /><br>
<button onClick="eeRead()">Read</button>
<button onClick="eeWrite(1)">Write</button><br>
32bit:
<input type="text" id="ee32bit_value" /><br>
<button onClick="eeRead()">Read</button>
<button onClick="eeWrite(2)">Write</button>
<br>

<input type="text" value="0" id="video_stream" title="this value is N in '/dev/videoN'" /><button onClick="change_video()">Change video</button>
</div>
</div>
</body>
</html>
