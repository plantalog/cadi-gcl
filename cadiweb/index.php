<?php
session_start();
$_SESSION['cadiweb_version'] = '1.0';

?>

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
<link rel="stylesheet" type="text/css" href="css/jquery.svg.css"> 
<script type="text/javascript" src="js/svg/jquery.svg.js"></script>
<!-- <script type="text/javascript" src="js/date.format.js"></script> -->

<?php
// load SVG settings
		if (($handle = fopen("cm/svg.conf", "r")) !== FALSE) {
		    	$data = fgetcsv($handle, 1000, ",");
		    	$svg_t3top = $data[0];
		    	$svg_t3btm  = $data[1];
		    	$svg_t3wx = $data[2];
		    	$svg_t3wy  = $data[3];

		    	$svg_t4top = $data[4];
		    	$svg_t4btm  = $data[5];
		    	$svg_t4wx = $data[6];
		    	$svg_t4wy  = $data[7];
		    fclose($handle);
		}
		else {
		    	$svg_t3top = 0;
		    	$svg_t3btm  = 0;
		    	$svg_t3wx = 0;
		    	$svg_t3wy  = 0;

		    	$svg_t4top = 0;
		    	$svg_t4btm  = 0;
		    	$svg_t4wx = 0;
		    	$svg_t4wy  = 0; 
/*		    	$svg_t3top = 0;
		    	$svg_t3btm  = 100;
		    	$svg_t3wx = 728;
		    	$svg_t3wy  = 193;

		    	$svg_t4top = 0;
		    	$svg_t4btm  = 100;
		    	$svg_t4wx = 586;
		    	$svg_t4wy  = 393; */
		}
?>

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
		$('#svg_container').svg({onLoad: drawMapLayer});

		$( ".btn_" ).button();
		$("#system_view_1").hide();
//		get_status_block();
		cadi_list_rfcomms();
		cadi_status_stream();
	});

	function drawMapLayer(svg){

		// valves
		svg.circle(645, 43, 15, {fill: 'red', stroke: 'blue', strokeWidth: 0, opacity: 0.5, id:'cv0'});
		svg.circle(783, 210, 15, {fill: 'red', stroke: 'blue', strokeWidth: 0, opacity: 0.5, id:'cv1'});
		svg.circle(540, 305, 15, {fill: 'red', stroke: 'blue', strokeWidth: 0, opacity: 0.5, id:'cv2'});
		svg.circle(540, 333, 15, {fill: 'red', stroke: 'blue', strokeWidth: 0, opacity: 0.5, id:'cv3'});

		// draw plug rectangles
		var plg_x = 730;
		var plg_y = 360;
		var plg_s = 30;
		svg.rect((plg_x+0*plg_s), plg_y, plg_s, plg_s, 3, 3, {fill: 'red', opacity: 0.5, id:"cp0"});
		svg.rect((plg_x+1*plg_s), plg_y, plg_s, plg_s, 3, 3, {fill: 'red', opacity: 0.5, id:"cp1"});
		svg.rect((plg_x+2*plg_s), plg_y, plg_s, plg_s, 3, 3, {fill: 'red', opacity: 0.5, id:"cp2"});
		svg.rect((plg_x+3*plg_s), plg_y, plg_s, plg_s, 3, 3, {fill: 'red', opacity: 0.5, id:"cp3"});
		svg.text((plg_x+20),(plg_y-10), 'Plugs',{fill: 'red', strokeWidth: 0});

		// draw dosing pump rectangles
		var plg_x = 618;
		var plg_y = 167;
		var plg_s = 13;
		svg.rect((plg_x+0*plg_s), plg_y, plg_s, plg_s, 3, 3, {fill: 'red', opacity: 0.5, id:"cdp1"});
		svg.rect((plg_x+1*plg_s), plg_y, plg_s, plg_s, 3, 3, {fill: 'red', opacity: 0.5, id:"cdp2"});
		svg.rect((plg_x+2*plg_s), plg_y, plg_s, plg_s, 3, 3, {fill: 'red', opacity: 0.5, id:"cdp3"});

		// draw water levels
		// tank max level in pixels
		tmlp = 135;		// deprecated?

		// tank 3 pecific settings
		<?php echo 't3top = '.$svg_t3top.';'.PHP_EOL; ?>
		<?php echo 't3btm = '.$svg_t3btm.';'.PHP_EOL; ?>
		<?php echo 't3wx = '.$svg_t3wx.';'.PHP_EOL; ?>
		<?php echo 't3wy = '.$svg_t3wy.';'.PHP_EOL; ?>
		ctl = 135;
		svg.polygon([
				[t3wx,t3wy],
				[t3wx,(t3wy-ctl)],
				[(t3wx+30),(t3wy-ctl-50)],
				[(t3wx+120),(t3wy-ctl-50)],
				[(t3wx+123),(t3wy-50+5)],
				[(t3wx+95),t3wy]], 
			{fill: 'blue', strokeWidth: 0, opacity: 0.5,  id:"tank3water"});


		// tank 4 specific settings
		<?php echo 't4top = '.$svg_t4top.';'.PHP_EOL; ?>
		<?php echo 't4btm = '.$svg_t4btm.';'.PHP_EOL; ?>
		<?php echo 't4wx = '.$svg_t4wx.';'.PHP_EOL; ?>
		<?php echo 't4wy = '.$svg_t4wy.';'.PHP_EOL; ?>
		ctl = 50;
		svg.polygon([
				[t4wx,t4wy],
				[t4wx,(t4wy-ctl)],
				[(t4wx+30),(t4wy-ctl-50)],
				[(t4wx+120),(t4wy-ctl-50)],
				[(t4wx+123),(t4wy-50+5)],
				[(t4wx+93),t4wy]], 
			{fill: 'blue', strokeWidth: 0, opacity: 0.5, id:"tank4water"}); 



 //   		var g = svg.group({stroke: 'black', strokeWidth: 2});

		// draw cadi time 
		svg.text(200, 25, 'Cadi time',{fill: 'red', strokeWidth: 0, id:'cadi_time2'});
		svg.text(150, 75, 'Temp',{fill: 'green', strokeWidth: 0, id:'cadi_temp'});
		svg.text(150, 95, 'rH',{fill: 'blue', strokeWidth: 0, id:'cadi_rh'});

		// draw tank levels in text
		svg.text(730, 135, '2Top',{fill: 'white', strokeWidth: 1, stroke: "black", id:'t3l_txt'});
		svg.text(590, 335, '2Top',{fill: 'white', strokeWidth: 1, stroke: "black", id:'t4l_txt'});

		// display pressure
		svg.text(450, 405, 'Pressure@7.2',{fill: 'red', strokeWidth: 0, id:'psi_val'});

		// display CDD status text
		svg.text(200, 45, 'CDD status',{fill: 'red', strokeWidth: 0, id:'cdds'});

		// display auto_flags
		svg.text(350, 45, 'auto_flags:',{fill: 'red', strokeWidth: 0, id:'auto_flags'});

	}

	function redraw_svg_layer(){

		$.post('cm/cadi_bt_processor.php', {action: 'get_status_csv'}, function(data){
		//	alert(data);
			var statusArray = data.split(',');
			var date = new Date(statusArray[0]*1000);
			$('#cadi_time2').html(date);
			var temp = parseFloat(statusArray[1]).toFixed(1);
			var rh = parseFloat(statusArray[2]).toFixed(1);
			$('#cadi_temp').html('&nbsp;T: '+temp+'C');
			$('#cadi_rh').html('rH: '+rh+'%');

			// tank 3 water level redraw
			
		<?php echo 't3top = '.$svg_t3top.';'.PHP_EOL; ?>
		<?php echo 't3btm = '.$svg_t3btm.';'.PHP_EOL; ?>
		<?php echo 't3wx = '.$svg_t3wx.';'.PHP_EOL; ?>
		<?php echo 't3wy = '.$svg_t3wy.';'.PHP_EOL; ?>

			ctl = Math.floor((120*(t3btm-statusArray[8]+t3top))/(t3btm-t3top));
			$('#tank3water').attr('points', t3wx+','+t3wy+' '+t3wx+','+(t3wy-ctl)+' '+(t3wx+30)+','+(t3wy-ctl-50)+' '+(t3wx+120)+','+(t3wy-ctl-50)+' '+(t3wx+123)+','+(t3wy-50+5)+' '+(t3wx+98)+','+t3wy);

			// tank 4 water lvl redraw
		<?php echo 't4top = '.$svg_t4top.';'.PHP_EOL; ?>
		<?php echo 't4btm = '.$svg_t4btm.';'.PHP_EOL; ?>
		<?php echo 't4wx = '.$svg_t4wx.';'.PHP_EOL; ?>
		<?php echo 't4wy = '.$svg_t4wy.';'.PHP_EOL; ?>
			var ctl = Math.floor((120*(t4btm-statusArray[9]+t4top))/(t4btm-t4top));
			//ctl=120;
			//alert(ctl);
			$('#tank4water').attr('points', t4wx+','+t4wy+' '+t4wx+','+(t4wy-ctl)+' '+(t4wx+30)+','+(t4wy-ctl-50)+' '+(t4wx+120)+','+(t4wy-ctl-50)+' '+(t4wx+123)+','+(t4wy-50+5)+' '+(t4wx+93)+','+t4wy);

			// draw labels for tanks, displaying current level
			$('#t3l_txt').html('2Top: '+statusArray[8]+'cm');
			$('#t4l_txt').html('2Top: '+statusArray[9]+'cm');
			$('#psi_val').html('Pressure@7.2 = '+statusArray[14]+'bar');

			// get valve state circles' colors
			var valves = statusArray[5];
			for (var i=0;i<4;i++) {
				if (valves.charAt(3-i)=="1") {
					$('#cv'+i).attr('fill', 'green');
				}
				else {
					$('#cv'+i).attr('fill', 'red');
				}
			}

			// set colors for plugs (Loads) state rectangles
			for (var i=0;i<4;i++) {
				if (statusArray[6].charAt(3-i)=="1") {
					$('#cp'+i).attr('fill', 'green');
				}
				else {
					$('#cp'+i).attr('fill', 'red');
				}
			}

			// dosing pumps status squares colors
			for (var i=1;i<4;i++) {
				if (statusArray[16].charAt(3-i)=="1") {
					$('#cdp'+i).attr('fill', 'green');
				}
				else {
					$('#cdp'+i).attr('fill', 'red');
				}
			}

			if (statusArray[15]==51){	
				$('#cdds').html('CDD Enabled');
				$('#cdds').attr('fill', 'green');
			}
			else {
				$('#cdds').html('CDD Disabled');
				$('#cdds').attr('fill', 'red');
			}
			var af_bin = statusArray[17].toString(2);
			$('#auto_flags').html(statusArray[17]+' ('+af_bin+')');

			var vs_flag = $('#vs_flag').is(':checked');
			if (vs_flag==1) {
				d = new Date();
			//	alert('vs checked');
				$("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
			}

		});
	}

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
//			setTimeout(function(){btd_stream_status(1)}, 5000);
		//	btd_stream_status(1);
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
		btd_stream_status(0);	// Stop pinging Cadi before disconnect
//		setTimeout(function(){}, 100);
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
				var vs_flag = $('#vs_flag').is(':checked');
				if (vs_flag==1) {
					alert('vs checked');
					$("#cadi_img").attr("src", "img/curimage.jpeg?"+d.getTime());
				}
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
//			var interval = setInterval(function(){get_status_block()},1000);	// enables drawing SVG with PHP
			var delay=$("#status_stream_delay").val();
			var interval = setInterval(function(){redraw_svg_layer()},delay);	// enables SVG draw with JS
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

function redraw_update_log(){
		$.post('cm/cadi_bt_processor.php', {action: 'redraw_update_log'}, function(data){
			$('cadi_update_log').html(data);
		});  
}

function cadiweb_update(){
		btd_stream_status(0);
		$.post('cm/cadi_bt_processor.php', {action: 'cadiweb_update'}, function(data){});
		alert("Do not use Cadiweb control panel until server finishes software upgrade with reboot!");
		var log_upd_int_id = setInterval(function(){redraw_update_log},1000);	// enables SVG draw with JS
		$('#log_interval_id').val(log_upd_int_id);
}

function stop_log_stream(){
		var  interval = $('#log_interval_id').val();
		clearInterval(interval);
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
<input type="checkbox" id="vs_flag" /> also Video
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
				<div style="float:left;">
					<div style="float:left;"><img src="img/cadi_watering_hptl(high_pressure_two_lines).jpg" /></div>
					<div id="svg_container" style="display:block; min-width:868px; min-height:415px; float:left; position: absolute; border:1px solid red;">

					</div>
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

<br> 
	<button onClick="cadi_bt_scan();" style="display:inline; border: 1px solid red;">Scan</button>
	<select id="bind_mac" name="bind_mac">
	<option>Scan to get the list</option>
	</select>
	<button onClick=bt_connect() style="display:inline; border: 1px solid red;">Connect</button>
	RFCOMM NUMBER:
	<input type="text" style="width: 40px;" value="0" id="rfcomm_nr" /><br>
	Binded RFCOMM list (<button onclick="cadi_list_rfcomms();" style="display:inline; border: 1px solid red;">refresh</button>):
	<div id="binded_rfcomms">
		
	</div>
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

<br>

	=================================================	
	<br>
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
====================================
<br>
<button onClick="cadiweb_update()">Cadiweb software update</button><br>
<input type="hidden" id="log_interval_id" value="0" />
<div>
	<textarea size="10" id="cadi_update_log"></textarea>
</div>
<button onClick=stop_log_stream();>Stop log stream</button>
====================================
<br>



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
Status stream delay<input type="text" id="status_stream_delay" value="1000"/>
<br>


<input type="text" value="0" id="video_stream" title="this value is N in '/dev/videoN'" /><button onClick="change_video()">Change video</button>
</div>
</div>
</body>
</html>
