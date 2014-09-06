<script>
$(function() {
	$( ".btn_cadi_timer_enabled" ).button();
	$( "#cadi_watering_accordion" ).accordion();
});

function submitWateringData(){
	$("#your_datepicker").datetimepicker("getDate").getTime() / 1000
}

function get_settings(){	// read 32 bit STM32 flash value
	var addr = $('#settings_block_id').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:58, addr:addr}, function(data){
		//	alert('sent settings request');
	});
}

function get_settings_block(){	// read 32 bit STM32 flash value
	var addr = $('#settings_block_id').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:59, addr:addr}, function(data){
		//	alert('sent settings request');
	});
}

// 16 bit
function save_settings(){
	var addr = $('#settings_block_id').val();
	var value = $('#settings_value').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:15, addr:addr, value:value}, function(data){
			alert('settings saved');
	});
}

function save_settings_block(){
	var addr = $('#settings_block_id').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:16, addr:addr}, function(data){
			alert('sent block of settings to Cadi');
	});
}

function apply_settings(){
	var addr = $('#settings_block_id').val();
	var value = $('#settings_value').val();
	$.post('cm/cadi_bt_processor.php', {action: 'tx', cmd:19}, function(data){});
}



</script>
<div id="cadi_watering_accordion">

<h3>Программа полива 1</h3>
<div>
	<table>
	<tr>
		<td title="в сантиметрах, уровня воды в баке">Количество воды, см</td>
		<td><input type="text" value="" /></td>
	</tr>
	<tr>
		<td>Удобрение 1</td>
		<td><input type="text" value="" />мл</td>
	</tr>
	<tr>
		<td>Удобрение 2</td>
		<td><input type="text" value="" />мл</td>
	</tr>
	<tr>
		<td>Удобрение 3</td>
		<td><input type="text" value="" />мл</td>
	</tr>
	<tr>
		<td>Интервал поливов</td>
		<td></td>
	</tr>
	<tr>
		<td>Начало</td>
		<td><input type="text" id="" name="" value="" /></td>
	</tr>
	<tr>
		<td>Конец</td>
		<td><input type="text" id="" name="" value="" /></td>
	</tr>
	<tr>
		<td>Линии полива</td>
		<td>1 / 2 / 3</td>
	</tr>
	<tr>
		<td>Включено?</td>
		<td>Да/Нет</td>
	</tr>
</table>

<b>Settings read</b>
Block ID<input type="text" id="settings_block_id" />
<button onClick="get_settings()">Get settings</button>
<br>
<button onClick="get_settings_block()">Get 32byte BLOCK</button>
<br>
<b>Settings write</b>
Value<input type="text" id="settings_value" />
<button onClick="save_settings()">Save settings</button>
<br>
<button onClick="save_settings_block()" title="from dump file, starting at address set above in 'BlockId'">Send block</button>
<br>
<button onClick="apply_settings()">Apply settings</button>
<br>
<br>
<b title="put start address in the field above">Daemon controlled settings dump file write to EEPROM</b><br>
Number of 16 bit vals to transfer<input type="text" id="settings_number" />
<button onClick="rx_ee()">BTD rx_ee</button>


</div>

<h3>NFT</h3>
<div style="background: url('../img/nft_system_feeding.png') no-repeat right; min-height:450px;">

		<input class="btn_cadi_timer_enabled" id="btn_cadi_timer_n" type="checkbox"><label for="btn_cadi_timer_n">Enabled</label>

		<table>
		<tr>
			<td>1. Watering pump plug</td>
			<td>
				<select>
					<option>Исполнительное устройство 1</option>
					<option>Исполнительное устройство 2</option>
					<option>Исполнительное устройство 3</option>
					<option>Исполнительное устройство 4</option>
				</select>
			</td>
		</tr>
		<tr>
			<td>2. Solution supply pump</td>
			<td>
				<select>
					<option>Исполнительное устройство 1</option>
					<option>Исполнительное устройство 2</option>
					<option>Исполнительное устройство 3</option>
					<option>Исполнительное устройство 4</option>
				</select>
			</td>
		</tr>
		<tr>
			<td>3. Solution mixing pump</td>
			<td>
				<select>
					<option>Исполнительное устройство 1</option>
					<option>Исполнительное устройство 2</option>
					<option>Исполнительное устройство 3</option>
					<option>Исполнительное устройство 4</option>
				</select>
			</td>
		</tr>
		<tr>
			<td>4. Fresh water pump</td>
			<td>
				<select>
					<option>Исполнительное устройство 1</option>
					<option>Исполнительное устройство 2</option>
					<option>Исполнительное устройство 3</option>
					<option>Исполнительное устройство 4</option>
				</select>
			</td>
		</tr>
		<tr>
			<td>5. NFT drain valve</td>
			<td>
				<select>
					<option>Исполнительное устройство 1</option>
					<option>Исполнительное устройство 2</option>
					<option>Исполнительное устройство 3</option>
					<option>Исполнительное устройство 4</option>
				</select>
			</td>
		</tr>
		<tr>
			<td>6. Fresh water supply valve</td>
			<td>
				<select>
					<option>Исполнительное устройство 1</option>
					<option>Исполнительное устройство 2</option>
					<option>Исполнительное устройство 3</option>
					<option>Исполнительное устройство 4</option>
				</select>
			</td>
		</tr>
		<tr>
			<td>7. Water intake valve</td>
			<td>
				<select>
					<option>Исполнительное устройство 1</option>
					<option>Исполнительное устройство 2</option>
					<option>Исполнительное устройство 3</option>
					<option>Исполнительное устройство 4</option>
				</select>
			</td>
		</tr>
		</table>



</div>
<h3>DWC</h3>
<div>

</div>

<h3>Rockwool</h3>
<div>

</div>

<h3>Custom 1</h3>
<div>

</div>

<h3>Custom 2</h3>
<div>

</div>

<h3>Custom 3</h3>
<div>

</div>

</div>

