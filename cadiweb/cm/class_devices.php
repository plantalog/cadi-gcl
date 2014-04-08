<?php

include('defines.php');

class Cadi_device{
	private	$device['id']=0;
	private	$device]'name']='default name';
	private	$device['type'] = 0;
	private	$device['mac'] = 'aa:bb:cc:dd:ee:ff';
	
	function	__construct($device_id){
		db_read($device_id);
	}

	public function get_device_data(){
		return $this->device;
	}

	public function set_id($id){
		$this->device['id'] = $id;
	}

	public function set_type($type){
		$this->device['type'] = $type;
	}

	public function set_name($name){
		$this->device['name'] = $name;
	}

	public function set_mac($mac){
		$this->device['mac'] = $mac;
	}
	




	// Database functions

	function save(){				// saves device object structure into database (DB)
		if (db_is_dev_exist($device_id)==true){
			db_update();
		}
		else {
			db_insert();
		}
	}

	function db_is_dev_exist($device_id){		// chechs if this device id already exists in DB
		include('sql_start.php');
		$sql = "SELECT * FROM ".TBL_DEVICES." WHERE device_id=".$device_id;
		$result = mysql_query($sql);
		mysql_close($con);
	}

	function db_read($device_id){		// reads device structure from DB into object
		include('sql_start.php');
		$sql = "SELECT * FROM ".TBL_DEVICES." WHERE device_id=".$device_id;
		$result = mysql_query($sql);
		
		mysql_close($con);
	}

	function db_update(){			// updates device with id already existing in DB
		include('sql_start.php');
		$sql = "UPDATE ".TBL_DEVICES." SET ";
		mysql_close();
	}

	function db_insert(){
		include('sql_start.php');
		$sql = "INSERT INTO ".TBL_DEVICES." (
							currency,
							vat,
							receiver,
							stock_id,
							item_name,
							invoice_number,
							item_qty,
							item_price,
							type,
							date,
							hash,
							item_discount,
							discount_abs,
							payment_type,
							created_date,
							created_by,
							modified_by,
							modified_date
						) VALUES(
							'".$this->currency."',
							'".$this->vat_value."',
							'".$this->receiver."',
							'".$invitem['apsi_id']."',
							'".$invitem['name']."',
							'".$this->number."',
							'".$invitem['qty']."',
							'".$invitem['price']."',
							'".$this->type."',
							'".$this->date."',
							'".$this->hash."',
							'".$invitem['discount']."',
							'".$invitem['discount_abs']."',
							'".$this->payment_type."',
							'".$this->created_date."',
							'".$this->created_by."',
							'".$this->modified_by."',
							'".$this->modified_date."'
						);";
	}
}

/*
Each timer has 80bits of data (10bytes):
	- Timer type - TT[0..2]. 001 - 24H, 010 - Full Range, 011 - Cyclic
	- Timer Enabled - TE[3]. If set to 1, Cadi loads this timer for processing (depending on firmware working speed and delay changes due to changing the processing loop length, we should decide if we want the loop to be always fixed length, or we can let it be adjustable without having problems with main program flow.
Этот параметр отвечает за включение таймера в очередь обработчика состояний таймеров. То есть, размер цикла for будет разным в зависимости от того какие таймеры включены. Это может повлиять на "гладкость" выполнения программы, что необходимо проверить на практике. Если будут проблемы, то простейшее решение - зафиксировать размер цикла)
	- TimerOn/Duration - TND[16..47]. 32 bits of data for timer ON or Duration, depending on type set in TE[0..2].
	- TimerOff/Interval - TFI[48..79]. Timer OFF or Interval

*/

class Cadi_Timer{
	private	$timer['id'] = 0;
	private	$timer['on'] = 0;
	private	$timer['off'] = 0;
	private	$timer['type'] = 0;	// 0 - full range, 1 - 24H, 2 - Cyclic timer
	private	$timer['enabled'] = 0;
}

class Cadi_Timer_Chain{
	private	$chain[0]=0;
	private	$chain[1]=0;
	private	$chain[2]=0;
	private	$chain[3]=0;
	private	$chain[4]=0;
}







?>
