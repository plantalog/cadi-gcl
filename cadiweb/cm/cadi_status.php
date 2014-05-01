<?php

/*
- plain text status
- svg image layer

*/

include_once('cm/cadi_response_parser.php');
include_once('cadi_response_parser.php');
if (isset($_SESSION['view_id'])){
	$view_id = $_SESSION['view_id'];
}
else {
	$view_id = 0;
}

switch ($view_id) {
	case 1:
		echo $_SESSION['cadi_status']['plain_status'];
	break;
	
	case 2:
		echo $_SESSION['cadi_status']['svg'];
	break;

	case 0:
		echo $_SESSION['cadi_status']['plain_status'];
	break;

	}

?>
