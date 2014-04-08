<?php
session_start();
// cs - for Cadi Settings
$_SESSION['cs'] = array();

$_SESSION['cs']['timers'][0]['description'] = 'default timer description';
$_SESSION['cs']['timers'][0]['on'] = 0;
$_SESSION['cs']['timers'][0]['off'] = 0;
$_SESSION['cs']['timers'][0]['enabled'] = 0;
$_SESSION['cs']['plugs']['0']['description'] = 'default plug description';
$_SESSION['cs']['plugs']['0']['program_link'] = '0';	// timer, ctimer... id

$_SESSION['cs']['valves']['0']['description'] = 'default plug description';
$_SESSION['cs']['valves']['0']['program_link'] = '0';	// timer, ctimer... id

?>
