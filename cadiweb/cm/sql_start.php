<?php

$con = mysql_connect('localhost', 'cadiweb', 'bewidac');
if (!$con)
  {
  die('Could not connect: ' . mysql_error());
  }

mysql_select_db("cadi_db", $con);
mysql_query("set names utf8",$con);

?>
