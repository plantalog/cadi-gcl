CadiProto communication test
========

This test suppose to check the data transfer consistency between Cadi and Linux machine (Raspberry Pi in this case).
Hardware needed:
- Raspberry Pi
- Cadi grow controller

During the test the binary stream between Cadi and Linux system have to include all possible byte values from 0 to 255 to check, that the test config does not escape any byte sent or received.

The test consists of two parts.
1. Test whether the bytes sent from Linux machine via serial /dev/rfcomm0 are successfully received by Cady and displayed on hd44780 LCD display when test function is running.
The test function for this test has to print on the first line of display 5 bytes such way: AAABBBCCCDDDEEE. Where AAA - first byte valiue from o to 255 and so on.
The second line displays 2 more bytes FFFGGG and the ASCII representation of these all 7 bytes.
Having 7 bytes and 256 you have to send 37 sequences to make sure all of them are transferred correctly, withoud escaping, screening or any other kind of source stream transformation.

2. This part assumes echoing the bytes sent from Linux machine to Cadi back to Linux machine. This is done to ensure, that all data transmitted there and back are consistent and not transformed any way.

For this test, please, use 1_6_3_based firmware and navigate to Temp&Humidity menu then click OK twice to see the test output on display.



Тест коммуникации Cadi

Тест проверяет целостность переданных данных с Linux устройства на Cadi и обратно.

Тест состоит из двух частей.
1. На Cadi запускается тестовая функция, которая отображает на дисплее 7 последних принятых байт. Первая строка содержит 5 групп по 3 символа, вида AAABBBCCCDDDEEE. Где AAA - значение первого байта в диапазоне от 0 до 255 и так далее. Иными словами - строчная репрезентация первых пяти байт. Следующая строка содержит ещё два байта строчной репрезентации и 7 байт в сыром виде ABCDEFG.
Тест подразумевает перебор всех значений от 0 до 255.
Отправляем строку из PHP вида:
$test_string = chr(0).chr(1).chr(2)....chr(6);

можно так:
exec("echo ".$test_string." > /dev/rfcomm0")

или
fopen();

2. Вторая часть теста подразумевает возврат принятых на Cadi данных обратно - на Linux устройство. Это называют ещё эхо.
Эхо должно содержать копию отправленого потока, без изменений.

Для теста используется прошивка на основе версии 1.6.3. После её прошивки в контроллер, тестовая функция доступна через основное меню выбором пункта Temp&Humidity. Два нажатия ОК и последние 7 байт принятых по USART отображаются на дисплее.

