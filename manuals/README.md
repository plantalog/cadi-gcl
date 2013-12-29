Как собрать Cadi
================

Для примера соберём базовый комплект, который будет иметь 4 кнопки для управления/настройки Cadi и двухстрочный дисплей чтобы видеть чем рулить. Кнопки будут самопальные, а дисплей любой, типа HD44780 или 1602.
Собираться всё будет на базе платы STM32VL-DISCOVERY. Вот схема подключения различных устройств к плате

https://github.com/plantalog/cadi-gcl/blob/master/hardware/kai_wiring_diagram_v1.2.png


Вверху рисунка подписана версия 1.2. Она совпадает с прошивкой, которая инициализирует устройства на тех пинах, что представлены на рисунке.

Самопальные кнопки имеют два контакта для подключения
Суть работы: когда ни одна кнопка не нажата, сопротивление между выводами всего бока кнопок стремится к бесконечности. Нажатие первой кнопки даёт сопротивление N. Нажатие второй кнопки (первая уже отпущена) даст сопротивление 2N и так далее. Для примера возьмём N=2.2кОм. Тогда четвёртая нажатая кнопка блока даст сопротивление 8.8кОм.

Подключим кнопки к контроллеру через делитель напряжения вроде такого 
https://github.com/plantalog/cadi-gcl/blob/master/hardware/4_analog_buttons_wiring.png

Далее необходимо подключить дисплей. Для ориентира возьмём такую схему
https://github.com/plantalog/cadi-gcl/blob/master/hardware/kai_wiring_diagram_v0.png

По итогу должен получиться кабель для соединения дисплея с платой Discovery

Подключаем с его помощью дисплей, ориентируясь на схему 1.2 (где шина данных подключена со смещением на 1 пин (PB10-PB13) относительно вышеприведённой (PB11-PB14) схемы).

Шьём контроллер версией 1.2, даём 5 вольт дисплею, контроллеру и кнопкам. Должно работать!



st.com resources - http://plantalog.livejournal.com/3972.html