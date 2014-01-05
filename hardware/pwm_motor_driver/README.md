Cadi PWM MOSFET valve motor driver
=============================

Одиночный драйвер для шаровых клапанов с моторами на 3 вольта и обратной связью

IRL640 можно заменить чем-то более простым, дешевым и доступным. L серия хорошо подходит для низковольтных применений благодаря низкому напряжению управления (gate threshold voltage).

В качестве накачивающего транзистора можно взять любой аналогичный, типа 2N2222


Single driver for spherical valves with 3 volt motors and feedback

IRL640 could be replaced with something less powerful and more cheaper.

Higher voltages motors are possible to use with this driver also.

2N3904 transistor used in schematics, but also possible to use any other general purpose transistor with similar characteristics. Like 2N2222.


Files:
	- Eagle schematics: single_pwm_mosfet_driver_with_npn_booster_3v.sch
	- Specctra: single_pwm_mosfet_driver_with_npn_booster_3v.dsn\
	- Topor: single_pwm_mosfet_driver_with_npn_booster_3v_v2.fsx
	- PCB PDF: single_pwm_mosfet_driver_with_npn_booster_topor_v2.pdf
	- PCB component layout: single_pwm_mosfet_driver_with_npn_booster_3v_v2_component_view.pdf
	


Special thanks to DIHLT for this article
 http://easyelectronics.ru/upravlenie-moshhnoj-nagruzkoj-postoyannogo-toka-chast-3.html
Отдельное спасибо DIHALTу за его статью про управление нагрузками
