cadi-gcl
========
Meet these two:
Cadi GCL project is an open-source plant growth controller.
Grolly - is complete plant watering and feeding solution.

Grolly system is just based on Cadi hardware

Goal: to have an irrigation system that mixes up plant feeding solutions and provides it to the root system of the plants automatically. Has communication capabilities and two main working modes - automatic and manual. Has interface for remote advanced tuning of device's behavior.


FOLDER STRUCTURE
 
android:
    - Cadi Bluetooth Monitor for Android
    
cadiweb:
	contains Cadiweb server software for Cadi Remote Control.

hardware:
    - Eagle schematics (принципиальные схемы)
    - Topor PCB layouts (дизайны плат)
    - ulp file to convert Eagle schematics to Topor (скрипт конвертиования из Eagle в Topor)
    - schematics image files (растровые изображения дизайнов плат и схем)
    - STM32 bin flashing manual (как шить STM32 контроллеры bin файлами)
    
history:
    - old versions
    
manuals:
    - Cadi manuals
    - Cadi firmware code descriptions and architecture
    
raspberry:
    - Raspberry Pi software
    
stm32f100:
    - firmwares for STM32VL-Discovery boards
    
stm32f407:
    - firmwares for STM32F4-Discovery boards
    
win32:
    - Windows Qt Bluetooth data receiver software

check http://plantalog.livejournal.com for more info


Special thanks to creators of the tools and materials I used:
Schematics made with Eagle (http://www.cadsoftusa.com/)
PCB layouts in Topor (http://eda.eremex.ru/products/topor/)
STM32 firmwares are written using CooCox IDE (coocox.org)
Putty terminal (http://www.chiark.greenend.org.uk/~sgtatham/putty/)
STM32 ANs and PMs, STM.com ST-Link utility and libraries (st.com)

