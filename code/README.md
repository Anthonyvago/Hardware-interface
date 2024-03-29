# Hardware-interface
Dit is een uitwerking van de Hardware-interface opdracht van WoR World.

Deze uitwerking van de hardware-interface opdracht is getest in WSL (Windows Subsystem for Linux) met Ubuntu 20.04.2 als distributie.

## 1. WSL Ubuntu 20.04.2

Hoe WSL Ubuntu 20.04.2 installeert op een Windows 10/11 machine, is te volgen via de volgende [link](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overview).

Zodra WSL met Ubuntu 20.04.2 is opgezet ben je nog niet klaar. 

### 1.1 Installatie ROS2 Foxy en Boost
Je moet namelijk ROS2 en Boost geïnstalleerd hebben in je linux omgeving.<br>Ik heb deze geïnstalleerd met behulp van de volgende handleidingen: [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) en [Boost](https://stackoverflow.com/questions/12578499/how-to-install-boost-on-ubuntu).

Nu zou je denken dat je er bent. Neehoor, want WSL heeft nog geen directe toegang tot de USB apparaten van je Windows 10/11 machine. 

### 1.2 USB apparaten attachen in WSL

Hoewel het op dit moment onmogelijk is om WSL direct toegang te geven tot álle USB apparaten, kunnen we wel per USB-apparaat aangeven dat deze 'geattached' moet worden in WSL. Dit doen we met een programma genaamd "[USBIPD](https://github.com/dorssel/usbipd-win/releases)". Met behulp van de [volgende handleiding](https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/) kun je deze installeren.

> Note: Wanneer je een USB-apparaat gaat attachen, zorg er dan voor dat er minimaal één windows van WSL open staat.

Eenmaal geïnstalleerd, moet je een Opdrachtprompt openen met administratorrechten.
Voer het volgende commando uit in de opdrachtprompt om alle USB-apparaten te zien die geattached kunnen worden aan WSL:
> usbipd wsl list

Zodra je het BusID hebt gevonden van het USB-apparaat die je wil attachen, voer je het volgende commando uit:
> usbipd wsl attach --busid \<busid>

Als je je geluid aan hebt staan, hoor je ook dat er een USB-verbinding is gecreëerd (typisch Windows USB geluid).

Gefeliciteerd! Nu kun je in WSL aan de slag met je USB-apparaat.

<b>Optioneel:</b>
Om het USB-apparaat te detachen, gebruik je het volgende commando in de <b>Opdrachtprompt in Windows met administratorrechten</b>:
> usbipd wsl detach --busid \<busid>

## 2. Compileren

Zodra alle benodigdheden zijn geïnstalleerd, kun je dit hardware-interface compileren op de volgende wijze:

Stap 1: Zorg ervoor dat je in WSL je bevindt in de root folder (waar deze README ook zich in bevindt).
Stap 2: Voer de onderstaande commando's uit:
> source ~/opt/ros/iron/setup.bash
> colcon build
> . install/setup.bash

## 3. Uitvoeren

Wanneer de package is gecompileerd, kun je hem uitvoeren met het volgende commando:
> ros2 run hardware_interface Main_RobotArmDriver

en:
> ros2 run hardware_interface Demo

## 4. Troubleshooting
In de code is hard-coded het pad aangegeven naar het USB apparaat (de robotarm).<br>
Het kan voorkomen dat dit een verkeerd pad is. Wanneer dit een verkeerd pad is, zou het programma onmiddelijk crashen bij het opstarten en zou de output iets betreffen met "open()".
In dit geval moet je het pad naar het USB apparaat (de robotarm) weten en deze aangeven in "Robot_arm_HLD/RobotArmDriver.cpp" op regel 22 als argument bij de variabele "servoDriver_".

Het pad naar het USB apparaat controleer je met het volgende commando:
> ls /dev | grep tty

