***Installation der benötigten Software***

1. Installation Docker auf Windows mit WSL- Kästchen angetickt

2. ROS Installation

	1) Powershell öffnen --> docker pull osrf/ros:noetic-desktop-full   -->   Pullt das Image aus docker
	
	2) docker run --name eigenen-namen -it osrf/ros:noetic-desktop-full bash   -->	Führt das Image aus und startet einen ROS Container
	
	3) echo source "/opt/ros/noetic/setup.bash" >> ~/.bashrc   -->   Als nächstes sourcen ROS immer wenn wir einen ROS container öffnen
	
	4) source ~/.bashrc 	-->	für unser jetziges Terminal/Powershell sourcen
	
	5) roscore	-->		ROS testen (Direkt drunter sollte sowas im Terminal erscheinen)
	
> ... logging to /root/.ros/log/58f66bc2-d4a4-11e9-be85-02420aff0002/roslaunch-59fe088dbe6a-325.log
> Checking log directory for disk usage. This may take awhile.
> Press Ctrl-C to interrupt
> Done checking log file disk usage. Usage is <1GB.
>  
> started roslaunch server http://59fe088dbe6a:34269/ros_comm version 1.12.14
>  
>  
> SUMMARY
> ========
>  
> PARAMETERS
>  * /rosdistro: noetic
>  * /rosversion: xxxx
>  
> NODES
>  
> auto-starting new master
> process[master]: started with pid [335]
> ROS_MASTER_URI=http://59fe088dbe6a:11311/
>  
> setting /run_id to 58f66bc2-d4a4-11e9-be85-02420aff0002
> process[rosout-1]: started with pid [348]
> started core service [/rosout]

	

	6) docker exec -it eigenen-namen bash	--> zum Starten des erstellten ROS containers kann man diese Zeile (mit dem gewählten Namen, 	    das wäre bei "eigenen-namen") in die Powershell schreiben.



**JETZT ZUM AUSFÜHREN VON GUI WIE RQT UND RVIZ, DIE OHNE DEM FOLGENDEN SETUP NICHT FUNKTIONIEREN!**



3. VcXsrv schonmal installieren.

	1) Bis zu den "Extra settings" Weiter anklicken und bei den "Extra settings" den Haken von "Native opengl" entfernen und bei 		 	    "Disable access control" einfügen

	2) Konfiguration speichern bei Bedarf und fertigstellen

	3) Neue Powershell öffnen und unseren ROS container mit docker exec -it eigenen-namen bash starten
	
	4) echo 'export DISPLAY={die eigene IP adresse ohne klammern}:0.0' >> ~/.bashrc  **Achtung: Falls man eine IP Adresse hat, die sich 	   von Zeit zu Zeit ändert (anderer Ort, Uni Wlan etc etc.), dann muss dieser Schritt wiederholt werden!!**
	   > (Die IP Adresse findet man unter Einstellunge --> Netzwerk und Internet --> Eigenschaften (steht im ersten Reiter Status!) --> 		 unter IPv4-Adresse)

	5) source ~/.bashrc  -->    das veränderte .bashrc file sourcen

	6) roscore eingeben und ROS starten.

	7) ein neues Terminal/Powershell öffnen und da z.B. rviz eingeben oder rqt oder rqt_console...


**Installation des rqt-Plugins**
	1. neuen Workspace-Ordner kreieren z.B. 
		`$ mkdir ~/ws_rqt_plugin`
	2. In dem neuen Ordner einen Ordner "src" erstellen
		`$ cd ws_rqt_plugin`
		`$ mkdir src`
	3. Im Workspace-Folder einen Catkin-Workspace kreieren
		`$ cd ..`
		`$ catkin_make`
	4. Das Package mit dem Plugin in den src-Ordner clonen
		`$ cd src`
		`$ git clone https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2020-21/team6`
	5. Den Workspace sourcen
		`$source ws_rqt_plugin/devel/setup.bash
	6. roscore und rqt starten in seperaten Terminals starten um das Plugin zu registrieren
		`$ roscore`
		`$ rqt --force-discover`
	7. Das Plugin erscheint unter dem Plugin-Tab in rqt
