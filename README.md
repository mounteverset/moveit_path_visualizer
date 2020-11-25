***Installation der benötigten Software***

1. Installation Docker auf Windows mit WSL- Kästchen angetickt. VcXsrv https://sourceforge.net/projects/vcxsrv/ (Downloadlink!) vorher installieren

1.1 In VcXsrv bis zu den "Extra settings" Weiter anklicken und bei den "Extra settings" den Haken von "Native opengl" entfernen und "Disable access control" aktivieren. **Das am Anfang machen, damit das Setup mit dem ISE Skript läuft. Falls das Setup von Hand gemacht wird, kann man die Installation von VcXsrv ab dem dritten Punkt starten!**

2. ROS Installation

	1) Powershell öffnen --> docker pull umutuzun/rosmove:latest  eingeben -->   Pullt das Image aus dockerhub. Natürlich kann man auch das Dockerfile 		  im Git benutzen das hier ist aber deutlich einfacher (Beides dasselbe)

	2) Im Docker Desktop das gepullte Image (unter dem Reiter Images) starten (Run) und bei Bedarf unter optional settings einen eigenen Container 		   Namen aussuchen. Notwendig ist es nicht, da Docker sonst einen eigenen Namen aussucht.

	2.1)In Powershell: docker exec -it eigenen-namen bash -->	Führt das Image aus und startet einen ROS Container
	
	3) In Powershell: echo source "/opt/ros/noetic/setup.bash" >> ~/.bashrc   -->   Als nächstes sourcen ROS immer wenn wir einen ROS container 		   öffnen, bei dem selben Container muss man dies nach dem ersten Setup nicht mehr tun!
	
	4) In Powershell: source ~/.bashrc 	-->	in die bash sourcen
	
	5) In Powershell: roscore	-->		ROS testen, ob alles funktioniert (Direkt drunter sollte sowas ähnliches im Terminal erscheinen). Mit Strg+C 		kann man roscore dann wieder schließen
	
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

	

	6) Nochmal: docker exec -it eigenen-namen bash	--> zum Starten des erstellten ROS containers kann man diese Zeile (mit dem gewählten Namen, das 		wäre bei "eigenen-namen") in die Powershell schreiben. Dann kann man roscore ausführen!



**JETZT ZUM AUSFÜHREN VON GUI WIE RQT UND RVIZ, DIE OHNE DEM FOLGENDEN SETUP NICHT FUNKTIONIEREN!**

2.1 Installation von rqt: Nach dem Starten des Containers sind wir erstmal im MoveIt- Verzeichnis. Wir müssen aber ins rospack- Verzeichnis, dazu geben wir in der Powershell das ein: 

	- cd /opt/ros/noetic/share

	- sudo apt-get update
					
	- sudo apt-get install ros-noetic-rqt
					
	- sudo apt-get install ros-noetic-rqt-common-plugins
					
	- sudo apt-get install ros-noetic-rqt-moveit
					
	- sudo apt-get install ros-noetic-rqt-robot-plugins

	Jetzt sollte rqt in Verbindung mit rviz ausführbar sein!

3. Wenn VcXsrv installiert wurde (oder auch nicht): https://sourceforge.net/projects/vcxsrv/ (Downloadlink!)

	1) Bis zu den "Extra settings" Weiter anklicken und bei den "Extra settings" den Haken von "Native opengl" entfernen und bei 		 	    "Disable access control" einfügen

	2) Konfiguration speichern bei Bedarf und fertigstellen

	3) Unseren ROS container mit docker exec -it eigenen-namen bash starten
	
	4) echo 'export DISPLAY={die eigene IP adresse ohne klammern}:0.0' >> ~/.bashrc  **Achtung: Falls man eine IP Adresse hat, die sich 	   von 		   Zeit zu Zeit ändert (anderer Ort, Uni Wlan etc etc.), dann muss dieser Schritt wiederholt werden!!**
	   > (Die IP Adresse findet man unter Einstellungen --> Netzwerk und Internet --> Eigenschaften (steht im ersten Reiter Status!) --> 		 unter 			IPv4-Adresse)

	5) source ~/.bashrc  -->    das veränderte .bashrc file sourcen

	6) roscore eingeben und ROS starten.

	7) ein neues Terminal/Powershell öffnen, da roscore separat laufen muss und da z.B. rviz eingeben oder rqt, um eine GUI zu starten


**Installation des rqt-Plugins**

1. neuen Workspace-Ordner kreieren z.B. 

	`mkdir ~/ws_rqt_plugin`

2. In dem neuen Ordner einen Ordner "src" erstellen
	
	`cd ws_rqt_plugin`
	`mkdir src`



3. Das Package mit dem Plugin in den src-Ordner clonen

	`cd src`
	`git clone https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2020-21/team6`

4. Im Workspace-Folder einen Catkin-Workspace kreieren

	`cd ..`
	`catkin_make`

5. Den Workspace sourcen

	`source ws_rqt_plugin/devel/setup.bash`

6. roscore und rqt starten in seperaten Terminals starten um das Plugin zu registrieren

	`roscore`
	`rqt --force-discover`
		
7. Das Plugin erscheint unter dem Plugin-Tab in rqt

