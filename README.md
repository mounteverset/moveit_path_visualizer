***Installation der benötigten Software/WSL2 Setup GANZ UNTEN***

1. Installation Docker auf Windows mit WSL- Kästchen angetickt. VcXsrv https://sourceforge.net/projects/vcxsrv/ (Downloadlink!) vorher installieren


2. ROS und MoveIt! Installation

	1) Powershell öffnen --> docker pull umutuzun/rosfinal2:rosfinal2 eingeben -->   Pullt das Image aus dockerhub. Natürlich kann man auch das Dockerfile 	im Git benutzen und das ganze selber bauen, aber das hier ist deutlich einfacher und schneller

	2) Im Docker Desktop das gepullte Image (unter dem Reiter Images) starten (Run) und im neuen Fenster bei Bedarf unter Optional Settings einen eigenen Container 	Namen aussuchen. Notwendig ist es nicht, da Docker sonst einen eigenen Namen aussucht, aber diesen muss man dann später verwenden!

	2.1) In Powershell: docker exec -it eigenen-namen bash -->	Führt das Image aus und startet einen ROS Container
	
	
	3) In Powershell: roscore	-->		ROS testen, ob alles funktioniert (Direkt drunter sollte sowas ähnliches im Terminal erscheinen). Mit Strg+C 		kann man roscore dann wieder schließen
	
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

	

	4) Nochmal zur Erinnerung: docker exec -it eigenen-namen bash--> zum Starten des erstellten ROS containers 
	kann man diese Zeile (mit dem gewählten Namen, das 		
	wäre bei "eigenen-namen") in die Powershell schreiben. So kann man roscore ausführen!



**JETZT ZUM AUSFÜHREN VON GUI WIE RQT UND RVIZ, DIE OHNE DEM FOLGENDEN SETUP NICHT FUNKTIONIEREN!**


3. Wenn VcXsrv installiert wurde (wenn nicht dann bitte jetzt): https://sourceforge.net/projects/vcxsrv/ (Downloadlink!)

	1) Starten und bis zu den "Extra settings" Weiter anklicken und bei den "Extra settings" den Haken von "Native opengl" entfernen und bei 		 	    "Disable access control" einfügen

	2) Konfiguration speichern bei Bedarf und fertigstellen

	3) Unseren ROS container mit docker exec -it eigenen-namen bash starten
	
	4) echo 'export DISPLAY={die eigene IP adresse ohne klammern}:0.0' >> ~/.bashrc  
	**Achtung: Falls man eine IP Adresse hat, die sich 	   von 		   Zeit zu Zeit ändert (anderer Ort, Uni Wlan etc etc.), dann muss dieser Schritt wiederholt werden!!**
	   > (Die IP Adresse findet man unter Einstellungen --> Netzwerk und Internet --> Eigenschaften (steht im ersten Reiter Status!) --> 		 unter 			IPv4-Adresse)

	5) source ~/.bashrc  -->    das veränderte .bashrc file sourcen

	6) roscore eingeben und ROS starten.

	7) ein neues Terminal/Powershell öffnen, da roscore separat laufen muss und da den Docker Container starten und hier z.B. rviz eingeben oder rqt, um eine GUI zu starten


**Installation des rqt-Plugins und des Fanuc Roboters mit MoveIt zusammen**

1. Wir müssen erstmal feststellen, ob **wir im ws_moveit/src** Verzeichnis sind 

	`cd ws_moveit/src --> :~/ws_moveit/src so sollte unser Pfad aussehen`

2. Den Ordner aus git clonen
	
	`git clone https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2020-21/team6`

3. Zurück ins ws_moveit Verzeichnis und den MoveIt- Workspace builden!

	`cd .. --> :~/ws_moveit# so sollte unser Pfad aussehen und so`
	 
	`catkin build rqt_mypkg` builden
	`catkin build fanic_m710` builden
	

4. Workspace sourcen

	`source ~/ws_moveit/devel/setup.bash`

5. Zuerst roscore, dann rqt in seperaten Terminals starten um das Plugin zu registrieren

	`roscore`
	
	`rqt --force-discover`
		
6. Das Plugin erscheint unter dem Plugin-Tab in rqt


**Starten und Bedienung der Anwendung**

1. Sichergehen, dass der Workspace gesourct ist

	`source ~/ws_moveit/devel/setup.bash`

2. roscore starten, falls es noch in keinem Terminal läuft

	`roscore`

3. Launchfile ausführen

	`roslaunch rqt_mypkg rqt_launch.launch`

4. rqt-Plugin einfügen unter Plugins -> User-Plugins -> Pfadplanung

5. rviz-Robotersimulation einfügen unter Plugins -> Visualization -> RViz. Es sollte sich jetzt eine leere Szene öffnen

6. Roboter zur Szene hinzufügen durch "Add" -> MotionPlanning im RViz Plugin

![Add-Button](<https://ibb.co/7nYVv1g>)

7. Der Roboter sollte jetzt in der Planungsszene erscheinen

8. Um den Pfad anzeigen zu lassen jetzt noch im MotionPlanning -> PlannedPath -> Häkchen bei Loop Animation & Show Trail

![Show Trail](<https://ibb.co/tpXyhSP>)

9. Eine vordefinierte Szene kann über den Tab PlanningScene geladen werden (Szene findet sicher unter /rqt_mpkg/config/demo_scene.scene)

10. Ein Start-und Endpunkt wird im Tab "Start_End_Punkte festgelegt. Dies ist verpflichtend bevor ein Pfad geplant werden kann

11. Im Tab Algorithmen ist OMPL voreingestellt. Hier gibt es im Moment noch nicht viel zu tun

12. Jetzt kann der Pfad berechnet werden. 


**Starten des Fanuc Roboters**

1. Sicher gehen, dass der Workspace gesourct ist `source ~/ws_moveit/devel/setup.bash`

2. Zum Starten des Roboters in Rviz `roslaunch fanuc_m710 demo.launch`

**Fehlerbehebung**

1. Rviz hängt und lässt sich nicht schließen: VcXsrv (XLaunch) unten im Startmenü schließen erzwingt das Schließen des Programms, da die GUIs ohne XLaunch nicht laufen.

2. Das RQT-Plugin erscheint nicht unter Plugins; Die GUI konnte nicht gestartet werden (Fehler output): Richtig sourcen (bzw. nicht vergessen zu sourcen!!!)

3. FPS- Einbrüche, schlechte Performance: Hardware-Acceleration existiert noch nicht. Da kann man leider nicht
viel machen.







**Umstieg auf WSL2**
**Wichtig**: Docker Desktop aktiviert schon WSL2 bei der Installation. Heißt, die folgenden Schritte nur machen, wenn Docker Desktop nicht installiert wurde.

1. Zuerst die WSL features in Powershell mit folgendem Command aktivieren (mit Adminrechte!): 
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart

2. Mit folgendem Command Virtual Machine Platform aktivieren (nötig, da der User wahrscheinlich keinen Microsoft insider build benutzt): 
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

3. Computer neustarten

4. WSL auf WSL2 mit folgendem Command stellen: wsl --set-default-version 2

5. Vielleicht ist ein Kernel Update nötig (sollte aber schon bei der Docker- Installation gemacht worden sein):
WSL 2 requires an update to its kernel component. For information please visit  UPDATE LINK: https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi von der Microsoft- Seite zu finden: https://docs.microsoft.com/en-us/windows/wsl/install-win10

6. Wieder diesen Command ausführen: wsl --set-default-version 2, falls die Kernel Updates nötig waren!

7. Jetzt kann man im Windows Store Ubuntu 20.04 runterladen! (Einfach Ubuntu in die Suchleiste eingeben, darauf achten welche Version. Die Ubuntu Version ohne irgendwelche Zahlen, sprich nur "Ubuntu" sollte 20.04 sein. Am besten die Beschreibung lesen!)

8. Mit folgendem Command kontrollieren, welche WSL Version die Distro benutzt: wsl -l -v

9. Dann folgendes eingeben: wsl --set-version <distribution name> 2  und als Standard setzen: wsl --set-default-version 2

10. Docker auf der Distro runterladen und die Schritte oben ausführen! (sudo apt-get update dann curl -fsSL https://get.docker.com -o get-docker.sh und zum Installieren: sudo sh get-docker.sh) 

**ACHTUNG: Es wird empfohlen jede Art von Docker zu deinstallieren, bevor man eine weitere Version installiert! Das bearbeitete Docker Image kann mit Docker save als .tar Datei gespeicher werden, falls man Docker Desktop auf Windows wieder verwenden sollte. Wird auch während der Installation auf der Linux Distro empfohlen!**

11. Das Image wie oben beschrieben pullen, mit folgendem Command starten: docker run -it --name eigenen-namen umutuzun/rosfinal2:rosfinal2 bash
	(eigenen-namen kann man ersetzen wie man will. Das wird der Containername sein.)

12. Mit dem Command davor ist man direkt schon im Container. Falls man den Container verlässt (mit "exit" z.B), kann man mit "docker start eigenen-namen" den Container erneut starten. Nach dem Starten muss er auch ausgeführt werden, damit man wieder im Container ist. Dies geschieht mit folgendem Command: docker exec -it eigenen-namen bash

13. Das wars dann. Wenn das ganze Setup fertig ist, kann man mit den Schritten davor weitermachen! (XLaunch für GUIs ist immer noch nötig, heißt man müsste da weitermachen)


