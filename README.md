<details>
<summary>Installation der benötigten Software/WSL2 Setup (Click here to collapse/unfold)</summary>

Installation Docker auf Windows mit WSL- Kästchen angetickt. VcXsrv https://sourceforge.net/projects/vcxsrv/ (Downloadlink!) vorher installieren. Empfehelung am Rande: Windows Terminal im Windows Store runterladen. Cooles Tool.


**Umstieg auf WSL2**
**Wichtig**: Docker Desktop aktiviert schon WSL2 bei der Installation. Heißt, die folgenden Schritte nur machen, wenn Docker Desktop nicht installiert wurde. Ansonsten bei Schritt 4 weitermachen!

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

**ACHTUNG: Es wird empfohlen jede Art von Docker zu deinstallieren, bevor man eine weitere Version installiert! Das bearbeitete Docker Image kann mit Docker save als .tar Datei gespeichert werden, falls man Docker Desktop auf Windows wieder verwenden sollte. Wird auch während der Installation auf der Linux Distro empfohlen!**

11. Powershell öffnen/Ubuntu starten mit "ubuntu" --> docker pull umutuzun/rosfinal2:rosfinal2 eingeben -->   Pullt das Image aus dockerhub. Natürlich kann man auch das Dockerfile im Git benutzen und das ganze selber bauen, aber das hier ist deutlich einfacher und schneller, mit folgendem Command starten: docker run -it --name eigenen-namen umutuzun/rosfinal2:rosfinal2 bash
	(eigenen-namen kann man ersetzen wie man will. Das wird der Containername sein.)

12. Mit dem Command davor ist man direkt schon im Container. Falls man den Container verlässt (mit "exit" z.B), kann man mit "docker start eigenen-namen" den Container erneut starten. Nach dem Starten muss er auch ausgeführt werden, damit man wieder im Container ist. Dies geschieht mit folgendem Command: docker exec -it eigenen-namen bash

13. Das wars dann. Wenn das ganze Setup fertig ist, kann man mit den Schritten davor weitermachen! (XLaunch für GUIs ist immer noch nötig, heißt man müsste da weitermachen)




**ROS und MoveIt! Installation**
	
	1) In Powershell: roscore	-->		ROS testen, ob alles funktioniert (Direkt drunter sollte sowas ähnliches im Terminal erscheinen). Mit Strg+C 		kann man roscore dann wieder schließen
	
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

	

	1) Nochmal zur Erinnerung: docker exec -it eigenen-namen bash--> zum Starten des erstellten ROS containers 
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

	7) ein neues Terminal/Powershell öffnen, da roscore separat laufen muss und da wieder den Docker Container starten und hier z.B. rviz eingeben oder rqt, um eine GUI zu starten
</details>



**Starten und Bedienung der Anwendung**

1. Sichergehen, dass der Workspace gesourct ist

	`source ~/ws_moveit/devel/setup.bash`

2. roscore starten, falls es noch in keinem Terminal läuft

	`roscore`

3. Launchfile ausführen

	`rqt`

4. rqt-Plugin einfügen unter Plugins -> User-Plugins -> Path Planning Plugin

5. rviz-Robotersimulation einfügen unter Plugins -> Visualization -> RViz. Es sollte sich jetzt eine leere Szene öffnen

6. Im Tab Algorithms unseres Plugin, sollte nun ein Planer gewählt werden, der den Roboter im Hintergrund lädt. OMPL ist eine Reihe von verschiedenen Planern, weshalb es möglich ist in der Drop- Downliste daneben einen anderen Planer von OMPL zu benutzen. CHOMP kann mit dem OMPL als Preprocessor gestartet werden. STOMP mit CHOMP als Postprocessor.

<details> <summary>Auswahl des Planers (click here)</summary>
![Semantic description of image](/rqt_mypkg/resource/algorithmstab.png)
</details>

7. Roboter zur Szene hinzufügen durch "Add" -> MotionPlanning im RViz Plugin.
   Unter "MotionPlanning" -> Context -> Planning Library , kann der aktuell genutzte Planer
   eingesehen werden.

8. Der Roboter sollte jetzt in der Planungsszene erscheinen.

9. Unter Displays -> "Add" -> rviz -> "MarkerArray" hinzufügen. DIE SCHRITTE 9 und 10 KÖNNEN MIT DER CONFIG DATEI ÜBERSPRUNGEN WERDEN: Die config Datei übernimmt, wenn der Roboter erstmal geladen wurde, indem man einen Planer wählt, diese Schritte. Die config- Datei findet man im rqt_mypkg Ordner unter dem Ordner "config"

10. Unter Displays Global Options -> Fixed Frame -> Wert auf "link_base" setzen, sonst werden die Marker nicht angezeigt.

<details> <summary>MotionPlanning, MarkerArray über Add hinzufügen und Fixed Frame ändern (click here)</summary>
![Semantic description of image](/rqt_mypkg/resource/add.png)
</details>

11. Eine vordefinierte Szene kann dann über den Tab Planning Scene geladen werden. Alle angezeigten Szenen sind im rqt_pkg Ordner unter Szenen. Der Refresh- Button aktualisiert die vorhandenen Szenen bzw. fügt neue hinzu, falls der Ordner aktualisiert werden sollte.

			Im Motion Planing Tab ist es möglich eine eigene Planungsszene zu erstellen. Dafür muss man auf den Reiter "Scene Objects."
			Unten im Tab kann man unter "Add/Remove scene object(s)" ein ausgewähltes Objekt über das Plus- Zeichen in die Szene setzen
			und exportieren/importieren. Falls man ein Objekt bewegen oder anderweites ändern möchte, muss man wieder in den Reite
			"Scene Objects" und das Objekt unter "Current Scene Objects" auswählen. In der Planungsszene sollte das Objekt jetzt
			verschiebbar/drehbar sein.

<details> <summary>Scene Objects Tab (click here)</summary>
![Semantic description of image](/rqt_mypkg/resource/sceneobject.png)
</details>


12. Unter dem Reiter Start-/Goalpoint kann man einen Startpunkt festlegen, zu dem der Roboter erstmal hinfährt. Mit Goalpoint setzt man das Ziel des Roboters. Als nächstes muss auf Apply gelickt werden, um die Punkte zu laden.

13.  Jetzt kann man schon mit Plan Path den Pfad planen/berechnen. 

14. Der Roboterablauf: Planung von aktueller Position auf Startposition. Falls die Berechnung eines Pfades möglich ist, wird der Planer direkt ausgeführt.

    Fehlermeldungen werden im Terminal ausgegeben, falls z.B. kein möglicher Pfad berechnet 
    werden kann.
    Es folgt die Planung vom Start- zum Zielpunkt und im Anschluss wird der Pfad in kleinen Schritten ausgeführt. Sobald das Ziel 			erreicht ist, wird der Pfad
    als MarkerArray (Punkt-Folge) dargestellt.


15. Unter dem Reiter Statistics werden von jedem berechnten Pfad einige Informationen zum Vergleichen der unterschiedlichen Pfadplanner angezeigt.

<details> <summary>Reiter Statistics (click here) </summary>
![Semantic description of image](/rqt_mypkg/resource/statistics.png)
</details>

16. In der Reihe Display Path kann man einen Häkchen setzen, um den jeweiligen Pfad ein- bzw. auszublenden. Dies geschieht in Form von Markern. Dabei wird der Pfad und der Name des Planners angezeigt.

<details> <summary>Anzeige der Pfade (click here) </summary>
![Semantic description of image](/rqt_mypkg/resource/pfade.png)
</details>

17. Mit Export kann man für den gewählten Planner den Pfad in eine YAML- Datei exportieren. Dabei öffnet sich ein Explorer, in dem man ein Verzeichnis auswählt und 1) eine vorhandene Datei überschreibt 2) oder einen neuen Namen eingibt und die Datei neuerstellt.
