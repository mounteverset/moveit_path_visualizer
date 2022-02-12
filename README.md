# Path Visualization with ROS MoveIt and RQT

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/ZNEJtlViGfs/0.jpg)](https://www.youtube.com/watch?v=ZNEJtlViGfs)


# Installationshinweise #

### Installation unter Windows / WSL2 Setup
<details>
<summary>Aufklappen</summary>

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

### Installation unter Ubuntu 20.04

<details>
<summary>Aufklappen</summary>

1. ROS Installation folgen: http://wiki.ros.org/noetic/Installation/Ubuntu 

2. MoveIt Installation folgen: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html

3. Dieses Repo in MoveIt Source clonen

`cd ~/ws_moveit/src/`

`git clone https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2020-21/team6.git`

4. STOMP aus dem ROS Industrial Repo herunterladen

`https://github.com/ros-industrial/stomp_ros.git`

5. TRAC-IK installieren

`sudo apt-get install ros-melodic-trac-ik-kinematics-plugin`

6. Nlopt 2.6.2 herunterladen und installieren (Hinweisen auf der Webseite folgen)

https://nlopt.readthedocs.io/en/latest/

7. Den gesamten MoveIt Workspace catkin builden und neu sourcen

`cd ~/ws_moveit/`

`caktin build`

`source ~/ws_moveit/devel/setup.bash`


</details>

# Bedienungshinweise

### Anleitung

<details>
<summary>Aufklappen</summary>


1. Sichergehen, dass der **Workspace gesourct** ist. In einem neuen Terminal diesen Befehl ausführen 

	`source ~/ws_moveit/devel/setup.bash`

2. **roscore starten**, falls es noch in keinem Terminal läuft

	`roscore`

3. Launchfile ausführen

	`rqt`

4. **Plugin aus diesem Repository** einfügen unter Context-Menü -> Plugins -> User-Plugins -> Path Planning Plugin

5. **rviz-Robotersimulation einfügen** unter Plugins -> Visualization -> RViz. Es sollte sich jetzt eine leere Szene öffnen

6. Im Tab **Algorithms** unseres Plugins, muss nun ein Pfadplaner gewählt werden, der den Roboter im Hintergrund lädt. **OMPL** ist eine Bibliothek aus verschiedenen Planern, weshalb es möglich ist in der Drop-Downliste daneben einen anderen Planer von OMPL zu benutzen. **CHOMP** kann mit dem OMPL als Preprocessor gestartet werden. **STOMP** mit CHOMP als Postprocessor.
Sobald man den Planer mit Apply bestätigt öffnet sich ein neues Terminal und der Planer lädt. Sobald die **Initialisierung abgeschlossen** ist, erscheint die Meldung _"You can start planning now!"_.

<details> 
<summary>Aufklappen</summary>
![Semantic description of image](/Resources/algorithmstab.png)
</details>

_Die Schritte **7 bis 10** können durch Laden einer rviz config Datei übersprungen werden: Die config Datei übernimmt die folgenden Schritte. Sie kann in rviz durch Strg + O oder über das Context Menü vom rviz Plugin unter File -> **Open Config** geöffnet werden und ist unter ~/ws_moveit/src/team6/rqt_mypkg/config/default_rviz_config.config zu finden._

7. Roboter zur Szene hinzufügen durch "Add" -> MotionPlanning im RViz Plugin.
   Unter "MotionPlanning" -> Context -> Planning Library , kann der aktuell genutzte Planer
   eingesehen werden.

8. Der Roboter sollte jetzt in der Planungsszene erscheinen.

9. Unter Displays -> "Add" -> rviz -> **"MarkerArray"** hinzufügen. 

10. Unter Displays Global Options -> **Fixed Frame** -> Wert auf **"link_base"** setzen, sonst werden die Marker nicht angezeigt.

<details> 
<summary>Aufklappen</summary>
![Semantic description of image](/Resources/add.png)
</details>

11. Eine vordefinierte Szene kann dann über den Tab **Planning Scene** geladen werden. Alle angezeigten Szenen sind unter rqt_mypkg/scenes zu finden. Der Refresh-Button aktualisiert die vorhandenen Szenen bzw. fügt neue hinzu, falls der Ordner aktualisiert werden sollte.

12. Im rviz **Motion Planing** Plugin im Tab "Scene Objects" ist es möglich eine **eigene Planungsszene** zu erstellen. Unten im Tab kann man unter "**Add/Remove scene object(s)**" ein ausgewähltes Objekt über das Plus- Zeichen in die Szene setzen und exportieren (Um sie komfortabel über unser Plugin zu laden sind die Szenen in team6/rqt_mypkg/scenes/ zu speichern). Falls man ein **Objekt bewegen** oder anderweites ändern möchte, muss man wieder im Tab "Scene Objects" und das Objekt unter "Current Scene Objects" auswählen. In der Planungsszene sollte das Objekt jetzt verschiebbar/drehbar sein.

<details> 
<summary>Aufklappen</summary>
![Semantic description of image](/Resources/sceneobject.png)
![Gepackte Szene](/Resources/demo_scene.jpg)

</details>


13. Unter dem Tab **Start-/Goalpoint** legt man die gewünschten Start-/und Zielpunkte für die Pfadplanung fest. Sie besitzen neben den karthesischen Koordinaten auch eine Orientierungskomponente, um Drehbewegungen abzubilden. Zur **Bestätigung der Punkte** muss auf Apply geklickt werden.

14.  Jetzt kann man mit dem _"Plan Path"_  Button den Pfad planen lassen. Eine Pfadplanung ist erst möglich, wenn im Hintergrund der Pfadplaner in einem seperaten Terminal läuft und man die Start-/Zielpunkte festgelegt hat.

15. **Ablauf der Pfadplanung** in rviz: 

  
  - Der Roboter bewegt sich von seiner aktuellen Position zur Startpose.
  - Die Pfadplanung von Start bis Ziel wird berechnet
  - Der Roboter bewegt sich einmal entlang des geplanten Pfades
  - Sobald der Roboter am Ziel ist wird der zurückgelegte Weg des End Effektors mit Markern visualiert  
  - Fehlermeldungen werden im Terminal ausgegeben in dem man rqt gestartet hat, falls z.B. kein möglicher Pfad für die Punkte berechnet werden kann.
    
16. Unter dem **Tab Statistics** in unserem Plugin werden von jedem berechnten Pfad wichtige Informationen zum **Vergleichen und Beurteilen **der unterschiedlichen Pfadplanner angezeigt.

<details> <summary>Aufklappen</summary>
![Semantic description of image](/Resources/statistics.png)
</details>

17. Unter den Statistiken zu den Pfaden kann man bei **_Display Path_** ein Häkchen setzen, um den jeweiligen Pfad **ein- bzw. auszublenden**. Dies geschieht in Form von Markern. Dabei wird der **Pfad und der Name des Planners** angezeigt.

<details> <summary>Aufklappen</summary>
![Semantic description of image](/Resources/pfade.png)
</details>

18. Mit **_"Export"_** im Statistics Tab kann man für den gewählten Planner den Pfad in eine YAML-Datei exportieren. Es öffnet sich ein FileDialog, in dem man ein Verzeichnis auswählt und eine vorhandene Datei überschreibt oder einen neuen Namen eingibt und die Datei neu erstellt.

</details>

### Bekannte Bugs

<details>
<summary>Aufklappen</summary>


-  Bei der Benutzung von STOMP als Pfadplaner wird manchmal keine Lösung der inversen Kinematik für den Startpunkt gefunden. Der Roboter fährt dann von seiner aktuellen Position direkt zur Zielposition.
   - Behebung: Noch einmal mit STOMP die gleichen Punkte planen lassen.
   - Alternativ: Start- und Zielpunkt vertauschen im Plugin, da sich der Roboter aktuell am Ziel befindet.

-  Trotz Verwendung eines speziellen Solvers für IK-Probleme findet MoveIt nicht immer eine Lösung für die inverse Kinematik, egal ob für Start- und Endpunkt und bei jedem der probierten Pfadplaner. Dies kann im Terminal in dem rqt gestartet ist als Fehlermeldung ausgelesen werden.
   - Behebung: Eventuell liegen die Punkte außerhalb des Arbeitsbereiches des Roboters. In diesem Fall sollten Punkte gewählt werden, die näher am Ursprung (0|0|0) liegen. 
   - Behebung: Eventuell befindet sich der Roboter in Selbstkollision. In diesem Fall wird ebenfalls keine Lösung der IK gefunden. Lösungsansatz: andere Punkten probieren und die Orientierung ändern, wenn man sich sicher ist, dass die Punkte im Arbeitsbereich liegen.


