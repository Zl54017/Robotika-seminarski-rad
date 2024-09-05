# Praktikum robotike - seminarski rad
 
Ovaj projekt opisuje proces izrade robota za prepoznavanje i povezivanje objekata. <br>
Projekt je izrađen unutar Matlaba i Autodesk Fusiona.<br>
    Pomoću Autodesk Fusiona izrađeni su dijelovi robota te su printani 3D printerom. <br>
    Korisničko sučelje, kinematike i pathPlanning alogritam izrađeni su unutar Matlaba.<br>

## Opis rada robota

Robot se pokreće pomoću korisničkog sučelja prikazan na Slici 1. Unutar korisničkog sučelja postoje 3 načina rada robota: isključen, ručni i automatski način. Ručni način rada omogućuje kontroliranje kretnji robota pomoću kutova 3 ključna zgloba. Automatski način rada omogućuje kontroliranje kretnji robota pomoću koordinata koje se računaju prema položaju kamere koja je spojena na računalo. Za postavljanje kamere koriste se kalibracijske matrice i konstante. Osim kontroliranja robota, automatski način rada omogućuje traženje puta od točke do točke. Za detekciju slika koristio se ugrađeni Matlab modul YOLOv4 te RRT algoritam za pronalaz puta. <br>
Primjer rada robota prikazan je unutar PowerPoint prezentacije.


<p align="center">
<img src="Slika1" width="750"/><br>
<i>Slika 1: Prikaz korisničkog sučelja</i>
</p>