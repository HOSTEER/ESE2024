# ESE2024
    Emilie Butruille
    Louis Liénart
    Théo Césari
    Victor Lesne
------
Activer doxygen dans les propriétés du projet -> général C/C++, tuto doxygen https://www.youtube.com/watch?v=GC9Xy7nLxyw

pour retirer un fichier "add" par erreur faire un "git reset nom_du_fichier", un reset seul retirera tout les add fait

Convention à avoir pour le stockage des sources :
pour stocker les lib créer faire un dossier source "Prod" avec comme sous fichier "Inc" et "Src"
Ensuite les ajouter dans "Project" -> "Properties" -> "C/C++ General" -> "Paths and Symbols" -> "add" -> entrer "Prod/Inc" et fair ok

Pour vérifier qu'une branch est ok cloner un 2nd repo de la branch et la testé pour voir si 0 error, sinon ça veut dire qu'il manque un truc à commit

Ensuite il faut résoudre les conflit entre .cproject ( globalement garder le code du main ); .ioc (globalement fusionner les pin ajoutés et faire gaffe au pin, en double) et enfin le main.c ( où il faut fusionner )

pour importer sur cubeIDE tu fait "File" -> "Import" -> "General" -> "Project from Folder or Archive" selectionner le dirtectory genre moi c'est "C:\Users\louis\Desktop\ST_proj_managing\401RE_try_Git2" et ensuite faire "finish"
Après ça faut regénérer des fichier du code donc tu clique sur le ioc et ensuite sur "Device configuration tool Code generation"
