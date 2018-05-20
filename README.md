# ArduinoLPT
Une station digitale et analogique qui contrôle un réseau de démonstration de trains HO

un projet de station DCC et analogique qui contrôle un réseau de démonstration.

Le projet est basé sur un Arduino Uno, un controleur de moteur L298 qui génère les signla DCC en digital 
et un signal PWM pour contrôler les trains en analogique.
Il y a un afficheur Nokia 5510 et un clavier I2C pour l'interface utilisateur ( et un buzzer pour attirer l'attention)
La vitresse des trains est contrôlée par radio depuis une petite télécommande radio basée sur un NRF20L01P.
On utilise des port I2C PCF8584 pour contrôler les entrées sortie : la présence d'un train sur une portion de voie,
le contrôle des aiguillages, les feux sur le réseaux (LED rouge et verte)

On peut voir le réseau en action sur https://www.youtube.com/watch?v=WtapWsqfiKo&feature=youtu.be

Le blog correspondant est : http://elcheapodcc.blogspot.fr/

Le code Arduino est fait de manière à être facilement adaptable à d'autres réseau.

Amusez vous bien ...
