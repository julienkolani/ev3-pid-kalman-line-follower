# EV3 PID Kalman Line Follower

Robot suiveur de ligne EV3 Lego en Pybricks MicroPython avec trois stratégies de contrôle de complexité croissante.

## Présentation

Implémentation de la régulation sur robot réel avec fusion de capteurs (capteur couleur + gyroscope). Trois méthodes de suivi de position en parallèle pour comparaison et validation. Enregistrement de 15+ métriques par cycle.

## Stratégies de contrôle

- **Bang-Bang** — Contrôle à seuil simple
- **PID** — Correcteur Proportionnel-Intégral-Dérivé (Kp=1.2, Ki=0.1, Kd=0.001) avec anti-windup et adaptation de vitesse en courbe
- **Filtre de Kalman** — Estimation de position par fusion odométrie + gyroscope

## Stack technique

- Pybricks MicroPython
- LEGO Mindstorms EV3 (brique, capteur couleur, gyroscope, ultrason)

## Structure

- `tp/TP1-TP2/` — Implémentations Bang-Bang et PID
- `tp/tp3_kalman/` — Filtre de Kalman et suivi de position avancé
- `td/` — Exercices de calibration et vue image

## Déploiement

Déposer les fichiers Python sur la brique EV3 via l'IDE Pybricks ou l'extension VS Code.
