# X500_IMAV2026
Software repository for Pumas Drone X500 V2 Team, for the IMAV 2026.

## Para clonar este repositorio 

```
git clone --recursive https://github.com/LIRA-UNAM/X500_IMAV2026.git
```

## SI ya lo tienes clonado
```
git submodule update --init --recursive
```

## EStructura principal del repositorio

```
X500_IMAV2026/
├── .gitmodules
├── PX4-Autopilot/        ← submodule PX4
├── src/
│   ├── px4_msgs/         ← submodule px4_msgs
│   ├── imav_mission/     ← control, misiones
│   └── imav_perception/  ← visión, detección
├── scripts/              ← setup, launch scripts
├── config/               ← parámetros del drone
└── README.md
```
