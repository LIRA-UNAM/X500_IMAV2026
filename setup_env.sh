#!/bin/bash
echo "Instalando dependencias de Python para el proyecto..."

# Instalar cflib a nivel de usuario
pip install cflib --user --break-system-packages

echo "¡Entorno listo!"