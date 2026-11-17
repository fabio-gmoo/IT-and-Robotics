#!/bin/bash
# Lanza PongBot desde la carpeta del repo (las rutas del YAML son relativas a ella).
# Guarda la salida en logs/pongbot_<fecha>.log para poder revisar errores.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DIR" || exit 1
mkdir -p logs
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# Compila si falta el ejecutable
[ -x bin/pongbot ] || make pongbot >> logs/build.log 2>&1
exec ./bin/pongbot "$@" >> "logs/pongbot_$(date +%Y%m%d_%H%M%S).log" 2>&1
