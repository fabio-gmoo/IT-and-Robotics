#!/bin/bash
# Instala un icono "PongBot" en el escritorio/menú y (opcional) el arranque automático.
#   ./scripts/install_autostart.sh            -> icono + arranque automático al iniciar sesión
#   ./scripts/install_autostart.sh --no-auto  -> solo el icono
#   ./scripts/install_autostart.sh --remove   -> quita todo
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APPS="$HOME/.local/share/applications"
AUTO="$HOME/.config/autostart"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
FILE=pongbot.desktop

if [ "$1" == "--remove" ]; then
    rm -f "$APPS/$FILE" "$AUTO/$FILE" "$DESK/$FILE"
    echo "PongBot quitado del menú, del escritorio y del arranque automático."
    exit 0
fi

chmod +x "$DIR/scripts/pongbot_launcher.sh"
mkdir -p "$APPS" "$AUTO"
cat > "$APPS/$FILE" <<DESKTOP
[Desktop Entry]
Type=Application
Name=PongBot
Comment=PongBot arcade (ECAM LaSalle)
Exec=$DIR/scripts/pongbot_launcher.sh
Path=$DIR
Icon=applications-games
Terminal=false
Categories=Game;
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=5
DESKTOP
chmod +x "$APPS/$FILE"
echo "[OK] Menú de aplicaciones: busca 'PongBot'"

if [ -d "$DESK" ]; then
    cp "$APPS/$FILE" "$DESK/$FILE"
    chmod +x "$DESK/$FILE"
    gio set "$DESK/$FILE" metadata::trusted true 2>/dev/null
    echo "[OK] Icono en el escritorio ($DESK). Si sale 'no confiable': clic derecho -> Permitir lanzar"
fi

if [ "$1" != "--no-auto" ]; then
    cp "$APPS/$FILE" "$AUTO/$FILE"
    echo "[OK] Arranque automático al iniciar sesión (quitar: $0 --remove)"
fi
echo "Logs de cada ejecución: $DIR/logs/"
