#!/usr/bin/env bash
set -euo pipefail

readonly DISPLAY_NUMBER=':1'
readonly VNC_PORT='5901'
readonly NOVNC_PORT='6080'
readonly VNC_DIR="${HOME}/.vnc"
readonly XSTARTUP="${VNC_DIR}/xstartup"
readonly X_LOCK="/tmp/.X${DISPLAY_NUMBER#:}-lock"
readonly X_SOCKET="/tmp/.X11-unix/X${DISPLAY_NUMBER#:}"

mkdir -p "${VNC_DIR}"
cat > "${XSTARTUP}" <<'EOF'
#!/usr/bin/env bash

unset WAYLAND_DISPLAY
unset WAYLAND_SOCKET
unset DBUS_SESSION_BUS_ADDRESS
unset SESSION_MANAGER

export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11
export QT_QPA_PLATFORM=xcb

# VNC does not provide a login session, so create a private D-Bus session for
# Xfce. Without it, Xfce falls back to dbus-launch and its settings daemon
# cannot start when that helper is unavailable.
exec dbus-run-session -- startxfce4
EOF
chmod 700 "${XSTARTUP}"

# Interrupted container shutdowns can leave stale display locks behind.
if [[ -r "${X_LOCK}" ]]; then
  x_server_pid="$(tr -cd '0-9' < "${X_LOCK}")"
  if [[ -z "${x_server_pid}" ]] || ! kill -0 "${x_server_pid}" 2>/dev/null; then
    rm -f "${X_LOCK}" "${X_SOCKET}"
  fi
fi

if ! tigervncserver -list 2>/dev/null | grep -q "^${DISPLAY_NUMBER}[[:space:]]"; then
  tigervncserver "${DISPLAY_NUMBER}" \
    -localhost yes \
    -rfbport "${VNC_PORT}" \
    -geometry 1600x1000 \
    -depth 24 \
    -SecurityTypes None \
    -xstartup "${XSTARTUP}"
fi

if ! pgrep -f "[w]ebsockify.*${NOVNC_PORT}.*${VNC_PORT}" >/dev/null; then
  setsid -f websockify --web=/usr/share/novnc \
    "127.0.0.1:${NOVNC_PORT}" "127.0.0.1:${VNC_PORT}" \
    > /tmp/novnc.log 2>&1 < /dev/null
fi
