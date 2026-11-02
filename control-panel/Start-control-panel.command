#!/bin/zsh
cd "${0:A:h}"
if curl --silent --fail http://127.0.0.1:8775/api/ports >/dev/null; then
  open http://127.0.0.1:8775
  exit 0
fi
python3 server.py &
panel_pid=$!
trap 'kill "$panel_pid" 2>/dev/null' EXIT INT TERM
for attempt in {1..30}; do
  if curl --silent --fail http://127.0.0.1:8775/api/ports >/dev/null; then
    open http://127.0.0.1:8775
    echo 'Glove control panel is running. Keep this window open.'
    wait "$panel_pid"
    exit $?
  fi
  sleep 0.2
done
echo 'Could not start the control panel. Check the error above.'
read '?Press Return to close.'
