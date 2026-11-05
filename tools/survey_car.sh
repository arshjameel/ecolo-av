#!/usr/bin/env bash
# Run tools/machine_survey.sh on every car computer, from a laptop, and save each one's
# output in ~/aav-audit/. Read-only on the car (machine_survey.sh says how).
#
# usage: pixi run bash tools/survey_car.sh ~/aav-audit/hosts.txt
#
# hosts.txt: one machine per line, "<label> <user>@<ip> [<keychain entry>]". Lines
# starting with # are skipped, and so is anything after a # on a line.
#   JETSON   aavjetson@192.168.1.140   aav-jetson
#   RTK-PI   pi@192.168.1.31
# Keep hosts.txt and the outputs out of git. Logins come from the team. Never put a
# password in hosts.txt: with a keychain entry, ssh gets the password from this Mac's
# Keychain (tools/keychain_askpass.sh); without one, ssh asks you (or uses your key).
#
# ssh settings: a machine seen for the first time has its host key accepted and kept
# (accept-new); a key that changed is refused. One password try each, so a wrong one
# can't lock the laptop out (fail2ban).
#
# Portable: bash 3.2 (a Mac), no GNU-only flags.
set -u
hosts=${1:?usage: survey_car.sh <hosts.txt>}
here=$(cd "$(dirname "$0")" && pwd)
out=${SURVEY_OUT:-$HOME/aav-audit}
mkdir -p "$out"
chmod 700 "$out"

while read -r label target entry _; do
  case "$label" in ''|'#'*) continue;; esac
  case "$entry" in '#'*) entry='';; esac
  file="$out/$label.txt"
  printf '%-12s %-28s ' "$label" "$target"
  # ssh's input is the script, so it can't swallow the rest of hosts.txt.
  if [[ -n "$entry" ]]; then
    # The password from the Keychain: ssh runs the helper instead of asking.
    AAV_KEYCHAIN_ENTRY="$entry" SSH_ASKPASS="$here/keychain_askpass.sh" \
      SSH_ASKPASS_REQUIRE=force DISPLAY="${DISPLAY:-:0}" \
      ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
        -o NumberOfPasswordPrompts=1 "$target" 'bash -s' \
        < "$here/machine_survey.sh" > "$file" 2>&1
  else
    ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
      -o NumberOfPasswordPrompts=1 "$target" 'bash -s' \
      < "$here/machine_survey.sh" > "$file" 2>&1
  fi
  lines=$(wc -l < "$file" | tr -d ' ')
  if grep -q '==== done' "$file"; then
    echo "OK    $lines lines -> $file"
  else
    echo "PART  $lines lines -> $file (stopped early: read its end)"
  fi
done < "$hosts"

echo
echo "Next: read each file before sharing it (secrets are cut, but check)."
echo "Copy anything that exists only on the car (weights, unpushed code): docs/history/friday-todo-2026-10-16.md, part 4."
