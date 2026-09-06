#!/usr/bin/env bash
# Upload the working copy to the VM, normalising line endings on the way.
#
# sync_from_vm.sh pulls; this pushes. The direction matters because the Windows
# working copy is not always what git stores: .gitattributes forces LF into the
# repository and on checkout, but an editor - or a script writing through
# Python's text mode - can leave CRLF in the file that actually gets copied.
# bash on the guest then reads a trailing carriage return as part of the
# filename or the option, and reports things like env.sh missing when it is
# present.
set -euo pipefail
VM="${VM:-nostalgia@127.0.0.1}"
PORT="${PORT:-2224}"
WS="${WS:-ros2_ws_course_repro/ros2_ws/src}"
EXPDIR_REMOTE="${EXPDIR_REMOTE:-exp}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

send() {          # send <local path> <remote path>
  local src="$1" dst="$2" staged="$TMP/$(basename "$1")"
  tr -d '\r' < "$src" > "$staged"
  scp -q -P "$PORT" "$staged" "$VM:$dst"
  echo "  $(basename "$src") -> $dst"
}

echo "experiment scripts -> ~/$EXPDIR_REMOTE"
ssh -p "$PORT" "$VM" "mkdir -p ~/$EXPDIR_REMOTE"
for f in "$ROOT"/tools/experiments/*.sh "$ROOT"/tools/experiments/*.py; do
  send "$f" "~/$EXPDIR_REMOTE/$(basename "$f")"
done
ssh -p "$PORT" "$VM" "chmod +x ~/$EXPDIR_REMOTE/*.sh"

echo "workspace sources -> ~/$WS"
cd "$ROOT/vm_sync/ros2_course_physics/src"
# ssh inside a while-read loop consumes the loop's stdin unless given -n, so
# the first iteration swallowed the remaining file list and only one file was
# ever uploaded.
find . \( -name '*.py' -o -name '*.xml' -o -name '*.yaml' -o -name '*.urdf' -o -name '*.srdf' \) | while read -r rel; do
  rel="${rel#./}"
  ssh -n -p "$PORT" "$VM" "test -e ~/$WS/$rel" || { echo "  skip (not on VM): $rel"; continue; }
  send "$rel" "~/$WS/$rel"
done

echo "verifying no carriage returns survived"
ssh -p "$PORT" "$VM" "! grep -rlU \$'\r' ~/$EXPDIR_REMOTE/ 2>/dev/null | grep . || { echo 'CR found'; exit 1; }"
echo "push complete"
