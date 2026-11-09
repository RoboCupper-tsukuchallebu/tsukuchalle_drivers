#!/bin/sh
set -eu
root=$(CDPATH='' cd -- "$(dirname "$0")/../.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
cat > "$tmp/docker" <<'MOCK'
#!/bin/sh
case $1 in
  ps) [ "${FAIL_PS:-0}" = 0 ] || exit 1; printf '%s\n' "${MOCK_PS:-}" ;;
  run) printf '%s\n' "$@" > "$MOCK_LOG" ;;
  *) exit 1 ;;
esac
MOCK
chmod +x "$tmp/docker"
PATH="$tmp:$PATH"; export PATH
MOCK_LOG="$tmp/run"; export MOCK_LOG
script=$root/ros2/tools/dds_shm_clean.sh
for row in "$(printf 'ros\tddt-can-bridge-ros2:latest\t')" "$(printf 'other\tcustom-image\tother-project')" "$(printf 'ros\tregistry/tsukuchalle-velodyne:tag\t')" "$(printf 'tools\ttsukuchalle-ros-tools\t')"; do
  if MOCK_PS=$row "$script" > "$tmp/out" 2>&1; then echo 'Expected refusal' >&2; exit 1; fi
  grep -F "$(printf '%s' "$row" | cut -f1)" "$tmp/out" >/dev/null
  test ! -e "$MOCK_LOG"
done
if FAIL_PS=1 "$script" > /dev/null 2>&1; then echo 'Expected docker failure' >&2; exit 1; fi
"$script" --dry-run > "$tmp/out"
test ! -e "$MOCK_LOG"
grep -F '1000:1000' "$tmp/out" >/dev/null
TSUKUCHALLE_UID=0 TSUKUCHALLE_GID=42 "$script"
printf '%s\n' run --rm --ipc host --pid host --user 0:42 -e TSUKUCHALLE_UID=0 tsukuchalle-ros-tools fastdds shm clean > "$tmp/expected"
diff -u "$tmp/expected" "$MOCK_LOG"
echo 'dds_shm_clean tests passed'
