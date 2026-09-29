#!/usr/bin/env bash
set -uo pipefail

usage()
{
  echo "usage: $0 --display HOST:DISPLAY [--config FILE] [--duration SECONDS] [--software] [--gl-version VERSION] [--evidence-root DIR]" >&2
}

display=""
config=""
duration=30
software=0
gl_version=""
evidence_root="rviz-x11-diagnostics"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --display) display="$2"; shift 2 ;;
    --config) config="$2"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    --software) software=1; shift ;;
    --gl-version) gl_version="$2"; shift 2 ;;
    --evidence-root) evidence_root="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

if [ -z "$display" ] || ! [[ "$duration" =~ ^[1-9][0-9]*$ ]]; then
  usage
  exit 2
fi

if [ -z "$config" ]; then
  share="$(ros2 pkg prefix --share pharmarobot_slam 2>/dev/null)" || {
    echo "pharmarobot_slam is not available in the sourced environment" >&2
    exit 2
  }
  config="$share/rviz/diagnostic_minimal.rviz"
fi

if [ ! -r "$config" ]; then
  echo "RViz config is not readable: $config" >&2
  exit 2
fi

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
evidence="$evidence_root/rviz-x11-$stamp"
mkdir -p "$evidence"
log="$evidence/rviz.stdout-stderr.log"
samples="$evidence/samples.tsv"

export DISPLAY="$display"
export QT_X11_NO_MITSHM=1
if [ "$software" -eq 1 ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  unset LIBGL_ALWAYS_SOFTWARE
fi
if [ -n "$gl_version" ]; then
  export MESA_GL_VERSION_OVERRIDE="$gl_version"
else
  unset MESA_GL_VERSION_OVERRIDE
fi

host="${display%%:*}"
display_tail="${display#*:}"
display_number="${display_tail%%.*}"
if ! [[ "$display_number" =~ ^[0-9]+$ ]]; then
  echo "DISPLAY must use HOST:NUMBER form" >&2
  exit 2
fi
port=$((6000 + display_number))

{
  echo "started_utc=$stamp"
  echo "display=$DISPLAY"
  echo "x11_host=$host"
  echo "x11_port=$port"
  echo "config=$config"
  echo "duration_s=$duration"
  echo "QT_X11_NO_MITSHM=$QT_X11_NO_MITSHM"
  echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE-}"
  echo "MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE-}"
  env | grep -E '^(DISPLAY|QT_|LIBGL|MESA|ROS_|RMW_)' | sort
  dpkg-query -W -f='${Package} ${Version}\n' ros-humble-rviz2 libgl1-mesa-dri libglx-mesa0 libqt5gui5 libxcb1 2>&1
  if command -v glxinfo >/dev/null 2>&1; then
    glxinfo -B 2>&1
  else
    echo "glxinfo=unavailable"
  fi
} >"$evidence/environment-renderer.txt"

x11_check()
{
  timeout 2 bash -c "exec 3<>/dev/tcp/$host/$port" >/dev/null 2>&1
}

if x11_check; then
  x11_before=0
else
  x11_before=$?
fi

rviz2 -d "$config" >"$log" 2>&1 &
pid=$!
echo -e "elapsed_s\tpid_alive\tx11_rc\tstat\tcpu\tmem" >"$samples"
elapsed=0
while [ "$elapsed" -lt "$duration" ] && kill -0 "$pid" 2>/dev/null; do
  if x11_check; then xrc=0; else xrc=$?; fi
  process="$(ps -p "$pid" -o stat=,%cpu=,%mem= 2>/dev/null)"
  echo -e "$elapsed\t1\t$xrc\t$process" >>"$samples"
  sleep 1
  elapsed=$((elapsed + 1))
done

if kill -0 "$pid" 2>/dev/null; then
  lifetime="alive_after_${duration}s"
  kill -INT "$pid"
else
  lifetime="exited_after_${elapsed}s"
fi

for _ in 1 2 3 4 5; do
  kill -0 "$pid" 2>/dev/null || break
  sleep 1
done
if kill -0 "$pid" 2>/dev/null; then
  kill -TERM "$pid"
fi
for _ in 1 2 3 4 5; do
  kill -0 "$pid" 2>/dev/null || break
  sleep 1
done
if kill -0 "$pid" 2>/dev/null; then
  kill -KILL "$pid"
fi
wait "$pid"
rviz_exit=$?

if x11_check; then x11_after=0; else x11_after=$?; fi
{
  echo "lifetime=$lifetime"
  echo "rviz_exit=$rviz_exit"
  echo "x11_before_rc=$x11_before"
  echo "x11_after_rc=$x11_after"
  echo "evidence=$evidence"
} | tee "$evidence/summary.txt"

exit 0
