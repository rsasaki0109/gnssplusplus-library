#!/usr/bin/env bash

# Store a Kaggle API access token without accepting it as a command-line
# argument.  The token value is intentionally never printed or logged.

set -Eeuo pipefail
export LC_ALL=C
umask 077

readonly SCRIPT_NAME="${0##*/}"
readonly TARGET_DIR="${KAGGLE_CONFIG_DIR:-$HOME/.kaggle}"
readonly TARGET_FILE="$TARGET_DIR/access_token"
readonly TEMP_TEMPLATE="$TARGET_DIR/.access_token.tmp.XXXXXXXXXX"

force=0
stdin_mode=0
check_only=0
temp_file=''

die() {
  printf '%s: %s\n' "$SCRIPT_NAME" "$1" >&2
  exit 1
}

usage() {
  cat >&2 <<'EOF'
Usage:
  configure_kaggle_token.sh [--force]
  configure_kaggle_token.sh --stdin [--force]
  configure_kaggle_token.sh --check

Options:
  --stdin  Read exactly one token line from stdin. Required when stdin is not a TTY.
  --force  Permit replacing an existing access_token without confirmation.
  --check  Verify only that the target directory/file exist with modes 700/600.
  --help   Show this help.

The token is stored at ${KAGGLE_CONFIG_DIR:-$HOME/.kaggle}/access_token.
The token must be non-empty and contain no whitespace or control characters.
No network request is made.
EOF
}

cleanup() {
  if [[ -n "$temp_file" && -e "$temp_file" ]]; then
    rm -f -- "$temp_file"
  fi
}

trap cleanup EXIT HUP INT TERM

while (($# > 0)); do
  case "$1" in
    --stdin)
      stdin_mode=1
      ;;
    --force)
      force=1
      ;;
    --check)
      check_only=1
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      die "unknown option or positional argument"
      ;;
  esac
  shift
done

if ((check_only)); then
  ((stdin_mode == 0)) || die "--check cannot be combined with --stdin"
  ((force == 0)) || die "--check cannot be combined with --force"
fi

mode_of() {
  local path=$1
  stat -c '%a' -- "$path" 2>/dev/null || die "cannot inspect target permissions"
}

check_target() {
  [[ -d "$TARGET_DIR" && ! -L "$TARGET_DIR" ]] || die "target directory is missing or not a directory"
  [[ "$(mode_of "$TARGET_DIR")" == '700' ]] || die "target directory must have mode 700"
  [[ -f "$TARGET_FILE" && ! -L "$TARGET_FILE" ]] || die "target access_token file is missing or not a regular file"
  [[ "$(mode_of "$TARGET_FILE")" == '600' ]] || die "target access_token file must have mode 600"
  [[ "$(stat -c '%s' -- "$TARGET_FILE" 2>/dev/null)" -gt 0 ]] || die "target access_token file is empty"
}

if ((check_only)); then
  check_target
  printf 'Kaggle token target exists with directory mode 700 and file mode 600.\n'
  exit 0
fi

if ((stdin_mode == 0)) && [[ ! -t 0 ]]; then
  die "non-interactive input requires explicit --stdin"
fi

if [[ -e "$TARGET_DIR" && ! -d "$TARGET_DIR" ]]; then
  die "target path exists but is not a directory"
fi
if [[ -L "$TARGET_DIR" ]]; then
  die "target directory must not be a symbolic link"
fi

mkdir -p -- "$TARGET_DIR"
chmod 700 -- "$TARGET_DIR"

if [[ -L "$TARGET_FILE" ]]; then
  die "target access_token must not be a symbolic link"
fi
if [[ -e "$TARGET_FILE" && ! -f "$TARGET_FILE" ]]; then
  die "target access_token exists but is not a regular file"
fi

if [[ -e "$TARGET_FILE" && $force -eq 0 ]]; then
  if ((stdin_mode)) || [[ ! -t 0 ]]; then
    die "target access_token already exists; use --force"
  fi
  if ! read -r -p 'Replace existing Kaggle token? [y/N] ' answer; then
    printf '\n' >&2
    die "overwrite confirmation was not received"
  fi
  printf '\n' >&2
  case "$answer" in
    y|Y|yes|YES|Yes)
      ;;
    *)
      die "existing token was not replaced"
      ;;
  esac
fi

read_stdin_token() {
  local value=''
  local extra=''
  # A final newline is the stdin record delimiter. Any second line, including
  # a blank line, is rejected so embedded/trailing newlines cannot be stored.
  IFS= read -r value || true
  if IFS= read -r extra; then
    die "stdin must contain exactly one token line"
  fi
  # `read` returns non-zero for a final unterminated second line, so checking
  # only its status would accept `token<newline>extra`.
  [[ -z "$extra" ]] || die "stdin must contain exactly one token line"
  token=$value
}

read_interactive_token() {
  if ! IFS= read -r -s -p 'Kaggle API token: ' token; then
    printf '\n' >&2
    die "token input was not received"
  fi
  printf '\n' >&2
  local confirmation=''
  if ! IFS= read -r -s -p 'Repeat Kaggle API token: ' confirmation; then
    printf '\n' >&2
    die "token confirmation was not received"
  fi
  printf '\n' >&2
  [[ "$token" == "$confirmation" ]] || die "token confirmation did not match"
  unset confirmation
}

token=''
if ((stdin_mode)); then
  if [[ -t 0 ]]; then
    # Explicit --stdin on a terminal is still interactive; keep the token
    # hidden while honoring the caller's explicit input-mode choice.
    read_interactive_token
  else
    read_stdin_token
  fi
else
  read_interactive_token
fi

[[ -n "$token" ]] || die "token must not be empty"
[[ "$token" != *[[:space:]]* ]] || die "token must not contain whitespace"
[[ "$token" != *[[:cntrl:]]* ]] || die "token must not contain control characters"

temp_file=$(mktemp -- "$TEMP_TEMPLATE") || die "cannot create a temporary token file"
chmod 600 -- "$temp_file"
printf '%s' "$token" > "$temp_file" || die "cannot write the temporary token file"
unset token

# The temporary file is in the target directory, so rename is atomic on the
# same filesystem. The old file is replaced only after all validation/writes
# above have succeeded.
mv -f -- "$temp_file" "$TARGET_FILE" || die "cannot atomically install the token file"
temp_file=''
chmod 600 -- "$TARGET_FILE"

check_target
printf 'Kaggle token saved; target exists with directory mode 700 and file mode 600.\n'
