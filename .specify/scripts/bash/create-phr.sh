#!/bin/bash

# Default PHR template content
PHR_TEMPLATE='---
id: {{ID}}
title: {{TITLE}}
stage: {{STAGE}}
date_iso: {{DATE_ISO}}
surface: agent
model: claude-sonnet-4-5-20250929
feature: {{FEATURE}}
branch: {{BRANCH}}
user: user
command: {{COMMAND}}
labels: [{{LABELS}}]
links:
  spec: {{LINKS_SPEC}}
  ticket: {{LINKS_TICKET}}
  adr: {{LINKS_ADR}}
  pr: {{LINKS_PR}}
files_yaml:
{{FILES_YAML}}
tests_yaml:
{{TESTS_YAML}}
---
### Prompt:

```text
{{PROMPT_TEXT}}
```

### Response:

{{RESPONSE_TEXT}}
'

TITLE=""
STAGE=""
FEATURE="none"
COMMAND=""
LABELS=""
LINKS_SPEC="null"
LINKS_TICKET="null"
LINKS_ADR="null"
LINKS_PR="null"
FILES_YAML=""
TESTS_YAML=""
PROMPT_TEXT=""
RESPONSE_TEXT=""
PROMPT_TEXT_ESCAPED=""
RESPONSE_TEXT_ESCAPED=""
PROMPT_FILE=""
RESPONSE_FILE=""
JSON_OUTPUT=false

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --title) TITLE="$2"; shift ;;
        --stage) STAGE="$2"; shift ;;
        --feature) FEATURE="$2"; shift ;;
        --json) JSON_OUTPUT=true ;;
        --command) COMMAND="$2"; shift ;;
        --labels) LABELS="$2"; shift ;;
        --links-spec) LINKS_SPEC="$2"; shift ;;
        --links-ticket) LINKS_TICKET="$2"; shift ;;
        --links-adr) LINKS_ADR="$2"; shift ;;
        --links-pr) LINKS_PR="$2"; shift ;;
        --files-yaml) FILES_YAML="$2"; shift ;;
        --tests-yaml) TESTS_YAML="$2"; shift ;;
        --prompt-text) PROMPT_TEXT="$2"; shift ;;
        --response-text) RESPONSE_TEXT="$2"; shift ;;
        --prompt-file) PROMPT_FILE="$2"; shift ;;
        --response-file) RESPONSE_FILE="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

if [ -z "$TITLE" ] || [ -z "$STAGE" ]; then
    echo "Usage: $0 --title <title> --stage <stage> [--feature <name>] [--json]"
    exit 1
fi

SLUG=$(echo "$TITLE" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9\s-]//g' | sed 's/[\s_-]\+/-/g' | sed 's/^-*//g' | sed 's/-*$//g')
DATE_ISO=$(date +%Y-%m-%d)
BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")

PHR_DIR="history/prompts/${STAGE}"
if [ "$STAGE" == "constitution" ]; then
    PHR_DIR="history/prompts/constitution"
elif [ "$STAGE" == "general" ]; then
    PHR_DIR="history/prompts/general"
elif [ "$FEATURE" != "none" ]; then
    PHR_DIR="history/prompts/${FEATURE}"
fi

mkdir -p "$PHR_DIR"

ID=1
FILE_PATH="${PHR_DIR}/${ID}-${SLUG}.${STAGE}.prompt.md"
while [ -f "$FILE_PATH" ]; do
    ID=$((ID+1))
    FILE_PATH="${PHR_DIR}/${ID}-${SLUG}.${STAGE}.prompt.md"
done

if [ -n "$PROMPT_FILE" ]; then
    PROMPT_TEXT=$(cat "$PROMPT_FILE")
fi
if [ -n "$RESPONSE_FILE" ]; then
    RESPONSE_TEXT=$(cat "$RESPONSE_FILE")
fi

# PROMPT_TEXT and RESPONSE_TEXT will be inserted directly later

PHR_CONTENT=$(cat <<EOF
$PHR_TEMPLATE
EOF
)

PHR_CONTENT=$(echo "$PHR_CONTENT" | \
    sed "s#{{ID}}#$ID#g" | \
    sed "s#{{TITLE}}#$TITLE#g" | \
    sed "s#{{STAGE}}#$STAGE#g" | \
    sed "s#{{DATE_ISO}}#$DATE_ISO#g" | \
    sed "s#{{FEATURE}}#$FEATURE#g" | \
    sed "s#{{BRANCH}}#$BRANCH#g" | \
    sed "s#{{COMMAND}}#$COMMAND#g" | \
    sed "s#{{LABELS}}#$LABELS#g" | \
    sed "s#{{LINKS_SPEC}}#$LINKS_SPEC#g" | \
    sed "s#{{LINKS_TICKET}}#$LINKS_TICKET#g" | \
    sed "s#{{LINKS_ADR}}#$LINKS_ADR#g" | \
    sed "s#{{LINKS_PR}}#$LINKS_PR#g" | \
    sed "s#{{FILES_YAML}}#$FILES_YAML#g" | \
    sed "s#{{TESTS_YAML}}#$TESTS_YAML#g")

# Insert PROMPT_TEXT and RESPONSE_TEXT using awk
PHR_CONTENT=$(echo "$PHR_CONTENT" | awk -v prompt_text="$PROMPT_TEXT" -v response_text="$RESPONSE_TEXT" '    /### Prompt:/ {
        print
        print ""
        print "```text"
        print prompt_text
        print "```"
        next
    }
    /### Response:/ {
        print
        print ""
        print response_text
        print ""
        next
    }
    { print }'
)

echo "$PHR_CONTENT" > "$FILE_PATH"

if $JSON_OUTPUT; then
    echo "{\"id\": \"$ID\", \"path\": \"$FILE_PATH\", \"stage\": \"$STAGE\", \"title\": \"$TITLE\"}"
else
    echo "ID: $ID"
    echo "Path: $FILE_PATH"
    echo "Stage: $STAGE"
    echo "Title: $TITLE"
fi
